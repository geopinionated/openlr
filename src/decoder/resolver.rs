use std::cmp::Reverse;
use std::fmt::Debug;

use tracing::debug;

use crate::decoder::candidates::{CandidateLine, CandidateLinePair, CandidateLines};
use crate::decoder::route::{CandidateRoute, CandidateRoutes};
use crate::decoder::shortest_path::shortest_path;
use crate::graph::path::{Path, is_path_connected, is_path_loop};
use crate::{DecodeError, DecoderConfig, DirectedGraph, Frc, Length, Offsets};

/// The decoder needs to compute a shortest-path between each pair of subsequent location reference
/// points. For each pair of location reference points suitable candidate lines must be chosen. The
/// candidate line of the first LRPs of this pair acts as start of the shortest-path calculation.
/// The candidate line of the second location reference point of this pair is the end of the
/// shortest-path calculation. If the chosen lines are equal no shortest-path calculation needs to
/// be started.
///
/// The shortest path algorithm should take the part of the network into account which contains all
/// lines having a functional road class lower than or equal to the lowest functional road class of
/// the first location reference point of the pair. This value might be altered if the decoder
/// anticipates having different functional road class values than the encoder map.
///
/// Additionally the shortest-path algorithm should fulfill the following constraints:
/// - All lengths of the lines should be measured in meters and should also be converted to integer
///   values, float values need to be rounded correctly.
/// - The search is node based and will start at the start node of the first line and will end at
///   the end node of the last line.
/// - The algorithm shall return an ordered list of lines representing the calculated shortest-path.
///
/// If no shortest-path can be calculated for two subsequent location reference points, the decoder
/// might try a different pair of candidate lines or finally fail and report an error. If a
/// different pair of candidate lines is tried it might happen that the start line needs to be
/// changed. In such a case this also affects the end line of the previous shortest-path and this
/// path also needs to be re-calculated and checked again. The number of retries of shortest-path
/// calculations should be limited in order to guarantee a fast decoding process.
///
/// After the shortest-path calculation the length of such a path should be checked against the
/// distance to next point information of the first location reference point of a pair. If the
/// length information differ too much the decoder could decide to try a different pair of candidate
/// lines (see also Step – 5) or to fail and report an error.
pub fn resolve_routes<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
    offsets: Offsets,
) -> Result<CandidateRoutes<G::EdgeId>, DecodeError<G::Error>> {
    debug!("Resolving routes for {} LRPs", candidate_lines.len());
    let best_edge = find_best_candidate_edge(candidate_lines);

    if let Some(routes) = best_edge.and_then(|best_edge| {
        resolve_single_line_routes(graph, candidate_lines, best_edge, offsets).transpose()
    }) {
        let routes = routes?;
        debug_assert!(is_path_connected(graph, &routes.to_path())?);
        return Ok(routes);
    }

    let mut routes: CandidateRoutes<_> = Vec::with_capacity(candidate_lines.len() - 1).into();

    for window in candidate_lines.windows(2) {
        let [candidates_lrp1, candidates_lrp2] = [&window[0], &window[1]];
        let routes_count = routes.len();
        let last_candidate_edge = routes.last().map(|route| route.last_candidate_edge());

        let pairs = resolve_top_k_candidate_pairs(
            config,
            candidates_lrp1,
            candidates_lrp2,
            best_edge,
            last_candidate_edge,
        );

        // Find the first candidates pair that can be used to construct a valid route between the
        // two consecutive LRPs, also try to find an alternative route if consecutive best pairs are
        // not connected to each other.
        for candidates in pairs {
            let route = resolve_candidate_route(config, graph, candidates)?
                .map(|route| resolve_alternative_route(config, graph, &mut routes, route))
                .transpose()?
                .flatten();

            if let Some(route) = route {
                let lrp2 = candidates_lrp2.lrp;
                let (pos, neg) = route.calculate_offsets(graph, offsets)?;
                // offsets cut only the start of the first route and the end of the last route
                let pos = if routes.is_empty() { pos } else { Length::ZERO };
                let neg = if lrp2.is_last() { neg } else { Length::ZERO };

                if !is_path_loop(graph, &route.path.edges, pos, neg)? {
                    routes.push(route);
                    break;
                }
            }
        }

        if routes.len() == routes_count {
            return Err(DecodeError::RouteNotFound((
                candidates_lrp1.lrp,
                candidates_lrp2.lrp,
            )));
        }
    }

    debug_assert!(is_path_connected(graph, &routes.to_path())?);
    Ok(routes)
}

/// Returns Some edge only if all the candidate lines have the same edge as best candidate.
/// Otherwise returns None.
fn find_best_candidate_edge<EdgeId: Copy + PartialEq>(
    candidate_lines: &[CandidateLines<EdgeId>],
) -> Option<EdgeId> {
    debug_assert!(candidate_lines.iter().all(|c| !c.lines.is_empty()));
    let mut best_candidates = candidate_lines.iter().filter_map(|c| c.best_candidate());

    let CandidateLine {
        edge: best_edge, ..
    } = best_candidates.next()?;

    if best_candidates.any(|candidate| candidate.edge != best_edge) {
        return None;
    }

    Some(best_edge)
}

/// If all the best candidate lines are equal there is no need to compute top K candidates and
/// their shortest paths, we can just return the best candidate line for each LRP.
fn resolve_single_line_routes<G: DirectedGraph>(
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
    best_edge: G::EdgeId,
    offsets: Offsets,
) -> Result<Option<CandidateRoutes<G::EdgeId>>, DecodeError<G::Error>> {
    debug!("Resolving single line routes on {best_edge:?} with {offsets:?}");

    let pairs = candidate_lines.windows(2).filter_map(|window| {
        Some(CandidateLinePair {
            line_lrp1: window[0].best_candidate()?,
            line_lrp2: window[1].best_candidate()?,
        })
    });

    let routes: Vec<_> = pairs
        .map(|candidates| same_edge_route(graph, candidates))
        .collect::<Result<_, _>>()?;

    let routes: CandidateRoutes<_> = routes.into();
    let (pos_offset, neg_offset) = routes.calculate_offsets(graph, offsets)?;

    if pos_offset + neg_offset >= routes.path_length() {
        debug!("Same line route on {best_edge:?} has invalid offsets");
        return Ok(None);
    }

    debug!("Route resolved on single best edge: {best_edge:?}");
    Ok(Some(routes))
}

fn resolve_candidate_route<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidates: CandidateLinePair<G::EdgeId>,
) -> Result<Option<CandidateRoute<G::EdgeId>>, DecodeError<G::Error>> {
    let CandidateLinePair {
        line_lrp1:
            CandidateLine {
                lrp: lrp1,
                edge: edge_lrp1,
                ..
            },
        line_lrp2:
            CandidateLine {
                lrp: lrp2,
                edge: edge_lrp2,
                distance_to_projection: projection_lrp2,
                ..
            },
    } = candidates;

    if edge_lrp1 == edge_lrp2 {
        debug!("Route found on same edge {edge_lrp1:?}");
        return Ok(Some(same_edge_route(graph, candidates)?));
    }

    // LRP1 lfrcnp (lowest FRC to the next point) encoded up to edge before LRP2, but the shortest
    // path implementation is edge-based and checks include the destination edge (LRP2 first edge)
    let destination_frc = graph.get_edge_frc(edge_lrp2)?;
    let lfrcnp = Frc::from_value(lrp1.lfrcnp().value() + Frc::variance(&lrp1.lfrcnp()));
    let lfrcnp = lfrcnp.unwrap_or(Frc::Frc7).max(destination_frc);

    let max_length = max_route_length(config, graph, &candidates)?;

    debug!("Finding route: {edge_lrp1:?} -> {edge_lrp2:?} (max={max_length} lfrcnp={lfrcnp:?})");

    if let Some(mut path) = shortest_path(graph, edge_lrp1, edge_lrp2, lfrcnp, max_length)? {
        let min_length = lrp1.dnp() - config.next_point_variance;
        let destination_length = graph.get_edge_length(edge_lrp2)?;

        // The path includes the whole destination edge, but the distance between the LRPs ends at
        // the beginning of the destination edge (plus its projection) or, for the last LRP, at the
        // projection of the LRP into the destination edge (or at its end).
        let projection = if let Some(projection) = projection_lrp2 {
            projection
        } else if lrp2.is_last() {
            destination_length
        } else {
            Length::ZERO
        };

        let path_length = path.length - destination_length + projection;

        if path_length < min_length {
            debug!("{path:?} length ({path_length}) is shorter than expected: {min_length}");
            return Ok(None);
        }

        if !lrp2.is_last()
            && let Some(last_edge) = path.edges.pop()
        {
            path.length -= graph.get_edge_length(last_edge)?;
        }

        debug_assert!(!path.edges.is_empty());
        debug_assert!(path.length <= max_length + graph.get_edge_length(edge_lrp2)?);

        debug!("Route found: {edge_lrp1:?} -> {edge_lrp2:?}: {path:?}");
        return Ok(Some(CandidateRoute { path, candidates }));
    }

    debug!("Route not found: {edge_lrp1:?} -> {edge_lrp2:?}");
    Ok(None)
}

/// Updates the last route with an alternative if this cannot be connected to the given new route.
/// Returns the new given route or None if the altenative is needed but cannot be computed.
fn resolve_alternative_route<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    routes: &mut [CandidateRoute<G::EdgeId>],
    new_route: CandidateRoute<G::EdgeId>,
) -> Result<Option<CandidateRoute<G::EdgeId>>, DecodeError<G::Error>> {
    if let Some(last_route) = routes.last_mut() {
        // if the previous route ends on a line that is not the start of this new route
        // then the previous route needs to be re-computed
        if last_route.last_candidate_edge() != new_route.first_candidate_edge() {
            let candidates = CandidateLinePair {
                line_lrp1: last_route.first_candidate(),
                line_lrp2: new_route.first_candidate(),
            };

            if let Some(route) = resolve_candidate_route(config, graph, candidates)? {
                *last_route = route;
            } else {
                return Ok(None);
            }
        }
    }

    Ok(Some(new_route))
}

/// Route between two LRPs whose candidate lines are on the same edge: it contains the edge only
/// when the 2nd LRP is the last one, otherwise the LRPs are connected without traversing any edge.
fn same_edge_route<G: DirectedGraph>(
    graph: &G,
    candidates: CandidateLinePair<G::EdgeId>,
) -> Result<CandidateRoute<G::EdgeId>, G::Error> {
    let path = if candidates.line_lrp2.lrp.is_last() {
        Path {
            length: graph.get_edge_length(candidates.line_lrp1.edge)?,
            edges: vec![candidates.line_lrp1.edge],
        }
    } else {
        Path::default()
    };

    Ok(CandidateRoute { path, candidates })
}

fn max_route_length<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidates: &CandidateLinePair<G::EdgeId>,
) -> Result<Length, DecodeError<G::Error>> {
    let CandidateLinePair { line_lrp1, .. } = candidates;

    let mut max_distance = line_lrp1.lrp.dnp() + config.next_point_variance;

    // Shortest path can only consider full edges, therefore we need to add the complete length when
    // computing the max distance upper bound if the lines were projected, but we can exclude the
    // 2nd LRP since finding the shortest path does not reject the path if the max distance is
    // exceeded right at the destination edge. This is an optimization that allows to keep the
    // search space smaller and avoid exploring too much when the LRP is on very long edges.
    if line_lrp1.is_projected() {
        max_distance += graph.get_edge_length(line_lrp1.edge)?;
    }

    Ok(max_distance.ceil())
}

fn resolve_top_k_candidate_pairs<EdgeId: Debug + Copy + PartialEq>(
    config: &DecoderConfig,
    lines_lrp1: &CandidateLines<EdgeId>,
    lines_lrp2: &CandidateLines<EdgeId>,
    best_single_line_edge: Option<EdgeId>,
    last_candidate_edge: Option<EdgeId>,
) -> Vec<CandidateLinePair<EdgeId>> {
    let max_size = lines_lrp1.lines.len() * lines_lrp2.lines.len();
    let k_size = max_size.min(config.max_number_retries + 1);
    debug!("Resolving candidate pair ratings with K={k_size}");

    let mut candidates = Vec::with_capacity(max_size);

    for &line_lrp1 in &lines_lrp1.lines {
        for &line_lrp2 in &lines_lrp2.lines {
            // discard the candidate line pair when there are multiple top K candidates and the best
            // single line edge exists but was previously not considered valid to form the route
            if let Some(best_edge) = best_single_line_edge
                && k_size > 1
                && line_lrp1.edge == line_lrp2.edge
                && line_lrp1.edge == best_edge
            {
                debug!("Discarding best single line edge {best_edge:?} from top K candidates");
                continue;
            }

            candidates.push(CandidateLinePair {
                line_lrp1,
                line_lrp2,
            });
        }
    }

    candidates.sort_by_cached_key(|pair| {
        Reverse(pair.rating(
            config.same_line_degradation,
            config.connected_route_boost,
            last_candidate_edge,
        ))
    });
    candidates.truncate(k_size);

    debug!(
        "Top K candidates: {:?}",
        candidates
            .iter()
            .map(|pair| (
                pair.line_lrp1.edge,
                pair.line_lrp2.edge,
                pair.rating(
                    config.same_line_degradation,
                    config.connected_route_boost,
                    last_candidate_edge,
                )
            ))
            .collect::<Vec<_>>()
    );

    candidates
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::decoder::candidates::CandidateLine;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
    use crate::model::RatingScore;
    use crate::{Bearing, Coordinate, Fow, LineAttributes, Offset, PathAttributes, Point};

    #[test]
    fn decoder_resolve_top_k_candidate_pairs_001() {
        let config = DecoderConfig {
            max_number_retries: 3,
            ..Default::default()
        };

        let line1 = CandidateLine {
            lrp: Point::default(),
            edge: 1,
            distance_to_projection: None,
            rating: RatingScore::from(926.3),
        };

        let line2 = CandidateLine {
            lrp: Point::default(),
            edge: 2,
            distance_to_projection: Some(Length::from_meters(141.6)),
            rating: RatingScore::from(880.4),
        };

        let line3 = CandidateLine {
            lrp: Point::default(),
            edge: 3,
            distance_to_projection: None,
            rating: RatingScore::from(924.9),
        };

        let line4 = CandidateLine {
            lrp: Point::default(),
            edge: 4,
            distance_to_projection: None,
            rating: RatingScore::from(100.0),
        };

        let line5 = CandidateLine {
            lrp: Point::default(),
            edge: 5,
            distance_to_projection: None,
            rating: RatingScore::from(10.0),
        };

        let pairs = resolve_top_k_candidate_pairs(
            &config,
            &CandidateLines {
                lrp: Point::default(),
                lines: vec![line1, line2],
            },
            &CandidateLines {
                lrp: Point::default(),
                lines: vec![line3, line4, line5],
            },
            None,
            None,
        );

        assert_eq!(
            pairs,
            [
                CandidateLinePair {
                    line_lrp1: line1,
                    line_lrp2: line3
                },
                CandidateLinePair {
                    line_lrp1: line2,
                    line_lrp2: line3
                },
                CandidateLinePair {
                    line_lrp1: line1,
                    line_lrp2: line4
                },
                CandidateLinePair {
                    line_lrp1: line2,
                    line_lrp2: line4
                }
            ]
        );
    }

    #[test]
    fn decoder_resolve_top_k_candidate_pairs_002() {
        let config = DecoderConfig {
            max_number_retries: 3,
            connected_route_boost: 0.10,
            ..Default::default()
        };

        let line1 = CandidateLine {
            lrp: Point::default(),
            edge: 1,
            distance_to_projection: None,
            rating: RatingScore::from(105.0),
        };

        let line2 = CandidateLine {
            lrp: Point::default(),
            edge: 2,
            distance_to_projection: None,
            rating: RatingScore::from(100.0),
        };

        let line3 = CandidateLine {
            lrp: Point::default(),
            edge: 3,
            distance_to_projection: None,
            rating: RatingScore::from(100.0),
        };

        let lines_lrp1 = CandidateLines {
            lrp: Point::default(),
            lines: vec![line1, line2],
        };
        let lines_lrp2 = CandidateLines {
            lrp: Point::default(),
            lines: vec![line3],
        };

        let pairs = resolve_top_k_candidate_pairs(&config, &lines_lrp1, &lines_lrp2, None, None);

        // Line 1 rates 5% above line 2, but line 2 is where the previous route ended and the
        // increase is 10%, so line 2 goes first. Without a previous route the order is given only
        // by the ratings.
        assert_eq!(
            pairs,
            [
                CandidateLinePair {
                    line_lrp1: line1,
                    line_lrp2: line3
                },
                CandidateLinePair {
                    line_lrp1: line2,
                    line_lrp2: line3
                },
            ]
        );

        let pairs = resolve_top_k_candidate_pairs(&config, &lines_lrp1, &lines_lrp2, None, Some(2));
        assert_eq!(
            pairs,
            [
                CandidateLinePair {
                    line_lrp1: line2,
                    line_lrp2: line3
                },
                CandidateLinePair {
                    line_lrp1: line1,
                    line_lrp2: line3
                },
            ]
        );
    }

    #[test]
    fn decoder_resolve_routes_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig {
            max_node_distance: Length::from_meters(100.0),
            max_bearing_difference: Bearing::from_degrees(90),
            min_line_rating: RatingScore::from(800.0),
            ..Default::default()
        };

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46112,
                lat: 52.51711,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(381.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46284,
                lat: 52.51500,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(17),
            },
            path: None,
        };

        let line1_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(926.3),
            distance_to_projection: None,
        };

        let line2_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(4925291),
            rating: RatingScore::from(880.4),
            distance_to_projection: Some(Length::from_meters(141.6)),
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(109783),
            rating: RatingScore::from(924.9),
            distance_to_projection: None,
        };

        let candidate_lines = [
            CandidateLines {
                lrp: first_lrp,
                lines: vec![line1_first_lrp, line2_first_lrp],
            },
            CandidateLines {
                lrp: last_lrp,
                lines: vec![line_last_lrp],
            },
        ];

        let routes = resolve_routes(&config, graph, &candidate_lines, Offsets::default()).unwrap();
        assert_eq!(routes.len(), 1);

        assert_eq!(
            routes[0],
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                    length: Length::from_meters(379.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line1_first_lrp,
                    line_lrp2: line_last_lrp
                }
            }
        );
    }

    #[test]
    fn decoder_resolve_routes_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig::default();

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4615506,
                lat: 52.5170544,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(70.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4625506,
                lat: 52.5168944,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(287),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1128.7),
            distance_to_projection: Some(Length::from_meters(29.0)),
        };

        let line1_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1122.7),
            distance_to_projection: Some(Length::from_meters(99.0)),
        };

        let line2_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(4925291),
            rating: RatingScore::from(900.0),
            distance_to_projection: None,
        };

        let candidate_lines = [
            CandidateLines {
                lrp: first_lrp,
                lines: vec![line_first_lrp],
            },
            CandidateLines {
                lrp: last_lrp,
                lines: vec![line1_last_lrp, line2_last_lrp],
            },
        ];

        let routes = resolve_routes(&config, graph, &candidate_lines, Offsets::default()).unwrap();
        assert_eq!(routes.len(), 1);

        assert_eq!(
            routes[0],
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174)],
                    length: Length::from_meters(136.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_first_lrp,
                    line_lrp2: line1_last_lrp
                }
            }
        );
    }

    #[test]
    fn decoder_resolve_routes_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig::default();

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4615506,
                lat: 52.5170544,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(70.0),
            }),
        };

        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4625506,
                lat: 52.5168944,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(280.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46284,
                lat: 52.51500,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(17),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1128.7),
            distance_to_projection: Some(Length::from_meters(29.0)),
        };

        let line_second_lrp = CandidateLine {
            lrp: second_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1122.7),
            distance_to_projection: Some(Length::from_meters(99.0)),
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(109783),
            rating: RatingScore::from(924.9),
            distance_to_projection: None,
        };

        let candidate_lines = [
            CandidateLines {
                lrp: first_lrp,
                lines: vec![line_first_lrp],
            },
            CandidateLines {
                lrp: second_lrp,
                lines: vec![line_second_lrp],
            },
            CandidateLines {
                lrp: last_lrp,
                lines: vec![line_last_lrp],
            },
        ];

        let routes = resolve_routes(&config, graph, &candidate_lines, Offsets::default()).unwrap();
        assert_eq!(routes.len(), 2);

        assert_eq!(
            routes[0],
            CandidateRoute {
                path: Path {
                    edges: vec![], // first and second LRPs are on the same line
                    length: Length::ZERO,
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_first_lrp,
                    line_lrp2: line_second_lrp
                }
            }
        );

        assert_eq!(
            routes[1],
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                    length: Length::from_meters(379.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_second_lrp,
                    line_lrp2: line_last_lrp
                }
            }
        );
    }

    #[test]
    fn decoder_resolve_routes_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig::default();

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46112,
                lat: 52.51711,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(107),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(381.0),
            }),
        };

        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.46284,
                lat: 52.51500,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(197),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(45.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4632200,
                lat: 52.5147507,
            },
            line: LineAttributes {
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(280),
            },
            path: None,
        };

        let line1_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(8717174),
            rating: RatingScore::from(1194.8),
            distance_to_projection: None,
        };

        let line2_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(4925291),
            rating: RatingScore::from(1135.3),
            distance_to_projection: Some(Length::from_meters(142.0)),
        };

        let line1_second_lrp = CandidateLine {
            lrp: second_lrp,
            edge: EdgeId(6770340),
            rating: RatingScore::from(1193.5),
            distance_to_projection: None,
        };

        let line2_second_lrp = CandidateLine {
            lrp: second_lrp,
            edge: EdgeId(109783),
            rating: RatingScore::from(1137.7),
            distance_to_projection: Some(Length::from_meters(191.0)),
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(7531947),
            rating: RatingScore::from(1176.0),
            distance_to_projection: None,
        };

        let candidate_lines = [
            CandidateLines {
                lrp: first_lrp,
                lines: vec![line1_first_lrp, line2_first_lrp],
            },
            CandidateLines {
                lrp: second_lrp,
                lines: vec![line1_second_lrp, line2_second_lrp],
            },
            CandidateLines {
                lrp: last_lrp,
                lines: vec![line_last_lrp],
            },
        ];

        let routes = resolve_routes(&config, graph, &candidate_lines, Offsets::default()).unwrap();
        assert_eq!(routes.len(), 2);

        assert_eq!(
            routes[0],
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                    length: Length::from_meters(379.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line1_first_lrp,
                    line_lrp2: line1_second_lrp
                }
            }
        );

        assert_eq!(
            routes[1],
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(6770340), EdgeId(7531947)],
                    length: Length::from_meters(53.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line1_second_lrp,
                    line_lrp2: line_last_lrp
                }
            },
        );
    }

    #[test]
    fn decoder_resolve_routes_005() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig::default();

        // node-11
        // line-7292030
        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4571122,
                lat: 52.5177995,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(20),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::ZERO,
            }),
        };

        // node-110
        // line-7292029
        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4576677,
                lat: 52.518717,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(20),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::ZERO,
            }),
        };

        // node-21
        // line-7292028
        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4581169,
                lat: 52.5194882,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(200),
            },
            path: None,
        };

        let line_first_lrp = CandidateLine {
            lrp: first_lrp,
            edge: EdgeId(-7292030),
            rating: RatingScore::from(1000.0),
            distance_to_projection: None,
        };

        let line1_second_lrp = CandidateLine {
            lrp: second_lrp,
            edge: EdgeId(-5530113),
            rating: RatingScore::from(2000.0),
            distance_to_projection: None,
        };

        let line2_second_lrp = CandidateLine {
            lrp: second_lrp,
            edge: EdgeId(-7292029),
            rating: RatingScore::from(100.0),
            distance_to_projection: None,
        };

        let line_last_lrp = CandidateLine {
            lrp: last_lrp,
            edge: EdgeId(-7292028),
            rating: RatingScore::from(1000.0),
            distance_to_projection: None,
        };

        let candidate_lines = [
            CandidateLines {
                lrp: first_lrp,
                lines: vec![line_first_lrp],
            },
            CandidateLines {
                lrp: second_lrp,
                lines: vec![line1_second_lrp, line2_second_lrp],
            },
            CandidateLines {
                lrp: last_lrp,
                lines: vec![line_last_lrp],
            },
        ];

        let routes = resolve_routes(&config, graph, &candidate_lines, Offsets::default()).unwrap();
        assert_eq!(routes.len(), 2);

        assert_eq!(
            routes[0],
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(-7292030)],
                    length: Length::from_meters(108.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_first_lrp,
                    line_lrp2: line2_second_lrp
                }
            }
        );

        assert_eq!(
            routes[1],
            CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(-7292029), EdgeId(-7292028)],
                    length: Length::from_meters(90.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line2_second_lrp,
                    line_lrp2: line_last_lrp
                }
            },
        );
    }

    #[test]
    fn decoder_resolve_routes_006() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig::default();

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.454214,
                lat: 52.5157088,
            },
            line: LineAttributes {
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(99),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc2,
                dnp: Length::from_meters(100.0),
            }),
        };

        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.455676,
                lat: 52.515561,
            },
            line: LineAttributes {
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(100),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc2,
                dnp: Length::from_meters(100.0),
            }),
        };

        let third_lrp = Point {
            coordinate: Coordinate {
                lon: 13.457137,
                lat: 52.515407,
            },
            line: LineAttributes {
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(100),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc2,
                dnp: Length::from_meters(17.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.457386,
                lat: 52.5153814,
            },
            line: LineAttributes {
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(280),
            },
            path: None,
        };

        let lrp_on_same_line = |lrp| CandidateLine {
            lrp,
            edge: EdgeId(16218),
            rating: RatingScore::from(1000.0),
            distance_to_projection: None,
        };

        let line_first_lrp = lrp_on_same_line(first_lrp);
        let line_second_lrp = lrp_on_same_line(second_lrp);
        let line_third_lrp = lrp_on_same_line(third_lrp);
        let line_last_lrp = lrp_on_same_line(last_lrp);

        let candidate_lines = [
            CandidateLines {
                lrp: first_lrp,
                lines: vec![line_first_lrp],
            },
            CandidateLines {
                lrp: second_lrp,
                lines: vec![line_second_lrp],
            },
            CandidateLines {
                lrp: third_lrp,
                lines: vec![line_third_lrp],
            },
            CandidateLines {
                lrp: last_lrp,
                lines: vec![line_last_lrp],
            },
        ];

        let routes = resolve_routes(&config, graph, &candidate_lines, Offsets::default()).unwrap();
        assert_eq!(routes.len(), 3);

        assert_eq!(
            routes[0],
            CandidateRoute {
                path: Path {
                    length: Length::ZERO,
                    edges: vec![]
                },
                candidates: CandidateLinePair {
                    line_lrp1: CandidateLine {
                        lrp: Point {
                            coordinate: Coordinate {
                                lon: 13.454214,
                                lat: 52.5157088
                            },
                            line: LineAttributes {
                                frc: Frc::Frc2,
                                fow: Fow::SingleCarriageway,
                                bearing: Bearing::from_degrees(99)
                            },
                            path: Some(PathAttributes {
                                lfrcnp: Frc::Frc2,
                                dnp: Length::from_meters(100.0)
                            })
                        },
                        edge: EdgeId(16218),
                        rating: RatingScore::from(1000.0),
                        distance_to_projection: None
                    },
                    line_lrp2: CandidateLine {
                        lrp: Point {
                            coordinate: Coordinate {
                                lon: 13.455676,
                                lat: 52.515561
                            },
                            line: LineAttributes {
                                frc: Frc::Frc2,
                                fow: Fow::SingleCarriageway,
                                bearing: Bearing::from_degrees(100)
                            },
                            path: Some(PathAttributes {
                                lfrcnp: Frc::Frc2,
                                dnp: Length::from_meters(100.0)
                            })
                        },
                        edge: EdgeId(16218),
                        rating: RatingScore::from(1000.0),
                        distance_to_projection: None
                    }
                }
            }
        );

        assert_eq!(
            routes[1],
            CandidateRoute {
                path: Path {
                    length: Length::ZERO,
                    edges: vec![]
                },
                candidates: CandidateLinePair {
                    line_lrp1: CandidateLine {
                        lrp: Point {
                            coordinate: Coordinate {
                                lon: 13.455676,
                                lat: 52.515561
                            },
                            line: LineAttributes {
                                frc: Frc::Frc2,
                                fow: Fow::SingleCarriageway,
                                bearing: Bearing::from_degrees(100)
                            },
                            path: Some(PathAttributes {
                                lfrcnp: Frc::Frc2,
                                dnp: Length::from_meters(100.0)
                            })
                        },
                        edge: EdgeId(16218),
                        rating: RatingScore::from(1000.0),
                        distance_to_projection: None
                    },
                    line_lrp2: CandidateLine {
                        lrp: Point {
                            coordinate: Coordinate {
                                lon: 13.457137,
                                lat: 52.515407
                            },
                            line: LineAttributes {
                                frc: Frc::Frc2,
                                fow: Fow::SingleCarriageway,
                                bearing: Bearing::from_degrees(100)
                            },
                            path: Some(PathAttributes {
                                lfrcnp: Frc::Frc2,
                                dnp: Length::from_meters(17.0)
                            })
                        },
                        edge: EdgeId(16218),
                        rating: RatingScore::from(1000.0),
                        distance_to_projection: None
                    }
                }
            }
        );

        assert_eq!(
            routes[2],
            CandidateRoute {
                path: Path {
                    length: Length::from_meters(217.0),
                    edges: vec![EdgeId(16218)]
                },
                candidates: CandidateLinePair {
                    line_lrp1: CandidateLine {
                        lrp: Point {
                            coordinate: Coordinate {
                                lon: 13.457137,
                                lat: 52.515407
                            },
                            line: LineAttributes {
                                frc: Frc::Frc2,
                                fow: Fow::SingleCarriageway,
                                bearing: Bearing::from_degrees(100)
                            },
                            path: Some(PathAttributes {
                                lfrcnp: Frc::Frc2,
                                dnp: Length::from_meters(17.0)
                            })
                        },
                        edge: EdgeId(16218),
                        rating: RatingScore::from(1000.0),
                        distance_to_projection: None
                    },
                    line_lrp2: CandidateLine {
                        lrp: Point {
                            coordinate: Coordinate {
                                lon: 13.457386,
                                lat: 52.5153814
                            },
                            line: LineAttributes {
                                frc: Frc::Frc2,
                                fow: Fow::SingleCarriageway,
                                bearing: Bearing::from_degrees(280)
                            },
                            path: None
                        },
                        edge: EdgeId(16218),
                        rating: RatingScore::from(1000.0),
                        distance_to_projection: None
                    }
                }
            }
        );
    }

    #[test]
    fn decoder_resolve_routes_007() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;
        let config = DecoderConfig::default();

        let length = |e: i64| graph.get_edge_length(EdgeId(e)).unwrap();
        let real_distance = length(8717174) + length(8717175);
        let destination_length = length(109783);
        assert!(config.next_point_variance < destination_length * 2.0);

        let candidates = |dnp: Length| CandidateLinePair {
            line_lrp1: CandidateLine {
                lrp: Point {
                    coordinate: Coordinate {
                        lon: 13.4615506,
                        lat: 52.5170544,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc6,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(107),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc6,
                        dnp,
                    }),
                },
                edge: EdgeId(8717174),
                rating: RatingScore::from(1000.0),
                distance_to_projection: None,
            },
            line_lrp2: CandidateLine {
                lrp: Point {
                    coordinate: Coordinate {
                        lon: 13.4625506,
                        lat: 52.5168944,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc6,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(17),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc6,
                        dnp: Length::from_meters(100.0),
                    }),
                },
                edge: EdgeId(109783),
                rating: RatingScore::from(1000.0),
                distance_to_projection: None,
            },
        };

        let route = resolve_candidate_route(&config, graph, candidates(real_distance)).unwrap();
        assert!(route.is_some());

        let dnp = real_distance + config.next_point_variance + destination_length * 0.5;
        let route = resolve_candidate_route(&config, graph, candidates(dnp)).unwrap();
        assert!(route.is_none());
    }

    #[test]
    fn decoder_resolve_routes_008() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;
        let config = DecoderConfig::default();

        let length = |e: i64| graph.get_edge_length(EdgeId(e)).unwrap();
        let projection = length(109783) * 0.25;
        let real_distance = length(8717174) + length(8717175) + projection;
        let margin = Length::from_meters(10.0);
        assert!(projection + margin < length(109783));

        let candidates = |dnp: Length| CandidateLinePair {
            line_lrp1: CandidateLine {
                lrp: Point {
                    coordinate: Coordinate {
                        lon: 13.4615506,
                        lat: 52.5170544,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc6,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(107),
                    },
                    path: Some(PathAttributes {
                        lfrcnp: Frc::Frc6,
                        dnp,
                    }),
                },
                edge: EdgeId(8717174),
                rating: RatingScore::from(1000.0),
                distance_to_projection: None,
            },
            line_lrp2: CandidateLine {
                lrp: Point {
                    coordinate: Coordinate {
                        lon: 13.46284,
                        lat: 52.51500,
                    },
                    line: LineAttributes {
                        frc: Frc::Frc6,
                        fow: Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(17),
                    },
                    path: None,
                },
                edge: EdgeId(109783),
                rating: RatingScore::from(1000.0),
                distance_to_projection: Some(projection),
            },
        };

        let route = resolve_candidate_route(&config, graph, candidates(real_distance)).unwrap();
        assert!(route.is_some());

        let dnp = real_distance + config.next_point_variance + margin;
        let route = resolve_candidate_route(&config, graph, candidates(dnp)).unwrap();
        assert!(route.is_none());
    }

    #[test]
    fn decoder_resolve_routes_009() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;
        let config = DecoderConfig::default();

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4572516,
                lat: 52.5149212,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(95),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(248.0),
            }),
        };

        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4572516,
                lat: 52.5149212,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(0),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(160.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.458181,
                lat: 52.515299,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(280),
            },
            path: None,
        };

        let candidate_lines = [
            CandidateLines {
                lrp: first_lrp,
                lines: vec![CandidateLine {
                    lrp: first_lrp,
                    edge: EdgeId(3622026),
                    rating: RatingScore::from(1000.0),
                    distance_to_projection: None,
                }],
            },
            CandidateLines {
                lrp: second_lrp,
                lines: vec![CandidateLine {
                    lrp: second_lrp,
                    edge: EdgeId(3622025),
                    rating: RatingScore::from(1000.0),
                    distance_to_projection: None,
                }],
            },
            CandidateLines {
                lrp: last_lrp,
                lines: vec![CandidateLine {
                    lrp: last_lrp,
                    edge: EdgeId(16219),
                    rating: RatingScore::from(1000.0),
                    distance_to_projection: None,
                }],
            },
        ];

        // routes (sub-path between two consecutive LRPs) that are closed loops must be rejected
        for offsets in [Offsets::ZERO, Offsets::negative(Offset::from_range(0.5))] {
            let routes = resolve_routes(&config, graph, &candidate_lines, offsets);
            assert!(matches!(routes, Err(DecodeError::RouteNotFound(_))));
        }
    }

    #[test]
    fn decoder_resolve_routes_010() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;
        let config = DecoderConfig::default();

        let first_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4551048,
                lat: 52.5152531,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(100),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(153.0),
            }),
        };

        let second_lrp = Point {
            coordinate: Coordinate {
                lon: 13.4572516,
                lat: 52.5149212,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(95),
            },
            path: Some(PathAttributes {
                lfrcnp: Frc::Frc6,
                dnp: Length::from_meters(299.0),
            }),
        };

        let last_lrp = Point {
            coordinate: Coordinate {
                lon: 13.457386,
                lat: 52.5153814,
            },
            line: LineAttributes {
                frc: Frc::Frc6,
                fow: Fow::SingleCarriageway,
                bearing: Bearing::from_degrees(0),
            },
            path: None,
        };

        let candidate_lines = [
            CandidateLines {
                lrp: first_lrp,
                lines: vec![CandidateLine {
                    lrp: first_lrp,
                    edge: EdgeId(7020005),
                    rating: RatingScore::from(1000.0),
                    distance_to_projection: None,
                }],
            },
            CandidateLines {
                lrp: second_lrp,
                lines: vec![CandidateLine {
                    lrp: second_lrp,
                    edge: EdgeId(3622026),
                    rating: RatingScore::from(1000.0),
                    distance_to_projection: None,
                }],
            },
            CandidateLines {
                lrp: last_lrp,
                lines: vec![CandidateLine {
                    lrp: last_lrp,
                    edge: EdgeId(3622025),
                    rating: RatingScore::from(1000.0),
                    distance_to_projection: None,
                }],
            },
        ];

        // routes (sub-path between two consecutive LRPs) that are closed loops must be rejected
        for offsets in [Offsets::ZERO, Offsets::positive(Offset::from_range(0.5))] {
            let routes = resolve_routes(&config, graph, &candidate_lines, offsets);
            assert!(matches!(routes, Err(DecodeError::RouteNotFound(_))));
        }
    }
}
