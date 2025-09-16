use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::fmt::Debug;

use rustc_hash::FxHashMap;
use tracing::debug;

use crate::decoder::candidates::{CandidateLine, CandidateLinePair, CandidateLines};
use crate::decoder::route::{CandidateRoute, CandidateRoutes};
use crate::decoder::shortest_path::shortest_path;
use crate::graph::path::{Path, is_path_connected, is_path_loop};
use crate::model::RatingScore;
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
) -> Result<CandidateRoutes<G::EdgeId>, DecodeError> {
    debug!("Resolving routes for {} LRPs", candidate_lines.len());
    let best_edge = find_best_candidate_edge(candidate_lines);

    if let Some(routes) = best_edge.and_then(|best_edge| {
        resolve_single_line_routes(graph, candidate_lines, best_edge, offsets)
    }) {
        debug_assert!(is_path_connected(graph, &routes.to_path()));
        return Ok(routes);
    }

    let mut routes: CandidateRoutes<_> = Vec::with_capacity(candidate_lines.len() - 1).into();

    for window in candidate_lines.windows(2) {
        let [candidates_lrp1, candidates_lrp2] = [&window[0], &window[1]];
        let routes_count = routes.len();

        let pairs =
            resolve_top_k_candidate_pairs(config, candidates_lrp1, candidates_lrp2, best_edge)?;

        // Find the first candidates pair that can be used to construct a valid route between the
        // two consecutive LRPs, also try to find an alternative route if consecutive best pairs are
        // not connected to each other.
        for candidates in pairs {
            let route = resolve_candidate_route(config, graph, candidates)
                .and_then(|route| resolve_alternative_route(config, graph, &mut routes, route))
                .filter(|route| {
                    let (pos_offset, neg_offset) = route.calculate_offsets(graph, offsets);
                    !is_path_loop(graph, &route.path.edges, pos_offset, neg_offset)
                });

            if let Some(route) = route {
                routes.push(route);
                break;
            }
        }

        if routes.len() == routes_count {
            return Err(DecodeError::RouteNotFound((
                candidates_lrp1.lrp,
                candidates_lrp2.lrp,
            )));
        }
    }

    debug_assert!(is_path_connected(graph, &routes.to_path()));
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
) -> Option<CandidateRoutes<G::EdgeId>> {
    let pairs = candidate_lines.windows(2).filter_map(|window| {
        Some(CandidateLinePair {
            line_lrp1: window[0].best_candidate()?,
            line_lrp2: window[1].best_candidate()?,
        })
    });

    let routes: Vec<_> = pairs
        .enumerate()
        .map(|(i, candidates)| {
            let edges = if i == 0 { vec![best_edge] } else { vec![] };

            let path = Path {
                length: edges.iter().filter_map(|&e| graph.get_edge_length(e)).sum(),
                edges,
            };

            CandidateRoute { path, candidates }
        })
        .collect();

    let routes: CandidateRoutes<_> = routes.into();
    let offsets = routes.calculate_offsets(graph, offsets);
    let (pos_offset, neg_offset) = offsets.unwrap_or_default();

    if pos_offset + neg_offset >= routes.path_length() {
        debug!("Same line route on {best_edge:?} has invalid offsets");
        return None;
    }

    debug!("Route resolved on single best edge: {best_edge:?}");
    Some(routes)
}

fn resolve_candidate_route<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidates: CandidateLinePair<G::EdgeId>,
) -> Option<CandidateRoute<G::EdgeId>> {
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
                ..
            },
    } = candidates;

    if edge_lrp1 == edge_lrp2 {
        let edges = if lrp2.is_last() {
            vec![edge_lrp1]
        } else {
            vec![]
        };

        let path = Path {
            length: edges.iter().filter_map(|&e| graph.get_edge_length(e)).sum(),
            edges,
        };

        return Some(CandidateRoute { path, candidates });
    }

    let lowest_frc_value = lrp1.lfrcnp().value() + Frc::variance(&lrp1.lfrcnp());
    let lowest_frc = Frc::from_value(lowest_frc_value).unwrap_or(Frc::Frc7);
    let max_length = max_route_length(config, graph, &candidates);

    debug!("Finding route: {edge_lrp1:?} -> {edge_lrp2:?} (max={max_length} low={lowest_frc:?})");

    if let Some(mut path) = shortest_path(graph, edge_lrp1, edge_lrp2, lowest_frc, max_length) {
        let min_length = lrp1.dnp() - config.next_point_variance;

        if path.length < min_length {
            debug!("{path:?} length is shorter than expected: {min_length}");
            return None;
        }

        if !lrp2.is_last() {
            let last_edge = path.edges.pop()?;
            path.length -= graph.get_edge_length(last_edge)?;
        }

        debug_assert!(!path.edges.is_empty());
        debug_assert!(path.length <= max_length);

        debug!("Route found: {edge_lrp1:?} -> {edge_lrp2:?}: {path:?}");
        return Some(CandidateRoute { path, candidates });
    }

    debug!("Route not found: {edge_lrp1:?} -> {edge_lrp2:?}");
    None
}

/// Updates the last route with an alternative if this cannot be connected to the given new route.
/// Returns the new given route or None if the altenative is needed but cannot be computed.
fn resolve_alternative_route<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    routes: &mut [CandidateRoute<G::EdgeId>],
    new_route: CandidateRoute<G::EdgeId>,
) -> Option<CandidateRoute<G::EdgeId>> {
    if let Some(last_route) = routes.last_mut() {
        // if the previous route ends on a line that is not the start of this new route
        // then the previous route needs to be re-computed
        if last_route.last_candidate_edge() != new_route.first_candidate_edge() {
            let candidates = CandidateLinePair {
                line_lrp1: last_route.first_candidate(),
                line_lrp2: new_route.first_candidate(),
            };

            *last_route = resolve_candidate_route(config, graph, candidates)?;
        }
    }

    Some(new_route)
}

fn max_route_length<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidates: &CandidateLinePair<G::EdgeId>,
) -> Length {
    let CandidateLinePair {
        line_lrp1,
        line_lrp2,
    } = candidates;

    let mut max_distance = line_lrp1.lrp.dnp() + config.next_point_variance;

    // shortest path can only stop at distances between real vertices, therefore we need to
    // add the complete length when computing max distance upper bound if the lines were projected
    if line_lrp1.is_projected() {
        max_distance += graph
            .get_edge_length(line_lrp1.edge)
            .unwrap_or(Length::ZERO);
    }

    if line_lrp2.is_projected() || !line_lrp2.lrp.is_last() {
        max_distance += graph
            .get_edge_length(line_lrp2.edge)
            .unwrap_or(Length::ZERO);
    }

    Length::from_meters(max_distance.meters().ceil())
}

fn resolve_top_k_candidate_pairs<EdgeId: Debug + Copy + PartialEq>(
    config: &DecoderConfig,
    lines_lrp1: &CandidateLines<EdgeId>,
    lines_lrp2: &CandidateLines<EdgeId>,
    best_single_line_edge: Option<EdgeId>,
) -> Result<Vec<CandidateLinePair<EdgeId>>, DecodeError> {
    let max_size = lines_lrp1.lines.len() * lines_lrp2.lines.len();
    let k_size = max_size.min(config.max_number_retries + 1);
    debug!("Resolving candidate pair ratings with K size: {k_size}");

    let mut pair_ratings: BinaryHeap<Reverse<RatingScore>> = BinaryHeap::with_capacity(k_size + 1);
    let mut rating_pairs: FxHashMap<RatingScore, Vec<_>> =
        FxHashMap::with_capacity_and_hasher(k_size + 1, Default::default());

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

            let candidate_pair = CandidateLinePair {
                line_lrp1,
                line_lrp2,
            };

            let pair_rating = candidate_pair.rating(config.same_line_degradation);
            pair_ratings.push(Reverse(pair_rating));

            if pair_ratings.len() <= k_size {
                rating_pairs
                    .entry(pair_rating)
                    .or_default()
                    .push(candidate_pair);

                continue;
            }

            let worst_rating = match pair_ratings.pop() {
                Some(Reverse(rating)) if pair_rating <= rating => continue,
                Some(Reverse(rating)) => rating,
                None => continue,
            };

            rating_pairs
                .entry(pair_rating)
                .or_default()
                .push(candidate_pair);

            if let Some(pairs) = rating_pairs.get_mut(&worst_rating)
                && pairs.len() > 1
            {
                pairs.pop();
            } else {
                rating_pairs.remove(&worst_rating);
            }
        }
    }

    let mut candidates = Vec::with_capacity(k_size);
    while let Some(Reverse(rating)) = pair_ratings.pop() {
        candidates.extend(rating_pairs.remove(&rating).into_iter().flatten());
    }
    candidates.reverse();

    debug!(
        "Top K candidates: {:?}",
        candidates
            .iter()
            .map(|pair| (
                pair.line_lrp1.edge,
                pair.line_lrp2.edge,
                pair.rating(config.same_line_degradation)
            ))
            .collect::<Vec<_>>()
    );

    debug_assert!(rating_pairs.is_empty());
    debug_assert!(candidates.len() <= k_size);
    debug_assert!(
        candidates.is_sorted_by_key(|pair| Reverse(pair.rating(config.same_line_degradation)))
    );

    Ok(candidates)
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::decoder::candidates::CandidateLine;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
    use crate::{Bearing, Coordinate, Fow, LineAttributes, PathAttributes, Point};

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
        )
        .unwrap();

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

        assert_eq!(
            routes,
            CandidateRoutes::from(vec![CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                    length: Length::from_meters(379.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line1_first_lrp,
                    line_lrp2: line_last_lrp
                }
            }])
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

        assert_eq!(
            routes,
            CandidateRoutes::from(vec![CandidateRoute {
                path: Path {
                    edges: vec![EdgeId(8717174)],
                    length: Length::from_meters(136.0),
                },
                candidates: CandidateLinePair {
                    line_lrp1: line_first_lrp,
                    line_lrp2: line1_last_lrp
                }
            }])
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

        assert_eq!(
            routes,
            CandidateRoutes::from(vec![
                CandidateRoute {
                    path: Path {
                        edges: vec![], // first and second LRPs are on the same line
                        length: Length::ZERO,
                    },
                    candidates: CandidateLinePair {
                        line_lrp1: line_first_lrp,
                        line_lrp2: line_second_lrp
                    }
                },
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
            ])
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

        assert_eq!(
            routes,
            CandidateRoutes::from(vec![
                CandidateRoute {
                    path: Path {
                        edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                        length: Length::from_meters(379.0),
                    },
                    candidates: CandidateLinePair {
                        line_lrp1: line1_first_lrp,
                        line_lrp2: line1_second_lrp
                    }
                },
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
            ])
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

        assert_eq!(
            routes,
            CandidateRoutes::from(vec![
                CandidateRoute {
                    path: Path {
                        edges: vec![EdgeId(-7292030)],
                        length: Length::from_meters(108.0),
                    },
                    candidates: CandidateLinePair {
                        line_lrp1: line_first_lrp,
                        line_lrp2: line2_second_lrp
                    }
                },
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
            ])
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

        assert_eq!(
            routes,
            CandidateRoutes::from(vec![
                CandidateRoute {
                    path: Path {
                        length: Length::from_meters(217.0),
                        edges: vec![EdgeId(16218)]
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
                },
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
                },
                CandidateRoute {
                    path: Path {
                        length: Length::ZERO,
                        edges: vec![]
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
            ])
        );
    }
}
