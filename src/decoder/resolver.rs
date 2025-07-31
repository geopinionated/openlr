use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::fmt::Debug;

use tracing::debug;

use crate::decoder::candidates::{CandidateLine, CandidateLinePair, CandidateLines};
use crate::decoder::route::{Route, Routes};
use crate::decoder::shortest_path::shortest_path;
use crate::{
    DecodeError, DecoderConfig, DirectedGraph, Frc, Length, Path, RatingScore, is_path_connected,
};

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
/// - All lengths of the lines should be measured in meters and should also be converted to
///   integer values, float values need to be rounded correctly.
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
pub fn resolve_routes<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
) -> Result<Routes<G::EdgeId>, DecodeError> {
    debug!("Resolving routes for {} candidates", candidate_lines.len());

    if let Some(routes) = resolve_single_line_routes(graph, candidate_lines) {
        debug_assert!(is_path_connected(graph, &routes.to_path()));
        return Ok(routes);
    }

    let mut routes: Vec<Route<G::EdgeId>> = Vec::with_capacity(candidate_lines.len() - 1);

    for window in candidate_lines.windows(2) {
        let [candidates_lrp1, candidates_lrp2] = [&window[0], &window[1]];
        let pairs = resolve_top_k_candidate_pairs(config, candidates_lrp1, candidates_lrp2)?;

        let CandidateLines { lrp: lrp1, .. } = candidates_lrp1;
        let CandidateLines { lrp: lrp2, .. } = candidates_lrp2;

        let lowest_frc_value = lrp1.lfrcnp().value() + Frc::variance(&lrp1.lfrcnp());
        let lowest_frc = Frc::from_value(lowest_frc_value).unwrap_or(Frc::Frc7);

        if let Some(route) = resolve_candidate_pairs_path(config, graph, pairs, lowest_frc) {
            // TODO: if the previous route ends on a line that is not the start of this new route
            // the previous route needs to be re-computed
            if let Some(last_edge) = routes.last().map(|r| r.last_candidate_edge())
                && last_edge != route.first_candidate_edge()
            {
                return Err(DecodeError::AlternativeRouteNotFound((*lrp1, *lrp2)));
            }

            routes.push(route);
        } else {
            return Err(DecodeError::RouteNotFound((*lrp1, *lrp2)));
        }
    }

    let routes = Routes::from(routes);
    debug_assert!(is_path_connected(graph, &routes.to_path()));
    Ok(routes)
}

/// If all the best candidate lines are equal there is no need to compute top K candidates and
/// their shortest paths, we can just return the best candidate line for each LRP.
fn resolve_single_line_routes<G: DirectedGraph>(
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
) -> Option<Routes<G::EdgeId>> {
    if let Some(best_candidate) = candidate_lines.first().and_then(|c| c.best_candidate())
        && candidate_lines
            .iter()
            .all(|c| c.best_candidate().map(|line| line.edge) == Some(best_candidate.edge))
    {
        let pairs = candidate_lines.windows(2).filter_map(|window| {
            Some(CandidateLinePair {
                line_lrp1: window[0].best_candidate()?,
                line_lrp2: window[1].best_candidate()?,
            })
        });

        let routes: Vec<_> = pairs
            .enumerate()
            .map(|(i, candidates)| {
                let edges = if i == 0 {
                    vec![best_candidate.edge]
                } else {
                    vec![]
                };

                let path = Path {
                    length: edges.iter().filter_map(|&e| graph.get_edge_length(e)).sum(),
                    edges,
                };

                Route { path, candidates }
            })
            .collect();

        Some(Routes::from(routes))
    } else {
        None
    }
}

fn resolve_candidate_pairs_path<G, I>(
    config: &DecoderConfig,
    graph: &G,
    pairs: I,
    lowest_frc: Frc,
) -> Option<Route<G::EdgeId>>
where
    G: DirectedGraph,
    I: IntoIterator<Item = CandidateLinePair<G::EdgeId>>,
{
    debug!("Resolving candidate pairs with lowest {lowest_frc:?}");

    for candidates in pairs {
        let CandidateLinePair {
            line_lrp1,
            line_lrp2,
        } = candidates;

        if line_lrp1.edge == line_lrp2.edge {
            let edges = if line_lrp2.lrp.is_last() {
                vec![line_lrp1.edge]
            } else {
                vec![]
            };

            let path = Path {
                length: edges.iter().filter_map(|&e| graph.get_edge_length(e)).sum(),
                edges,
            };

            return Some(Route { path, candidates });
        }

        let origin = graph.get_edge_start_vertex(line_lrp1.edge)?;
        let destination = if line_lrp2.lrp.is_last() {
            graph.get_edge_end_vertex(line_lrp2.edge)?
        } else {
            graph.get_edge_start_vertex(line_lrp2.edge)?
        };

        let max_length = max_route_length(config, graph, &line_lrp1, &line_lrp2);

        if let Some(path) = shortest_path(graph, origin, destination, lowest_frc, max_length) {
            debug_assert!(path.length <= max_length);
            let min_length = line_lrp1.lrp.dnp() - config.next_point_variance;

            if path.length >= min_length {
                return Some(Route { path, candidates });
            }
        }
    }

    None
}

fn max_route_length<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line_lrp1: &CandidateLine<G::EdgeId>,
    line_lrp2: &CandidateLine<G::EdgeId>,
) -> Length {
    let mut max_distance = line_lrp1.lrp.dnp() + config.next_point_variance;

    // shortest path can only stop at distances between real vertices, therefore we need to
    // add the complete length when computing max distance upper bound if the lines were projected
    if line_lrp1.is_projected() {
        max_distance += graph
            .get_edge_length(line_lrp1.edge)
            .unwrap_or(Length::ZERO);
    }
    if line_lrp2.is_projected() {
        max_distance += graph
            .get_edge_length(line_lrp2.edge)
            .unwrap_or(Length::ZERO);
    }

    max_distance
}

fn resolve_top_k_candidate_pairs<EdgeId: Debug + Copy>(
    config: &DecoderConfig,
    lines_lrp1: &CandidateLines<EdgeId>,
    lines_lrp2: &CandidateLines<EdgeId>,
) -> Result<Vec<CandidateLinePair<EdgeId>>, DecodeError> {
    let max_size = lines_lrp1.lines.len() * lines_lrp2.lines.len();
    let k_size = max_size.min(config.max_number_retries + 1);
    debug!("Resolving candidate pair ratings with size={k_size}");

    let mut pair_ratings: BinaryHeap<Reverse<RatingScore>> = BinaryHeap::with_capacity(k_size + 1);
    let mut rating_pairs: HashMap<RatingScore, Vec<_>> = HashMap::with_capacity(k_size + 1);

    for &line_lrp1 in &lines_lrp1.lines {
        for &line_lrp2 in &lines_lrp2.lines {
            let pair_rating = line_lrp1.rating * line_lrp2.rating;
            pair_ratings.push(Reverse(pair_rating));

            if pair_ratings.len() <= k_size {
                rating_pairs
                    .entry(pair_rating)
                    .or_default()
                    .push(CandidateLinePair {
                        line_lrp1,
                        line_lrp2,
                    });

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
                .push(CandidateLinePair {
                    line_lrp1,
                    line_lrp2,
                });

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

    debug_assert_eq!(candidates.len(), k_size);
    debug_assert!(candidates.is_sorted_by_key(|pair| Reverse(pair.rating())));

    Ok(candidates)
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
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

        let routes = resolve_routes(&config, graph, &candidate_lines).unwrap();

        assert_eq!(
            routes,
            Routes::from(vec![Route {
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

        let routes = resolve_routes(&config, graph, &candidate_lines).unwrap();

        assert_eq!(
            routes,
            Routes::from(vec![Route {
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

        let routes = resolve_routes(&config, graph, &candidate_lines).unwrap();

        assert_eq!(
            routes,
            Routes::from(vec![
                Route {
                    path: Path {
                        edges: vec![], // first and second LRPs are on the same line
                        length: Length::ZERO,
                    },
                    candidates: CandidateLinePair {
                        line_lrp1: line_first_lrp,
                        line_lrp2: line_second_lrp
                    }
                },
                Route {
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

        let routes = resolve_routes(&config, graph, &candidate_lines).unwrap();

        assert_eq!(
            routes,
            Routes::from(vec![
                Route {
                    path: Path {
                        edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                        length: Length::from_meters(379.0),
                    },
                    candidates: CandidateLinePair {
                        line_lrp1: line1_first_lrp,
                        line_lrp2: line1_second_lrp
                    }
                },
                Route {
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
}
