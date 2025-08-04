use std::cmp::Reverse;
use std::fmt::Debug;

use tracing::{debug, trace};

use crate::model::RatingScore;
use crate::{Bearing, DecodeError, DecoderConfig, DirectedGraph, Fow, Frc, Length, Point};

/// List of candidate nodes for a Location Reference Point (LRP).
/// Nodes are sorted based on their distance to the point (closest to farthest).
#[derive(Debug, Clone, PartialEq)]
pub struct CandidateNodes<VertexId> {
    pub lrp: Point,
    pub nodes: Vec<CandidateNode<VertexId>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CandidateNode<VertexId> {
    pub vertex: VertexId,
    pub distance_to_lrp: Length,
}

/// List of candidate lines for a Location Reference Point (LRP).
/// Lines are sorted based on their rating (descending - higher rating is better).
#[derive(Debug, Clone, PartialEq)]
pub struct CandidateLines<EdgeId> {
    pub lrp: Point,
    pub lines: Vec<CandidateLine<EdgeId>>,
}

impl<EdgeId: Copy> CandidateLines<EdgeId> {
    /// Returns the candidate line with the highest rating.
    pub fn best_candidate(&self) -> Option<CandidateLine<EdgeId>> {
        self.lines.first().copied()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CandidateLine<EdgeId> {
    pub lrp: Point,
    pub edge: EdgeId,
    pub rating: RatingScore,
    /// If this line is the result of a projection of the LRP into it, this represents the distance
    /// from the beginning of the line (start vertex) to the point where the LRP was projected.
    pub distance_to_projection: Option<Length>,
}

impl<EdgeId> CandidateLine<EdgeId> {
    pub const fn is_projected(&self) -> bool {
        self.distance_to_projection.is_some()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct CandidateLinePair<EdgeId> {
    pub line_lrp1: CandidateLine<EdgeId>,
    pub line_lrp2: CandidateLine<EdgeId>,
}

impl<EdgeId> CandidateLinePair<EdgeId> {
    pub fn rating(&self) -> RatingScore {
        self.line_lrp1.rating * self.line_lrp2.rating
    }
}

/// Candidate line that is yet to be rated.
#[derive(Debug, Clone)]
struct ProvisionalCandidateLine<EdgeId> {
    /// Location Reference Point associated to the line.
    lrp: Point,
    /// Edge of the candidate line.
    /// When not projected, this edge exits the LRP (or enters the last LRP).
    edge: EdgeId,
    /// Linear distance from the LRP to the candidate line (i.e. to the candidate node vertex,
    /// or to candidate line edge if the LRP was projected).
    distance_to_lrp: Length,
    /// Distance from the start of the edge to the projected LRP into the edge.
    /// If the LRP is not projected it will be None.
    distance_to_projection: Option<Length>,
    /// Functional Road Class of the line.
    frc: Frc,
    /// Form of Way of the line.
    fow: Fow,
    /// Bearing of the part of the line (of a fixed length) that will be considered starting from
    /// the distance to the LRP projection.
    bearing: Bearing,
}

/// Each location reference point contains coordinates specifying a node in the encoder map. The
/// decoder should try to find so called candidate nodes in the decoder map whereby the coordinates
/// of the candidate nodes are close to the coordinates of the location reference point coordinates.
/// The straight line distance should be used to identify close-by nodes. Nodes in the decoder map
/// which are far away from the coordinates of the location reference point should not be considered
/// as candidate nodes in the further processing. It might happen that several candidate nodes for
/// one location reference point exist.
///
/// If no candidate node has been determined for a location reference point the decoder should try
/// to determine a candidate line directly. The LRP coordinate can be projected onto lines which are
/// not far away from that coordinate.
pub fn find_candidate_nodes<'a, G, I>(
    config: &DecoderConfig,
    graph: &G,
    points: I,
) -> impl ExactSizeIterator<Item = CandidateNodes<G::VertexId>>
where
    G: DirectedGraph,
    I: IntoIterator,
    I::IntoIter: ExactSizeIterator<Item = &'a Point>,
{
    points.into_iter().map(move |&lrp| {
        debug!("Finding candidate nodes for {lrp:?}");

        let nodes: Vec<_> = graph
            .nearest_vertices_within_distance(lrp.coordinate, config.max_node_distance)
            .map(|(vertex, distance_to_lrp)| {
                debug_assert!(distance_to_lrp <= config.max_node_distance);
                CandidateNode {
                    vertex,
                    distance_to_lrp,
                }
            })
            .collect();

        debug_assert!(nodes.is_sorted_by_key(|n| n.distance_to_lrp));
        CandidateNodes { lrp, nodes }
    })
}

/// For each location reference point the decoder tries to determine lines which should fulfill the
/// following constraints:
/// - The start node, end node for the last location reference point or projection point shall be
///   close to the coordinates of the location reference point.
/// - The candidate lines should be outgoing lines (incoming lines for the last location reference
///   point) of the candidate nodes or projection points determined in the previous step.
/// - The candidate lines should match the attributes functional road class, form of way and bearing
///   as extracted from the physical data. Slight variances are allowed and shall be taken into
///   account in step 4.
///
/// The direct search of lines using a projection point may also be executed even if candidate nodes
/// are found. This might increase the number of candidate nodes but it could help to determine the
/// correct candidate line in the next step if the nodes in the encoder and decoder map differ
/// significantly.
///
/// If no candidate line can be found for a location reference point, the decoder should report an
/// error and stop further processing.
pub fn find_candidate_lines<G: DirectedGraph, I>(
    config: &DecoderConfig,
    graph: &G,
    candidate_nodes: I,
) -> Result<Vec<CandidateLines<G::EdgeId>>, DecodeError>
where
    I: IntoIterator,
    I::IntoIter: ExactSizeIterator<Item = CandidateNodes<G::VertexId>>,
{
    let candidate_nodes = candidate_nodes.into_iter();
    let mut candidate_lines = Vec::with_capacity(candidate_nodes.len());

    for lrp_nodes in candidate_nodes {
        let mut lrp_lines = find_candidate_lines_from_nodes(config, graph, lrp_nodes);
        append_projected_candidate_lines(config, graph, &mut lrp_lines);

        if lrp_lines.lines.is_empty() {
            return Err(DecodeError::CandidatesNotFound(lrp_lines.lrp));
        }

        lrp_lines
            .lines
            .sort_unstable_by_key(|line| Reverse(line.rating));
        candidate_lines.push(lrp_lines);
    }

    Ok(candidate_lines)
}

fn find_candidate_lines_from_nodes<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_nodes: CandidateNodes<G::VertexId>,
) -> CandidateLines<G::EdgeId> {
    let CandidateNodes { lrp, nodes } = candidate_nodes;
    debug!("Finding lines from {} nodes of {lrp:?}", nodes.len());

    let mut candidate_lines = CandidateLines { lrp, lines: vec![] };

    for CandidateNode {
        vertex,
        distance_to_lrp,
    } in nodes
    {
        // only outgoing lines are accepted for the LRPs
        // except for the last LRP where only incoming lines are accepted
        let edges: Box<dyn Iterator<Item = _>> = if lrp.is_last() {
            trace!("Finding entering edges to {vertex:?}");
            Box::new(graph.vertex_entering_edges(vertex))
        } else {
            trace!("Finding exiting edges from {vertex:?}");
            Box::new(graph.vertex_exiting_edges(vertex))
        };

        let candidates = edges.into_iter().filter_map(|(edge, _)| {
            let bearing = if lrp.is_last() {
                graph.get_edge_bearing(
                    edge,
                    graph.get_edge_length(edge)?,
                    config.bearing_distance.reverse(),
                )?
            } else {
                graph.get_edge_bearing(edge, Length::ZERO, config.bearing_distance)?
            };

            let line = ProvisionalCandidateLine {
                lrp,
                edge,
                distance_to_lrp,
                distance_to_projection: None,
                frc: graph.get_edge_frc(edge)?,
                fow: graph.get_edge_fow(edge)?,
                bearing,
            };

            rate_line(config, lrp, line).inspect(|line| debug!("Accepted candidate: {line:?}"))
        });

        candidate_lines.lines.extend(candidates);
    }

    candidate_lines
}

fn append_projected_candidate_lines<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &mut CandidateLines<G::EdgeId>,
) {
    let lrp = candidate_lines.lrp;
    debug!("Finding candidates from projected lines of {lrp:?}");

    let projected_lines = graph
        .nearest_edges_within_distance(lrp.coordinate, config.max_node_distance)
        .filter_map(|(edge, distance_to_lrp)| {
            debug_assert!(distance_to_lrp <= config.max_node_distance);

            let distance_to_projection = graph
                .get_distance_along_edge(edge, lrp.coordinate)
                .filter(|&distance| distance > Length::ZERO)?;

            let bearing = if lrp.is_last() {
                graph.get_edge_bearing(
                    edge,
                    distance_to_projection,
                    config.bearing_distance.reverse(),
                )?
            } else {
                graph.get_edge_bearing(edge, distance_to_projection, config.bearing_distance)?
            };

            let line = ProvisionalCandidateLine {
                lrp,
                edge,
                distance_to_lrp,
                distance_to_projection: Some(distance_to_projection),
                frc: graph.get_edge_frc(edge)?,
                fow: graph.get_edge_fow(edge)?,
                bearing,
            };

            rate_line(config, lrp, line)
        });

    for mut projected_line in projected_lines {
        if !candidate_lines.lines.is_empty() {
            projected_line.rating *= config.projected_line_factor;
            if projected_line.rating < config.min_line_rating {
                debug!("Discarding {projected_line:?} because rating became too low");
                continue;
            }
        }

        if let Some(candidate) = candidate_lines
            .lines
            .iter_mut()
            .find(|candidate| candidate.edge == projected_line.edge)
        {
            debug_assert_eq!(candidate.lrp, projected_line.lrp);

            if candidate.rating < projected_line.rating {
                debug!("Overriding candidate line with {projected_line:?}");
                candidate.rating = projected_line.rating;
                candidate.distance_to_projection = projected_line.distance_to_projection;
            } else {
                debug!("Discarding {projected_line:?}: already exists with better rating");
            }
        } else {
            debug!("Accepted projected candidate: {projected_line:?}");
            candidate_lines.lines.push(projected_line);
        }
    }
}

/// All candidate lines for a location reference point shall be rated according to the following
/// criteria:
/// - The start node, end node for the last location reference point or projection point shall be as
///   close as possible to the coordinates of the location reference point.
/// - The functional road class of the candidate line should match the functional road class of the
///   location reference point.
/// - The form of way of the candidate line should match the form of way of the location reference
///   point.
/// - The bearing of the candidate line should match indicated bearing angles of the location
///   reference point.
///
/// Slight variances in the concrete values are allowed and shall be considered in the rating
/// function.
///
/// The candidate lines should be ordered in a way that the best matching line comes first.
fn rate_line<EdgeId: Debug>(
    config: &DecoderConfig,
    lrp: Point,
    line: ProvisionalCandidateLine<EdgeId>,
) -> Option<CandidateLine<EdgeId>> {
    trace!("Rating: {line:?}");

    if let Some(path) = &lrp.path
        && !line.frc.is_within_variance(&path.lfrcnp)
    {
        trace!("Candidate FRC variance out of bounds: {line:?}");
        return None;
    }

    if line.bearing.difference(&lrp.line.bearing) > config.max_bearing_difference {
        trace!("Candidate bearing out of bounds: {line:?}");
        return None;
    }

    #[derive(Debug)]
    struct Ratings {
        distance: RatingScore,
        bearing: RatingScore,
        frc: RatingScore,
        fow: RatingScore,
    }

    let distance = (config.max_node_distance - line.distance_to_lrp).max(Length::ZERO);

    let ratings = Ratings {
        distance: RatingScore::from(distance),
        bearing: Bearing::rating_score(line.bearing.rating(&lrp.line.bearing)),
        frc: Frc::rating_score(line.frc.rating(&lrp.line.frc)),
        fow: Fow::rating_score(line.fow.rating(&lrp.line.fow)),
    };

    let node_rating = config.node_factor * ratings.distance;
    let line_rating = config.line_factor * (ratings.bearing + ratings.frc + ratings.fow);
    let rating = node_rating + line_rating;

    if rating < config.min_line_rating {
        trace!("Rating {:?} too low = {rating:?} {ratings:?}", line.edge);
        None
    } else {
        trace!("Rating {:?} accepted = {rating:?} {ratings:?}", line.edge);
        Some(CandidateLine {
            lrp: line.lrp,
            edge: line.edge,
            distance_to_projection: line.distance_to_projection,
            rating,
        })
    }
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};
    use crate::{Coordinate, LineAttributes, PathAttributes};

    #[test]
    fn decoder_find_candidate_nodes_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig {
            max_node_distance: Length::from_meters(10.0),
            ..Default::default()
        };

        let points = [
            Point {
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
            },
            Point {
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
        ];

        let nodes: Vec<_> = find_candidate_nodes(&config, graph, &points)
            .flat_map(|candidate| candidate.nodes)
            .map(|node| (node.vertex, (node.distance_to_lrp.meters() * 100.0).round()))
            .collect();

        assert_eq!(nodes, [(VertexId(68), 174.0), (VertexId(20), 216.0)]);
    }

    #[test]
    fn decoder_find_candidate_nodes_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig {
            max_node_distance: Length::from_meters(100.0),
            ..Default::default()
        };

        let points = [Point {
            coordinate: Coordinate {
                lon: 13.454789,
                lat: 52.5157088,
            },
            line: LineAttributes::default(),
            path: None,
        }];

        let nodes: Vec<_> = find_candidate_nodes(&config, graph, &points)
            .flat_map(|candidate| candidate.nodes)
            .map(|node| (node.vertex, node.distance_to_lrp.meters().round()))
            .collect();

        assert_eq!(
            nodes,
            [
                (VertexId(37), 37.0),
                (VertexId(1), 39.0),
                (VertexId(105), 55.0)
            ]
        );
    }

    #[test]
    fn decoder_find_candidate_nodes_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig {
            max_node_distance: Length::from_meters(5.0),
            ..Default::default()
        };

        let lrp = Point {
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

        let nodes: Vec<_> = find_candidate_nodes(&config, graph, &[lrp]).collect();
        assert_eq!(nodes, [CandidateNodes { lrp, nodes: vec![] }]);
    }

    #[test]
    fn decoder_find_candidate_nodes_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig {
            max_node_distance: Length::from_meters(100.0),
            ..Default::default()
        };

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

        let nodes: Vec<_> = find_candidate_nodes(&config, graph, [first_lrp, last_lrp].iter())
            .map(|candidate| {
                candidate
                    .nodes
                    .into_iter()
                    .map(|node| node.vertex)
                    .collect::<Vec<_>>()
            })
            .collect();

        assert_eq!(
            nodes,
            [
                vec![VertexId(68), VertexId(93), VertexId(92)],
                vec![
                    VertexId(95),
                    VertexId(93),
                    VertexId(92),
                    VertexId(19),
                    VertexId(68)
                ]
            ]
        );
    }

    #[test]
    fn decoder_find_candidate_lines_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig {
            max_node_distance: Length::from_meters(100.0),
            max_bearing_difference: Bearing::from_degrees(90),
            min_line_rating: RatingScore::from(800.0),
            ..Default::default()
        };

        let points = [
            CandidateNodes {
                lrp: Point {
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
                },
                nodes: vec![CandidateNode {
                    vertex: VertexId(68),
                    distance_to_lrp: Length::from_meters(1.74),
                }],
            },
            CandidateNodes {
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
                nodes: vec![CandidateNode {
                    vertex: VertexId(20),
                    distance_to_lrp: Length::from_meters(2.16),
                }],
            },
        ];

        let lines: Vec<_> = find_candidate_lines(&config, graph, points)
            .unwrap()
            .into_iter()
            .map(|candidate| {
                candidate
                    .lines
                    .into_iter()
                    .map(|line| {
                        assert!(line.rating >= config.min_line_rating);
                        (line.edge, line.distance_to_projection.map(|d| d.round()))
                    })
                    .collect::<Vec<_>>()
            })
            .collect();

        assert_eq!(
            lines,
            [
                vec![
                    (EdgeId(8717174), None),
                    (EdgeId(4925291), Some(Length::from_meters(142.0))) // projected line
                ],
                vec![(EdgeId(109783), None)]
            ]
        );
    }

    #[test]
    fn decoder_find_candidate_lines_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = DecoderConfig {
            max_node_distance: Length::from_meters(100.0),
            max_bearing_difference: Bearing::from_degrees(90),
            min_line_rating: RatingScore::from(800.0),
            ..Default::default()
        };

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

        let points = [
            CandidateNodes {
                lrp: first_lrp,
                nodes: vec![CandidateNode {
                    vertex: VertexId(68),
                    distance_to_lrp: Length::from_meters(30.0),
                }],
            },
            CandidateNodes {
                lrp: last_lrp,
                nodes: vec![CandidateNode {
                    vertex: VertexId(68),
                    distance_to_lrp: Length::from_meters(100.0),
                }],
            },
        ];

        let lines = find_candidate_lines(&config, graph, points).unwrap();

        let lines: Vec<_> = lines
            .into_iter()
            .map(|candidate| {
                candidate
                    .lines
                    .into_iter()
                    .map(|line| {
                        assert!(line.rating >= config.min_line_rating);
                        (line.edge, line.distance_to_projection.map(|d| d.round()))
                    })
                    .collect::<Vec<_>>()
            })
            .collect();

        assert_eq!(
            lines,
            [
                vec![(EdgeId(8717174), Some(Length::from_meters(29.0)))],
                vec![
                    (EdgeId(8717174), Some(Length::from_meters(99.0))),
                    (EdgeId(4925291), None)
                ]
            ]
        );
    }

    #[test]
    fn decoder_find_candidate_lines_003() {
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

        let points = [
            CandidateNodes {
                lrp: first_lrp,
                nodes: vec![CandidateNode {
                    vertex: VertexId(68),
                    distance_to_lrp: Length::from_meters(1.74),
                }],
            },
            CandidateNodes {
                lrp: second_lrp,
                nodes: vec![CandidateNode {
                    vertex: VertexId(20),
                    distance_to_lrp: Length::from_meters(2.16),
                }],
            },
            CandidateNodes {
                lrp: last_lrp,
                nodes: vec![CandidateNode {
                    vertex: VertexId(7),
                    distance_to_lrp: Length::from_meters(8.0),
                }],
            },
        ];

        let lines = find_candidate_lines(&config, graph, points).unwrap();

        let lines: Vec<_> = lines
            .into_iter()
            .map(|candidate| {
                candidate
                    .lines
                    .into_iter()
                    .map(|line| {
                        assert!(line.rating >= config.min_line_rating);
                        (line.edge, line.distance_to_projection.map(|d| d.round()))
                    })
                    .collect::<Vec<_>>()
            })
            .collect();

        assert_eq!(
            lines,
            [
                vec![
                    (EdgeId(8717174), None),
                    (EdgeId(4925291), Some(Length::from_meters(142.0))) // projected line
                ],
                vec![
                    (EdgeId(6770340), None),
                    (EdgeId(109783), Some(Length::from_meters(191.0)))
                ],
                vec![(EdgeId(7531947), None)]
            ]
        );
    }
}
