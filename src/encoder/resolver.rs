use tracing::warn;

use crate::EncoderError::InvalidLrp;
use crate::encoder::lrp::{LocRefPoint, LocRefPoints};
use crate::encoder::shortest_path::{Intermediate, ShortestPath, shortest_path_location};
use crate::{DirectedGraph, EncoderConfig, EncoderError, LineLocation};

/// Resolves all the LRPs that should be necessary to encode the given line.
pub fn resolve_lrps<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: LineLocation<G::EdgeId>,
) -> Result<LocRefPoints<G::EdgeId>, EncoderError> {
    let mut location: Vec<G::EdgeId> = line.path.clone();

    let last_edge = location[location.len() - 1];
    let mut candidate_lrps = vec![];

    // Step – 7 Find shortest paths until the whole location is covered by a concatenation of these.
    while !location.is_empty() {
        // Step - 3 Determine coverage of the location by a shortest-path.
        match shortest_path_location(graph, &location, config.max_lrp_distance)? {
            // Step – 4 Check whether the calculated shortest-path covers the location completely.
            ShortestPath::Location => {
                candidate_lrps.push(LocRefPoint::node(config, graph, location));
                break;
            }
            // Step – 6 Restart shortest path calculation between the new intermediate location
            // reference point and the end of the location.
            ShortestPath::Intermediate(Intermediate { location_index }) => {
                let loc = location[..location_index].to_vec();
                candidate_lrps.push(LocRefPoint::node(config, graph, loc));
                location.drain(..location_index);
            }
            ShortestPath::NotFound => {
                return Err(EncoderError::RouteNotFound);
            }
        }
    }

    candidate_lrps.push(LocRefPoint::last_node(config, graph, last_edge));

    let lrp_edges = || candidate_lrps.iter().flat_map(|lrp| &lrp.edges);
    debug_assert_eq!(line.path.len(), lrp_edges().count());

    // Step – 8 Check validity of the location reference path.
    if line.path.iter().zip(lrp_edges()).any(|(e1, e2)| e1 != e2) {
        warn!("Resolved LRPs don't exactly cover the location edges");
        return Err(InvalidLrp);
    }

    // Step – 9 Add a sufficient number of additional intermediate location reference points if the
    // distance between two location reference points exceeds the maximum distance.
    let mut lrps = Vec::with_capacity(candidate_lrps.len());
    for lrp in candidate_lrps {
        lrps.append(&mut split_lrp(config, graph, lrp)?);
    }

    Ok(LocRefPoints {
        lrps,
        pos_offset: line.pos_offset,
        neg_offset: line.neg_offset,
    })
}

/// If the maximum distance between two subsequent location reference points is exceeded additional
/// location reference points shall be placed at valid nodes along the location reference path
/// between these two location reference points. If placing on valid nodes is not possible an
/// invalid node shall be used. Examples for such a case are lines which are longer than 15 km. In
/// such a case the coordinates of additional LRP may be resolved from a point being less than 15 km
/// away from the start node of that line. Such a point is directly on the line and is not
/// represented by a node. All LRP attributes need to be calculated relative to this point. The
/// calculated attributes need to be updated and the new intermediate location reference point must
/// be added to the list without breaking the order from start to end.
fn split_lrp<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    lrp: LocRefPoint<G::EdgeId>,
) -> Result<Vec<LocRefPoint<G::EdgeId>>, EncoderError> {
    let EncoderConfig {
        max_lrp_distance, ..
    } = *config;

    let mut lrps = vec![lrp];
    let lrp = &lrps[0];

    if lrp.point.dnp() <= max_lrp_distance {
        return Ok(lrps);
    }

    // long single line was not handled during the shortest route stage when finding intermediates
    debug_assert!(!lrp.point.is_last());
    debug_assert_eq!(lrp.edges.len(), 1);
    debug_assert!(lrp.projection_coordinate.is_none());

    let edge = lrp.edges[0];
    let mut dnp = lrp.point.dnp();
    let mut distance = max_lrp_distance;

    while dnp > max_lrp_distance {
        let coordinate = graph.get_coordinate_along_edge(edge, distance);

        if let Some(path) = lrps.last_mut().and_then(|lrp| lrp.point.path.as_mut()) {
            // creating another LRP on the same line requires updating the DNP of the previous
            path.dnp = max_lrp_distance;
        }

        lrps.push(LocRefPoint::line(config, graph, edge, coordinate));
        dnp -= max_lrp_distance;
        distance += max_lrp_distance;
    }

    debug_assert!(lrps.iter().all(|lrp| lrp.point.dnp() <= max_lrp_distance));
    Ok(lrps)
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
    use crate::{Bearing, Coordinate, Fow, Frc, Length, LineAttributes, PathAttributes, Point};

    #[test]
    fn encoder_resolve_lrps_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(9044472)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(9044472)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.459407,
                            lat: 52.5143601
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(303),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(14.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4592303,
                            lat: 52.5144292
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(123),
                        },
                        path: None
                    }
                },
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(-9044470), EdgeId(-9044471), EdgeId(-9044472)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-9044470), EdgeId(-9044471), EdgeId(-9044472)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.458825,
                            lat: 52.5145838
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(122),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(45.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.459407,
                            lat: 52.5143601
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(303),
                        },
                        path: None
                    }
                },
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![
                EdgeId(-7292030),
                EdgeId(-7292029),
                EdgeId(7516886),
                EdgeId(7516883),
                EdgeId(7516885),
            ],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-7292030)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4571122,
                            lat: 52.5177995
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(20),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(108.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![
                        EdgeId(-7292029),
                        EdgeId(7516886),
                        EdgeId(7516883),
                        EdgeId(7516885)
                    ],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4576677,
                            lat: 52.518717
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(20),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(94.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4580594,
                            lat: 52.5186534
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(286),
                        },
                        path: None
                    }
                }
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![
                EdgeId(7516884),
                EdgeId(7516884), // loop into first line
                EdgeId(7516885),
            ],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(7516884)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4576677,
                            lat: 52.518717
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(105),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(17.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(7516884), EdgeId(7516885)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4576677,
                            lat: 52.518717
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(105),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(27.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4580594,
                            lat: 52.5186534
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(286),
                        },
                        path: None
                    }
                }
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_005() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![
                EdgeId(-7516884),
                EdgeId(-7292029),
                EdgeId(7516886),
                EdgeId(7516883),
                EdgeId(-7516884), // loop into origin
                EdgeId(7292030),
            ],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-7516884), EdgeId(-7292029), EdgeId(7516886)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4579134,
                            lat: 52.5186781
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(285),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(67.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(7516883), EdgeId(-7516884), EdgeId(7292030)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4580688,
                            lat: 52.5189743
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(198),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(159.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4571122,
                            lat: 52.5177995
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(20),
                        },
                        path: None
                    }
                }
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_006() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![
                EdgeId(-7516885),
                EdgeId(-7516884), // loop into destination
                EdgeId(-7292029),
                EdgeId(7516886),
                EdgeId(7516883),
                EdgeId(-7516884),
            ],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-7516885)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4580594,
                            lat: 52.5186534
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(286),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(10.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-7516884)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4579134,
                            lat: 52.5186781
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(285),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(17.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-7292029), EdgeId(7516886), EdgeId(7516883)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4576677,
                            lat: 52.518717
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(20),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(84.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-7516884)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4579134,
                            lat: 52.5186781
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(285),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(17.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4576677,
                            lat: 52.518717
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(105),
                        },
                        path: None
                    }
                }
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_007() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4611206,
                            lat: 52.5170944
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(106),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(379.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4628442,
                            lat: 52.5149807
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(18),
                        },
                        path: None
                    }
                }
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_008() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![
                EdgeId(1653344),
                EdgeId(4997411),
                EdgeId(5359424),
                EdgeId(5359425),
            ],
            pos_offset: Length::from_meters(11.0),
            neg_offset: Length::from_meters(14.0),
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![
                        EdgeId(1653344),
                        EdgeId(4997411),
                        EdgeId(5359424),
                        EdgeId(5359425),
                    ],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4659771,
                            lat: 52.5181688
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(287),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(489.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4611206,
                            lat: 52.5170944
                        },
                        line: LineAttributes {
                            frc: Frc::Frc4,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(17),
                        },
                        path: None
                    }
                }
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_009() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(16218)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(16218)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.454214,
                            lat: 52.5157088
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(99),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(217.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.457386,
                            lat: 52.5153814
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(280),
                        },
                        path: None
                    }
                },
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_010() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig::default();

        let line = LineLocation {
            path: vec![EdgeId(16218), EdgeId(16219)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(16218), EdgeId(16219)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.454214,
                            lat: 52.5157088
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(99),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(326.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4589756,
                            lat: 52.515216
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(280),
                        },
                        path: None
                    }
                },
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_011() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig {
            max_lrp_distance: Length::from_meters(250.0),
            ..Default::default()
        };

        let line = LineLocation {
            path: vec![EdgeId(16218), EdgeId(16219)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(16218)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.454214,
                            lat: 52.5157088
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(99),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(217.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(16219)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.457386,
                            lat: 52.5153814
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(100),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(109.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4589756,
                            lat: 52.515216
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(280),
                        },
                        path: None
                    }
                },
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_012() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig {
            max_lrp_distance: Length::from_meters(100.0),
            ..Default::default()
        };

        let line = LineLocation {
            path: vec![EdgeId(16218), EdgeId(16219)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(16218)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.454214,
                            lat: 52.5157088
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(99),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(100.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: Some(Coordinate {
                        lon: 13.455676767654381,
                        lat: 52.5155615984457,
                    }),
                    edges: vec![EdgeId(16218)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.455676,
                            lat: 52.515561
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(100),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(100.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: Some(Coordinate {
                        lon: 13.457137508978576,
                        lat: 52.51540708300186,
                    }),
                    edges: vec![EdgeId(16218)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.457137,
                            lat: 52.515407
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(100),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(16.346160186372742)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(16219)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.457386,
                            lat: 52.5153814
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(100),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(100.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: Some(Coordinate {
                        lon: 13.458844359049749,
                        lat: 52.515229693289946,
                    },),
                    edges: vec![EdgeId(16219)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.458844,
                            lat: 52.515229
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(100),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc2,
                            dnp: Length::from_meters(8.884732961834075)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4589756,
                            lat: 52.515216
                        },
                        line: LineAttributes {
                            frc: Frc::Frc2,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(280),
                        },
                        path: None
                    }
                },
            ]
        );
    }

    #[test]
    fn encoder_resolve_lrps_013() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let config = EncoderConfig {
            max_lrp_distance: Length::from_meters(15.0),
            ..Default::default()
        };

        let line = LineLocation {
            path: vec![EdgeId(-9044470), EdgeId(-9044471), EdgeId(-9044472)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        };

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-9044470)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.458825,
                            lat: 52.5145838
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(122),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(15.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: Some(Coordinate {
                        lon: 13.459018736929124,
                        lat: 52.51450982635798,
                    }),
                    edges: vec![EdgeId(-9044470)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.459018,
                            lat: 52.514509
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(122),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(3.523195526723825)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-9044471)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4590704,
                            lat: 52.5144901
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(122),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(12.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![EdgeId(-9044472)],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4592303,
                            lat: 52.5144292
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(123),
                        },
                        path: Some(PathAttributes {
                            lfrcnp: Frc::Frc6,
                            dnp: Length::from_meters(14.0)
                        })
                    }
                },
                LocRefPoint {
                    projection_coordinate: None,
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.459407,
                            lat: 52.5143601
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(303),
                        },
                        path: None
                    }
                },
            ]
        );
    }
}
