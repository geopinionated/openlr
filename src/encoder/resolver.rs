use tracing::warn;

use crate::encoder::lrp::{LocRefPoint, LocRefPoints};
use crate::encoder::shortest_path::{IntermediateLocation, ShortestRoute, shortest_path_location};
use crate::{DirectedGraph, EncoderConfig, EncoderError, LineLocation, LocationError};

/// Resolves all the LRPs that should be necessary to encode the given line, and its expansion.
pub fn resolve_lrps<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
) -> Result<LocRefPoints<G::EdgeId>, EncoderError> {
    let mut location: Vec<G::EdgeId> = line.path.clone();

    let last_edge = if let Some(&last_edge) = location.last() {
        last_edge
    } else {
        return Err(LocationError::Empty.into());
    };

    let last_lrp = LocRefPoint::from_last_node(config, graph, last_edge)?;
    let mut lrps = vec![];

    // Step - 3 Determine coverage of the location by a shortest-path.
    // Find shortest paths until the whole location is covered by a concatenation of these.
    while !location.is_empty() {
        match shortest_path_location(graph, &location, config.max_lrp_distance)? {
            // Step – 4 Check whether the calculated shortest-path covers the location completely.
            ShortestRoute::Location => {
                let lrp = LocRefPoint::from_node(config, graph, location)?;
                lrps.push(lrp);
                break;
            }
            // Step – 6 Restart shortest path calculation between the new intermediate location
            // reference point and the end of the location.
            ShortestRoute::Intermediate(intermediate) => {
                let IntermediateLocation { location_index, .. } = intermediate;
                let mut intermediates = intermediate_lrps(config, graph, &location, intermediate)?;
                lrps.append(&mut intermediates);
                location.drain(..location_index);
            }
            ShortestRoute::NotFound => {
                return Err(EncoderError::RouteNotFound);
            }
        }
    }

    lrps.push(last_lrp);

    // Step – 8 Check validity of the location reference path.
    debug_assert!(
        line.path
            .iter()
            .zip(lrps.iter().flat_map(|lrp| &lrp.edges))
            .all(|(e1, e2)| e1 == e2),
        "Resolved LRPs don't cover the exact expanded location edges"
    );

    // TODO: determine new intermediate LRPs if the maximum distance was exceeded.
    if let Some(lrp) = lrps
        .iter()
        .find(|lrp| lrp.point.dnp() > config.max_lrp_distance)
    {
        warn!("Maximum LRP distance exceeded by {lrp:?}");
        return Err(EncoderError::MaxDistanceExceeded);
    }

    Ok(LocRefPoints {
        lrps,
        pos_offset: line.pos_offset,
        neg_offset: line.neg_offset,
    })
}

fn intermediate_lrps<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    location: &[G::EdgeId],
    intermediate: IntermediateLocation,
) -> Result<Vec<LocRefPoint<G::EdgeId>>, EncoderError> {
    let location = location[..intermediate.location_index].to_vec();
    let lrp = LocRefPoint::from_node(config, graph, location)?;
    Ok(vec![lrp])
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::network::{EdgeId, NETWORK_GRAPH, NetworkGraph};
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

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, &line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
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

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, &line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
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

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, &line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
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

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, &line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
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

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, &line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
                    edges: vec![
                        EdgeId(-7516884),
                        EdgeId(-7292029),
                        EdgeId(7516886),
                        EdgeId(7516883)
                    ],
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
                            dnp: Length::from_meters(101.0)
                        })
                    }
                },
                LocRefPoint {
                    edges: vec![EdgeId(-7516884), EdgeId(7292030)],
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
                            dnp: Length::from_meters(125.0)
                        })
                    }
                },
                LocRefPoint {
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

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, &line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
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
                    edges: vec![
                        EdgeId(-7292029),
                        EdgeId(7516886),
                        EdgeId(7516883),
                        EdgeId(-7516884)
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
                            dnp: Length::from_meters(101.0)
                        })
                    }
                },
                LocRefPoint {
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

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, &line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
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

        let LocRefPoints { lrps, .. } = resolve_lrps(&config, graph, &line).unwrap();

        assert_eq!(
            lrps,
            [
                LocRefPoint {
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
}
