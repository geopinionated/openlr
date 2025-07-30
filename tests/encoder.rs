mod graph;

use openlr::{
    Bearing, Coordinate, DecoderConfig, DirectedGraph, EncoderConfig, Frc, Length, LineAttributes,
    LineLocation, LocRefPoint, LocRefPoints, Location, Path, PathAttributes, Point,
    decode_base64_openlr, edge_backward_expansion, edge_forward_expansion, encode_base64_openlr,
    is_node_valid, is_opposite_direction, resolve_lrps, select_edge_expansion_candidate,
};
use test_log::test;

use crate::graph::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};

#[test]
fn encoder_encode_line_location_reference_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = Location::Line(LineLocation {
        path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    });

    let encoded = encode_base64_openlr(&EncoderConfig::default(), graph, location.clone()).unwrap();
    let decoded = decode_base64_openlr(&DecoderConfig::default(), graph, &encoded).unwrap();
    assert_eq!(decoded, location);
}

#[test]
fn encoder_encode_line_location_reference_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = Location::Line(LineLocation {
        path: vec![
            EdgeId(1653344),
            EdgeId(4997411),
            EdgeId(5359424),
            EdgeId(5359425),
        ],
        pos_offset: Length::from_meters(11.0),
        neg_offset: Length::from_meters(14.0),
    });

    let encoded = encode_base64_openlr(&EncoderConfig::default(), graph, location.clone()).unwrap();
    let decoded = decode_base64_openlr(&DecoderConfig::default(), graph, &encoded).unwrap();
    assert_eq!(decoded, location);
}

#[test]
fn encoder_is_opposite_direction_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert!(!is_opposite_direction(graph, EdgeId(16218), EdgeId(16218)));
    assert!(!is_opposite_direction(graph, EdgeId(16218), EdgeId(16219)));
    assert!(!is_opposite_direction(graph, EdgeId(16219), EdgeId(16218)));
    assert!(!is_opposite_direction(graph, EdgeId(16218), EdgeId(961826)));
    assert!(!is_opposite_direction(
        graph,
        EdgeId(-5707439),
        EdgeId(-8717174)
    ));

    assert!(is_opposite_direction(
        graph,
        EdgeId(4925290),
        EdgeId(-4925290)
    ));
    assert!(is_opposite_direction(
        graph,
        EdgeId(8345025),
        EdgeId(-8345025)
    ));
}

#[test]
fn encoder_is_valid_node_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    // 1 ---> 2
    assert_eq!(graph.vertex_degree(VertexId(1)), 1);
    assert!(is_node_valid(graph, VertexId(1)));

    // 2 ---> 3 ---> 34
    assert_eq!(graph.vertex_degree(VertexId(3)), 2);
    assert!(
        !is_node_valid(graph, VertexId(3)),
        "2nd degree and not a dead-end"
    );

    // 105 <====> 58
    assert_eq!(graph.vertex_degree(VertexId(105)), 2);
    assert!(is_node_valid(graph, VertexId(105)), "2nd degree dead-end");

    // 1 ---> 2 ---> 3
    //       ||
    //       58
    assert_eq!(graph.vertex_degree(VertexId(2)), 4);
    assert!(is_node_valid(graph, VertexId(2)));

    // 139 <====> 138 <====> 140
    assert_eq!(graph.vertex_degree(VertexId(138)), 4);
    assert!(!is_node_valid(graph, VertexId(138)));

    assert_eq!(graph.vertex_degree(VertexId(75)), 6);
    assert!(is_node_valid(graph, VertexId(75)));

    assert_eq!(graph.vertex_degree(VertexId(20)), 6);
    assert!(is_node_valid(graph, VertexId(20)));

    assert_eq!(graph.vertex_degree(VertexId(68)), 8);
    assert!(is_node_valid(graph, VertexId(68)));
}

#[test]
fn encoder_select_edge_expansion_candidate_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        select_edge_expansion_candidate(graph, EdgeId(16218), []),
        None
    );

    assert_eq!(
        select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219)]),
        Some(EdgeId(16219))
    );

    assert_eq!(
        select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219), EdgeId(3622025)]),
        None
    );

    assert_eq!(
        select_edge_expansion_candidate(graph, EdgeId(16218), [EdgeId(16219), EdgeId(3622025)]),
        None
    );

    assert_eq!(
        select_edge_expansion_candidate(graph, EdgeId(3622025), [EdgeId(-3622025), EdgeId(16219)]),
        Some(EdgeId(16219))
    );

    assert_eq!(
        select_edge_expansion_candidate(
            graph,
            EdgeId(7020005),
            [EdgeId(-7020005), EdgeId(3622025), EdgeId(3622026)]
        ),
        None
    );
}

#[test]
fn encoder_edge_expansion_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

    let line = LineLocation {
        path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    };

    assert_eq!(
        edge_forward_expansion(&config, graph, &line),
        Path::default(),
        "End VertexId(20) is a valid node"
    );

    assert_eq!(
        edge_backward_expansion(&config, graph, &line),
        Path::default(),
        "Start VertexId(68) is a valid node"
    );
}

#[test]
fn encoder_edge_expansion_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

    let line = LineLocation {
        path: vec![EdgeId(16219)],
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    };

    assert_eq!(
        edge_forward_expansion(&config, graph, &line),
        Path {
            edges: vec![EdgeId(7430347)],
            length: Length::from_meters(78.0)
        },
        "End VertexId(3) is not a valid node"
    );

    assert_eq!(
        edge_backward_expansion(&config, graph, &line),
        Path::default(),
        "Start VertexId(2) is a valid node"
    );
}

#[test]
fn encoder_edge_expansion_003() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

    let line = LineLocation {
        path: vec![EdgeId(7430347)],
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    };

    assert_eq!(
        edge_forward_expansion(&config, graph, &line),
        Path::default(),
        "End VertexId(34) is a valid node"
    );

    assert_eq!(
        edge_backward_expansion(&config, graph, &line),
        Path {
            edges: vec![EdgeId(16219)],
            length: Length::from_meters(109.0)
        },
        "Start VertexId(3) is not a valid node"
    );
}

#[test]
fn encoder_edge_expansion_004() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

    let line = LineLocation {
        path: vec![EdgeId(-9044470)],
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    };

    assert_eq!(
        edge_forward_expansion(&config, graph, &line),
        Path {
            edges: vec![EdgeId(-9044471), EdgeId(-9044472)],
            length: Length::from_meters(26.0)
        },
        "End VertexId(138) is not a valid node"
    );
}

#[test]
fn encoder_edge_expansion_005() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

    let line = LineLocation {
        path: vec![EdgeId(-9044472)],
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    };

    assert_eq!(
        edge_backward_expansion(&config, graph, &line),
        Path {
            edges: vec![EdgeId(-9044470), EdgeId(-9044471)],
            length: Length::from_meters(31.0)
        },
        "Start VertexId(140) is not a valid node"
    );
}

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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
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
                        fow: openlr::Fow::SingleCarriageway,
                        bearing: Bearing::from_degrees(17),
                    },
                    path: None
                }
            }
        ]
    );
}

#[test]
fn encoder_trim_lrps_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

    let lrps = vec![
        LocRefPoint {
            edges: vec![EdgeId(9044472)],
            point: Point {
                coordinate: Coordinate {
                    lon: 13.459407,
                    lat: 52.5143601,
                },
                line: LineAttributes {
                    frc: Frc::Frc6,
                    fow: openlr::Fow::SingleCarriageway,
                    bearing: Bearing::from_degrees(303),
                },
                path: Some(PathAttributes {
                    lfrcnp: Frc::Frc6,
                    dnp: Length::from_meters(14.0),
                }),
            },
        },
        LocRefPoint {
            edges: vec![],
            point: Point {
                coordinate: Coordinate {
                    lon: 13.4592303,
                    lat: 52.5144292,
                },
                line: LineAttributes {
                    frc: Frc::Frc6,
                    fow: openlr::Fow::SingleCarriageway,
                    bearing: Bearing::from_degrees(123),
                },
                path: None,
            },
        },
    ];

    assert_eq!(
        LocRefPoints {
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
            lrps: lrps.clone()
        }
        .trim(&config, graph)
        .unwrap(),
        LocRefPoints {
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
            lrps: lrps.clone()
        }
    );
}

#[test]
fn encoder_trim_lrps_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

    let lrps = vec![
        LocRefPoint {
            edges: vec![EdgeId(-7292030)],
            point: Point {
                coordinate: Coordinate {
                    lon: 13.4571122,
                    lat: 52.5177995,
                },
                line: LineAttributes {
                    frc: Frc::Frc6,
                    fow: openlr::Fow::SingleCarriageway,
                    bearing: Bearing::from_degrees(20),
                },
                path: Some(PathAttributes {
                    lfrcnp: Frc::Frc6,
                    dnp: Length::from_meters(108.0),
                }),
            },
        },
        LocRefPoint {
            edges: vec![
                EdgeId(-7292029),
                EdgeId(7516886),
                EdgeId(7516883),
                EdgeId(7516885),
            ],
            point: Point {
                coordinate: Coordinate {
                    lon: 13.4576677,
                    lat: 52.518717,
                },
                line: LineAttributes {
                    frc: Frc::Frc6,
                    fow: openlr::Fow::SingleCarriageway,
                    bearing: Bearing::from_degrees(20),
                },
                path: Some(PathAttributes {
                    lfrcnp: Frc::Frc6,
                    dnp: Length::from_meters(94.0),
                }),
            },
        },
        LocRefPoint {
            edges: vec![],
            point: Point {
                coordinate: Coordinate {
                    lon: 13.4580594,
                    lat: 52.5186534,
                },
                line: LineAttributes {
                    frc: Frc::Frc6,
                    fow: openlr::Fow::SingleCarriageway,
                    bearing: Bearing::from_degrees(286),
                },
                path: None,
            },
        },
    ];

    assert_eq!(
        LocRefPoints {
            pos_offset: Length::from_meters(1.0),
            neg_offset: Length::ZERO,
            lrps: lrps.clone()
        }
        .trim(&config, graph)
        .unwrap(),
        LocRefPoints {
            pos_offset: Length::from_meters(1.0),
            neg_offset: Length::ZERO,
            lrps: lrps.clone()
        }
    );

    assert_eq!(
        LocRefPoints {
            pos_offset: Length::from_meters(108.0),
            neg_offset: Length::ZERO,
            lrps: lrps.clone()
        }
        .trim(&config, graph)
        .unwrap(),
        LocRefPoints {
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
            lrps: lrps[1..].to_vec()
        }
    );

    assert_eq!(
        LocRefPoints {
            pos_offset: Length::from_meters(1.0),
            neg_offset: Length::from_meters(1.0),
            lrps: lrps.clone()
        }
        .trim(&config, graph)
        .unwrap(),
        LocRefPoints {
            pos_offset: Length::from_meters(1.0),
            neg_offset: Length::from_meters(1.0),
            lrps: lrps.clone()
        }
    );

    assert_eq!(
        LocRefPoints {
            pos_offset: Length::from_meters(109.0),
            neg_offset: Length::ZERO,
            lrps: lrps.clone()
        }
        .trim(&config, graph)
        .unwrap(),
        LocRefPoints {
            pos_offset: Length::from_meters(1.0),
            neg_offset: Length::ZERO,
            lrps: lrps[1..].to_vec()
        }
    );

    assert_eq!(
        LocRefPoints {
            pos_offset: Length::ZERO,
            neg_offset: Length::from_meters(95.0),
            lrps: lrps.clone()
        }
        .trim(&config, graph)
        .unwrap(),
        LocRefPoints {
            pos_offset: Length::ZERO,
            neg_offset: Length::from_meters(1.0),
            lrps: vec![
                lrps[0].clone(),
                LocRefPoint {
                    edges: vec![],
                    point: Point {
                        coordinate: Coordinate {
                            lon: 13.4576677,
                            lat: 52.518717
                        },
                        line: LineAttributes {
                            frc: Frc::Frc6,
                            fow: openlr::Fow::SingleCarriageway,
                            bearing: Bearing::from_degrees(200),
                        },
                        path: None
                    }
                }
            ]
        }
    );
}
