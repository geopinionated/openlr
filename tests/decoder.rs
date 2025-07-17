mod graph;

use openlr::{
    Bearing, CandidateLine, CandidateLinePair, CandidateLines, CandidateNode, CandidateNodes,
    Coordinate, DecoderConfig, Fow, Frc, Length, LineAttributes, PathAttributes, Point,
    RatingScore, Route, decode_base64_openlr, find_candidate_lines, find_candidate_nodes,
    resolve_routes,
};
use test_log::test;

use crate::graph::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};

#[test]
fn decode_line_location_reference_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = DecoderConfig {
        max_node_distance: Length::from_meters(10.0),
        ..Default::default()
    };

    let _ = decode_base64_openlr(&config, graph, "CwmShiVYczPJBgCs/y0zAQ==");
}

#[test]
fn find_candidate_nodes_001() {
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

    let nodes: Vec<_> = find_candidate_nodes(&config, graph, points.iter())
        .flat_map(|candidate| candidate.nodes)
        .map(|node| (node.vertex, (node.distance_to_lrp.meters() * 100.0).round()))
        .collect();

    assert_eq!(nodes, [(VertexId(68), 174.0), (VertexId(20), 216.0)]);
}

#[test]
fn find_candidate_nodes_002() {
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

    let nodes: Vec<_> = find_candidate_nodes(&config, graph, points.iter())
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
fn find_candidate_nodes_003() {
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

    let nodes: Vec<_> = find_candidate_nodes(&config, graph, [lrp].iter()).collect();
    assert_eq!(nodes, [CandidateNodes { lrp, nodes: vec![] }]);
}

#[test]
fn find_candidate_nodes_004() {
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
fn find_candidate_lines_001() {
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

    let lines: Vec<_> = find_candidate_lines(&config, graph, points.into_iter())
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
fn find_candidate_lines_002() {
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

    let lines = find_candidate_lines(&config, graph, points.into_iter()).unwrap();

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
fn find_candidate_lines_003() {
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

    let lines = find_candidate_lines(&config, graph, points.into_iter()).unwrap();

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
            vec![(EdgeId(7531947), None),]
        ]
    );
}

#[test]
fn resolve_routes_001() {
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
        [Route {
            edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            candidates: CandidateLinePair {
                line_lrp1: line1_first_lrp,
                line_lrp2: line_last_lrp
            }
        }]
    );
}

#[test]
fn resolve_routes_002() {
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
        [Route {
            edges: vec![EdgeId(8717174)],
            candidates: CandidateLinePair {
                line_lrp1: line_first_lrp,
                line_lrp2: line1_last_lrp
            }
        }]
    );
}

#[test]
fn resolve_routes_003() {
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
        [
            Route {
                edges: vec![], // first and second LRPs are on the same line
                candidates: CandidateLinePair {
                    line_lrp1: line_first_lrp,
                    line_lrp2: line_second_lrp
                }
            },
            Route {
                edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                candidates: CandidateLinePair {
                    line_lrp1: line_second_lrp,
                    line_lrp2: line_last_lrp
                }
            }
        ]
    );
}

#[test]
fn resolve_routes_004() {
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
        [
            Route {
                edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
                candidates: CandidateLinePair {
                    line_lrp1: line1_first_lrp,
                    line_lrp2: line1_second_lrp
                }
            },
            Route {
                edges: vec![EdgeId(6770340), EdgeId(7531947)],
                candidates: CandidateLinePair {
                    line_lrp1: line1_second_lrp,
                    line_lrp2: line_last_lrp
                }
            },
        ]
    );
}
