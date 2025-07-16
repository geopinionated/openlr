mod graph;

use openlr::{
    Bearing, CandidateLine, CandidateLines, CandidateNode, CandidateNodes, Coordinate,
    DecoderConfig, Fow, Frc, Length, LineAttributes, PathAttributes, Point, RatingScore, Route,
    decode_base64_openlr, find_candidate_lines, find_candidate_nodes, resolve_routes,
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

    let candidate_lines = [
        CandidateLines {
            lrp: first_lrp,
            lines: vec![
                CandidateLine {
                    lrp: first_lrp,
                    edge: EdgeId(8717174),
                    rating: RatingScore::from(926.3),
                    distance_to_projection: None,
                },
                CandidateLine {
                    lrp: first_lrp,
                    edge: EdgeId(4925291),
                    rating: RatingScore::from(880.4),
                    distance_to_projection: Some(Length::from_meters(141.6)),
                },
            ],
        },
        CandidateLines {
            lrp: last_lrp,
            lines: vec![CandidateLine {
                lrp: last_lrp,
                edge: EdgeId(109783),
                rating: RatingScore::from(924.9),
                distance_to_projection: None,
            }],
        },
    ];

    let routes = resolve_routes(&config, graph, &candidate_lines).unwrap();

    assert_eq!(
        routes,
        [Route {
            lrp: first_lrp,
            length: Length::from_meters(379.0),
            edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)]
        }]
    );
}
