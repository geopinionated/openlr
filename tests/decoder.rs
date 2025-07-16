mod graph;

use openlr::{
    Bearing, Coordinate, DecoderConfig, Fow, Frc, Length, LineAttributes, PathAttributes, Point,
    decode_base64_openlr, find_candidate_nodes,
};

use crate::graph::{NETWORK_GRAPH, NetworkGraph, VertexId};

#[test]
fn decode_line_location_reference_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = DecoderConfig {
        max_node_distance: Length::from_meters(10.0),
    };

    let _ = decode_base64_openlr(&config, graph, "CwmShiVYczPJBgCs/y0zAQ==");
}

#[test]
fn find_candidate_nodes_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = DecoderConfig {
        max_node_distance: Length::from_meters(10.0),
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
fn find_candidate_nodes_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = DecoderConfig {
        max_node_distance: Length::from_meters(100.0),
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
