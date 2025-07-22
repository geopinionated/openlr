mod graph;

use openlr::{Length, Path, ShortestPathConfig, shortest_path};

use crate::graph::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};

#[test]
fn routing_shortest_path_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            &ShortestPathConfig::default(),
            graph,
            VertexId(68),
            VertexId(68),
        )
        .unwrap(),
        Path {
            length: Length::ZERO,
            edges: vec![],
        }
    );
}

#[test]
fn routing_shortest_path_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            &ShortestPathConfig::default(),
            graph,
            VertexId(1),
            VertexId(2)
        )
        .unwrap(),
        Path {
            length: Length::from_meters(217.0),
            edges: vec![EdgeId(16218)],
        }
    );
}

#[test]
fn routing_shortest_path_003() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            &ShortestPathConfig::default(),
            graph,
            VertexId(2),
            VertexId(1)
        ),
        None
    );
}

#[test]
fn routing_shortest_path_004() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            &ShortestPathConfig::default(),
            graph,
            VertexId(68),
            VertexId(20)
        )
        .unwrap(),
        Path {
            length: Length::from_meters(379.0),
            edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
        }
    );
}

#[test]
fn routing_shortest_path_005() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            &ShortestPathConfig::default(),
            graph,
            VertexId(1),
            VertexId(37)
        )
        .unwrap(),
        Path {
            length: Length::from_meters(753.0),
            edges: vec![
                EdgeId(16218),
                EdgeId(16219),
                EdgeId(7430347),
                EdgeId(4232179),
                EdgeId(961826)
            ],
        }
    );
}

#[test]
fn routing_shortest_path_006() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            &ShortestPathConfig {
                max_length: Length::from_meters(752.0),
                ..Default::default()
            },
            graph,
            VertexId(1),
            VertexId(37)
        ),
        None
    );
}

#[test]
fn routing_shortest_path_007() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            &ShortestPathConfig::default(),
            graph,
            VertexId(36),
            VertexId(34)
        )
        .unwrap(),
        Path {
            length: Length::from_meters(16.0),
            edges: vec![EdgeId(-4232179)],
        }
    );
}

#[test]
fn routing_shortest_path_008() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            &ShortestPathConfig::default(),
            graph,
            VertexId(1),
            VertexId(57)
        )
        .unwrap(),
        Path {
            length: Length::from_meters(1462.0),
            edges: vec![
                EdgeId(16218),
                EdgeId(16219),
                EdgeId(7430347),
                EdgeId(961825),
                EdgeId(7531950),
                EdgeId(7531947),
                EdgeId(7430351),
                EdgeId(7430360),
                EdgeId(7430361),
                EdgeId(7430362),
                EdgeId(7430348),
                EdgeId(-244115),
                EdgeId(-9497548),
                EdgeId(-9497547),
                EdgeId(3227046)
            ],
        }
    );
}
