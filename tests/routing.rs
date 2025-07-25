mod graph;

use openlr::{
    Frc, IntermediateRoute, Length, Path, ShortestRoute, shortest_path, shortest_path_location,
};
use test_log::test;

use crate::graph::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};

#[test]
fn decoder_routing_shortest_path_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(graph, VertexId(68), VertexId(68), Frc::Frc7, Length::MAX).unwrap(),
        Path {
            length: Length::ZERO,
            edges: vec![],
        }
    );
}

#[test]
fn decoder_routing_shortest_path_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(graph, VertexId(1), VertexId(2), Frc::Frc7, Length::MAX).unwrap(),
        Path {
            length: Length::from_meters(217.0),
            edges: vec![EdgeId(16218)],
        }
    );
}

#[test]
fn decoder_routing_shortest_path_003() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(graph, VertexId(2), VertexId(1), Frc::Frc7, Length::MAX),
        None
    );
}

#[test]
fn decoder_routing_shortest_path_004() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(graph, VertexId(68), VertexId(20), Frc::Frc7, Length::MAX).unwrap(),
        Path {
            length: Length::from_meters(379.0),
            edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
        }
    );
}

#[test]
fn decoder_routing_shortest_path_005() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(graph, VertexId(1), VertexId(37), Frc::Frc7, Length::MAX).unwrap(),
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
fn decoder_routing_shortest_path_006() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(
            graph,
            VertexId(1),
            VertexId(37),
            Frc::Frc7,
            Length::from_meters(752.0)
        ),
        None
    );
}

#[test]
fn decoder_routing_shortest_path_007() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(graph, VertexId(36), VertexId(34), Frc::Frc7, Length::MAX).unwrap(),
        Path {
            length: Length::from_meters(16.0),
            edges: vec![EdgeId(-4232179)],
        }
    );
}

#[test]
fn decoder_routing_shortest_path_008() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    assert_eq!(
        shortest_path(graph, VertexId(1), VertexId(57), Frc::Frc7, Length::MAX).unwrap(),
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

#[test]
fn encoder_routing_shortest_path_location_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [EdgeId(-9044470), EdgeId(-9044471)];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Route(Path {
            length: Length::from_meters(31.0),
            edges: location.to_vec()
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [EdgeId(-9044470), EdgeId(-9044471), EdgeId(-9044472)];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Route(Path {
            length: Length::from_meters(45.0),
            edges: location.to_vec()
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_003() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [EdgeId(-9044472), EdgeId(4993083)];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Route(Path {
            length: Length::from_meters(219.0),
            edges: location.to_vec()
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_004() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(-7292030),
        EdgeId(-7292029),
        EdgeId(7516886),
        EdgeId(7516883),
        EdgeId(7516885),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![EdgeId(-7292030)],
            intermediate: EdgeId(-7292029),
            intermediate_index: 1,
            intermediate_2: None,
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_005() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(-4925290),
        EdgeId(-7292030),
        EdgeId(-7292029),
        EdgeId(7516886),
        EdgeId(7516883),
        EdgeId(7516885),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![EdgeId(-4925290), EdgeId(-7292030)],
            intermediate: EdgeId(-7292029),
            intermediate_index: 2,
            intermediate_2: None,
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_006() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [EdgeId(-7519159), EdgeId(5104156), EdgeId(-7519157)];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![EdgeId(-7519159)],
            intermediate: EdgeId(5104156),
            intermediate_index: 1,
            intermediate_2: None,
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_007() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(7531947),
        EdgeId(86727),
        EdgeId(4921654),
        EdgeId(-7144581),
        EdgeId(-7144582),
        EdgeId(-7144580),
        EdgeId(-7144579),
        EdgeId(-79715),
        EdgeId(7430361),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![EdgeId(7531947), EdgeId(86727)],
            intermediate: EdgeId(4921654),
            intermediate_index: 2,
            intermediate_2: None,
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_008() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(7516884),
        EdgeId(-7292029),
        EdgeId(7516886),
        EdgeId(-7516883),
        EdgeId(7516884),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![EdgeId(7516884)],
            intermediate: EdgeId(-7292029),
            intermediate_index: 1,
            intermediate_2: None,
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_009() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(7516884),
        EdgeId(7516884), // loop into first line
        EdgeId(7516885),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![EdgeId(7516884)],
            intermediate: EdgeId(7516884),
            intermediate_index: 1,
            intermediate_2: None,
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_010() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(7516884),
        EdgeId(-7292029), // getting here requires uturn into same (undirected) previous edge
        EdgeId(7516886),
        EdgeId(7516883),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![EdgeId(7516884)],
            intermediate: EdgeId(-7292029),
            intermediate_index: 1,
            intermediate_2: Some(EdgeId(7516884)),
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_011() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(-7516884),
        EdgeId(-7292029),
        EdgeId(7516886),
        EdgeId(7516883),
        EdgeId(-7516884), // loop into origin
        EdgeId(7292030),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![
                EdgeId(-7516884),
                EdgeId(-7292029),
                EdgeId(7516886),
                EdgeId(7516883)
            ],
            intermediate: EdgeId(-7516884),
            intermediate_index: 4,
            intermediate_2: None,
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_012() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(-7516885),
        EdgeId(-7516884), // loop into destination
        EdgeId(-7292029),
        EdgeId(7516886),
        EdgeId(7516883),
        EdgeId(-7516884),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![EdgeId(-7516885)],
            intermediate: EdgeId(-7516884),
            intermediate_index: 1,
            intermediate_2: None,
        })
    );
}

#[test]
fn encoder_routing_shortest_path_location_013() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = [
        EdgeId(961825),
        EdgeId(7531950),
        EdgeId(-6770340),
        EdgeId(7531949),
        EdgeId(7430352),
        EdgeId(7430353),
        EdgeId(-4232179),
        EdgeId(961825),
        EdgeId(7531950),
        EdgeId(-6770340),
        EdgeId(7531949),
        EdgeId(7430352),
        EdgeId(7430353),
        EdgeId(-4232179),
        EdgeId(4993081),
        EdgeId(4957478),
        EdgeId(4957479),
        EdgeId(4957480),
        EdgeId(-869555),
        EdgeId(-869554),
    ];

    let route = shortest_path_location(graph, &location).unwrap();

    assert_eq!(
        route,
        ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![
                EdgeId(961825),
                EdgeId(7531950),
                EdgeId(-6770340),
                EdgeId(7531949),
                EdgeId(7430352),
                EdgeId(7430353),
                EdgeId(-4232179)
            ],
            intermediate: EdgeId(961825),
            intermediate_index: 7,
            intermediate_2: None,
        })
    );
}
