mod graph;

use openlr::{
    DirectedGraph, EncoderConfig, Length, LineLocation, Location, Path, edge_backward_expansion,
    edge_forward_expansion, encode_base64_openlr, is_node_valid, is_opposite_direction,
    select_edge_expansion_candidate,
};
use test_log::test;

use crate::graph::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};

#[ignore]
#[test]
fn encoder_encode_line_location_reference_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

    let location = Location::Line(LineLocation {
        path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    });

    let location = encode_base64_openlr(&config, graph, location).unwrap();
    assert_eq!(location, "CwmShiVYczPJBgCs/y0zAQ==");
}

#[ignore]
#[test]
fn encoder_encode_line_location_reference_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = EncoderConfig::default();

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

    let location = encode_base64_openlr(&config, graph, location).unwrap();
    assert_eq!(location, "CwmTaSVYpTPZCP4a/5UjYQUH");
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
