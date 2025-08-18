mod graph;

use openlr::{Length, LineLocation};
use test_log::test;

use crate::graph::{EdgeId, NETWORK_GRAPH, NetworkGraph};

#[test]
fn trim_line_location_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = LineLocation {
        path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
        pos_offset: Length::ZERO,
        neg_offset: Length::ZERO,
    };

    assert_eq!(location.clone().trim(graph), Ok(location));
}

#[test]
fn trim_line_location_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = LineLocation {
        path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
        pos_offset: Length::from_meters(10.0),
        neg_offset: Length::from_meters(10.0),
    };

    assert_eq!(location.clone().trim(graph), Ok(location));
}

#[test]
fn trim_line_location_003() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = LineLocation {
        path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
        pos_offset: Length::from_meters(136.0),
        neg_offset: Length::ZERO,
    };

    assert_eq!(
        location.trim(graph),
        Ok(LineLocation {
            path: vec![EdgeId(8717175), EdgeId(109783)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        })
    );
}

#[test]
fn trim_line_location_004() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = LineLocation {
        path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
        pos_offset: Length::from_meters(137.0),
        neg_offset: Length::ZERO,
    };

    assert_eq!(
        location.trim(graph),
        Ok(LineLocation {
            path: vec![EdgeId(8717175), EdgeId(109783)],
            pos_offset: Length::from_meters(1.0),
            neg_offset: Length::ZERO,
        })
    );
}

#[test]
fn trim_line_location_005() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let location = LineLocation {
        path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)], // 136m + 51m + 192m
        pos_offset: Length::from_meters(137.0),
        neg_offset: Length::from_meters(193.0),
    };

    assert_eq!(
        location.trim(graph),
        Ok(LineLocation {
            path: vec![EdgeId(8717175)],
            pos_offset: Length::from_meters(1.0),
            neg_offset: Length::from_meters(1.0),
        })
    );
}
