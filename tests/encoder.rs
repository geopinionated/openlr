mod graph;

use openlr::{EncoderConfig, Length, LineLocation, Location, encode_base64_openlr};
use test_log::test;

use crate::graph::{EdgeId, NETWORK_GRAPH, NetworkGraph};

#[ignore]
#[test]
fn encode_line_location_reference_001() {
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
fn encode_line_location_reference_002() {
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
