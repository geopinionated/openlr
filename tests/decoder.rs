use openlr::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
use openlr::{DecoderConfig, Length, LineLocation, Location, decode_base64_openlr};
use test_log::test;

#[test]
fn decode_line_location_reference_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = DecoderConfig::default();
    let location = decode_base64_openlr(&config, graph, "CwmShiVYczPJBgCs/y0zAQ==").unwrap();

    assert_eq!(
        location,
        Location::Line(LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO
        })
    );
}

#[test]
fn decode_line_location_reference_002() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let config = DecoderConfig {
        max_node_distance: Length::from_meters(10.0),
        ..Default::default()
    };

    let location = decode_base64_openlr(&config, graph, "CwmTaSVYpTPZCP4a/5UjYQUH").unwrap();

    assert_eq!(
        location,
        Location::Line(LineLocation {
            path: vec![
                EdgeId(1653344),
                EdgeId(4997411),
                EdgeId(5359424),
                EdgeId(5359425)
            ],
            pos_offset: Length::from_meters(11.0),
            neg_offset: Length::from_meters(14.0)
        })
    );
}
