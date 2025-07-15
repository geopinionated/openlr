mod graph;

use openlr::decode_base64_openlr;

use crate::graph::{NETWORK_GRAPH, NetworkGraph};

#[test]
fn decode_line_location_reference_001() {
    let graph: &NetworkGraph = &NETWORK_GRAPH;

    let _ = decode_base64_openlr(graph, "CwmShiVYczPJBgCs/y0zAQ==");
}
