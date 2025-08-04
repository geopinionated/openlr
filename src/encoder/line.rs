use tracing::info;

use crate::encoder::expansion::line_location_expansion;
use crate::encoder::resolver::resolve_lrps;
use crate::{DirectedGraph, EncoderConfig, EncoderError, LineLocation, LocationReference};

pub fn encode_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: LineLocation<G::EdgeId>,
) -> Result<LocationReference, EncoderError> {
    info!("Encoding {line:?} with {config:?}");

    // Step – 1 Check validity of the location and offsets to be encoded
    let line = line.trim(graph)?;

    // Step – 2 Adjust start and end node of the location to represent valid map nodes
    let line = line_location_expansion(config, graph, &line);
    debug_assert!(!line.path.is_empty());

    // Step – 3..8 Split location into intermediate LRPs until full coverage.
    let lrps = resolve_lrps(config, graph, &line)?;
    debug_assert!(!lrps.is_empty());

    // Step – 9 Trim LRPs if the offset values exceeds the length of the corresponding path.
    let lrps = lrps.trim(config, graph)?;

    Ok(LocationReference::Line(lrps.into()))
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
    use crate::{DecoderConfig, Length, Location, decode_base64_openlr, encode_base64_openlr};

    #[test]
    fn encoder_encode_line_location_reference_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let line = Location::Line(LineLocation {
            path: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            pos_offset: Length::ZERO,
            neg_offset: Length::ZERO,
        });

        let encoded = encode_base64_openlr(&EncoderConfig::default(), graph, line.clone()).unwrap();
        let decoded = decode_base64_openlr(&DecoderConfig::default(), graph, &encoded).unwrap();
        assert_eq!(decoded, line);
    }

    #[test]
    fn encoder_encode_line_location_reference_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        let line = Location::Line(LineLocation {
            path: vec![
                EdgeId(1653344),
                EdgeId(4997411),
                EdgeId(5359424),
                EdgeId(5359425),
            ],
            pos_offset: Length::from_meters(11.0),
            neg_offset: Length::from_meters(14.0),
        });

        let encoded = encode_base64_openlr(&EncoderConfig::default(), graph, line.clone()).unwrap();
        let decoded = decode_base64_openlr(&DecoderConfig::default(), graph, &encoded).unwrap();
        assert_eq!(decoded, line);
    }
}
