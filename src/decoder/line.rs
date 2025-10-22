//! 1. Decode physical data and check its validity.
//! 2. For each location reference point find candidate nodes.
//! 3. For each location reference point find candidate lines.
//! 4. Rate candidate lines for each location reference point.
//! 5. Determine shortest-path(s) between two subsequent location reference points.
//! 6. Check validity of the calculated shortest-path(s).
//! 7. Concatenate shortest-path(s) to form the location and trim path according to the offsets.

use tracing::debug;

use crate::decoder::candidates::{find_candidate_lines, find_candidate_nodes};
use crate::decoder::resolver::resolve_routes;
use crate::{DecodeError, DecoderConfig, DirectedGraph, Line, LineLocation};

pub fn decode_line<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line: Line,
) -> Result<LineLocation<G::EdgeId>, DecodeError<G::Error>> {
    debug!("Decoding {line:?} with {config:?}");

    // Step – 2 For each location reference point find candidate nodes
    let lrps_count = line.points.len();
    let nodes = find_candidate_nodes(config, graph, line.points)?;
    debug_assert_eq!(nodes.len(), lrps_count);

    // Step – 3 For each location reference point find candidate lines
    // Step – 4 Rate candidate lines for each location reference point
    let lines = find_candidate_lines(config, graph, nodes)?;
    debug_assert_eq!(lines.len(), lrps_count);

    // Step – 5 Determine shortest-path(s) between all subsequent location reference points
    // Step – 6 Check validity of the calculated shortest-path(s)
    let routes = resolve_routes(config, graph, &lines, line.offsets)?;
    debug_assert!(!routes.is_empty() && routes.len() < lrps_count);

    // Step – 7 Concatenate and trim path according to the offsets
    let (pos_offset, neg_offset) = routes.calculate_offsets(graph, line.offsets)?;

    let location = LineLocation {
        path: routes.to_path(),
        pos_offset,
        neg_offset,
    }
    .trim(graph)?;

    debug_assert!(!location.path.is_empty());
    debug_assert!(location.path.windows(2).all(|w| w[0] != w[1]));

    Ok(location)
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph};
    use crate::{DecoderConfig, Length, Location, decode_base64_openlr};

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

        let config = DecoderConfig::default();
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
}
