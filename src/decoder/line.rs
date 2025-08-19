use tracing::info;

use crate::decoder::candidates::{find_candidate_lines, find_candidate_nodes};
use crate::decoder::resolver::resolve_routes;
use crate::{DecodeError, DecoderConfig, DirectedGraph, Line, LineLocation};

pub fn decode_line<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line: Line,
) -> Result<LineLocation<G::EdgeId>, DecodeError> {
    info!("Decoding {line:?} with {config:?}");

    // Step – 2 For each location reference point find candidate nodes
    let nodes = find_candidate_nodes(config, graph, &line.points);
    debug_assert_eq!(nodes.len(), line.points.len());

    // Step – 3 For each location reference point find candidate lines
    // Step – 4 Rate candidate lines for each location reference point
    let lines = find_candidate_lines(config, graph, nodes)?;
    debug_assert_eq!(lines.len(), line.points.len());

    // Step – 5 Determine shortest-path(s) between all subsequent location reference points
    // Step – 6 Check validity of the calculated shortest-path(s)
    let routes = resolve_routes(config, graph, &lines, line.offsets)?;
    debug_assert_eq!(routes.len(), line.points.len() - 1);

    // Step – 7 Concatenate and trim path according to the offsets
    let offsets = routes.calculate_offsets(graph, line.offsets);
    let (pos_offset, neg_offset) = offsets.unwrap_or_default();

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
