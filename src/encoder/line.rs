use tracing::info;

use crate::{
    DirectedGraph, EncoderConfig, EncoderError, LineLocation, LocationReference,
    ensure_line_is_valid, line_location_expansion, resolve_lrps,
};

pub fn encode_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: LineLocation<G::EdgeId>,
) -> Result<LocationReference, EncoderError> {
    info!("Encoding {line:?} with {config:?}");

    // Step – 1 Check validity of the location and offsets to be encoded
    ensure_line_is_valid(graph, &line, config.max_lrp_distance)?;
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
