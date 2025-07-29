use tracing::info;

use crate::encoder::expansion::line_location_expansion;
use crate::encoder::resolver::resolve_lrps;
use crate::{DirectedGraph, EncoderConfig, EncoderError, LineLocation, ensure_line_is_valid};

pub fn encode_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: LineLocation<G::EdgeId>,
) -> Result<Vec<u8>, EncoderError> {
    info!("Encoding {line:?} with {config:?}");

    // Step – 1 Check validity of the location and offsets to be encoded
    ensure_line_is_valid(graph, &line, config.max_lrp_distance)?;
    let line = line.trim(graph)?;

    // TODO: dedup line edges?

    // Step – 2 Adjust start and end node of the location to represent valid map nodes
    let expansion = line_location_expansion(config, graph, &line);

    // Step – 3,4,5,6 Split location into intermediate LRPs until full coverage.
    let lrps = resolve_lrps(config, graph, &line, &expansion)?;

    todo!()
}
