use tracing::info;

use crate::{DirectedGraph, EncoderConfig, EncoderError, LineLocation, ensure_line_is_valid};

pub fn encode_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: LineLocation<G::EdgeId>,
) -> Result<Vec<u8>, EncoderError> {
    info!("Encoding {line:?} with {config:?}");

    // Step – 1 Check validity of the location and offsets to be encoded.
    ensure_line_is_valid(graph, &line)?;
    let line = line.trim(graph)?;

    todo!()
}
