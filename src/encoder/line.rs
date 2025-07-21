use tracing::info;

use crate::{
    DirectedGraph, EncoderConfig, EncoderError, InvalidLocationError, LineLocation,
    is_path_connected,
};

pub fn encode_line<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: LineLocation<G::EdgeId>,
) -> Result<Vec<u8>, EncoderError> {
    info!("Encoding {line:?} with {config:?}");

    // Step – 1 Check validity of the location and offsets to be encoded.
    ensure_line_location_is_valid(config, graph, &line)?;

    todo!()
}

/// A line location is valid if the following constraints are fulfilled:
/// - The location is a connected path.
/// - The location is traversable from its start to its end.
///
/// The offsets must fulfill the following constraints:
/// - The sum of the positive and negative offset cannot be greater than the total length of the
///   location lines.
/// - Positive offset value shall be less than the length of the first line:
///     - Otherwise the first line can be removed from the list of location lines and the offset
///       value must be reduced in the same way.
///     - This procedure shall be repeated until this constraint is fulfilled.
/// - Negative offset value shall be less than the length of the last line:
///     - Otherwise the last line can be removed from the list of location lines and the offset
///       value must be reduced in the same way.
///     - This procedure shall be repeated until this constraint is fulfilled.
///
/// If it is intended to use the binary physical format this step should additionally calculate the
/// maximum (minimum) latitude values along the location and adjust the maximum distance between two
/// LR-points in Rule – 1, if necessary. The value defined in Rule – 1 is not applicable for
/// locations above the latitude value 65.70° (or below the latitude value -65.70°).
///
/// If the location is not valid the encoder should fail.
fn ensure_line_location_is_valid<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    line: &LineLocation<G::EdgeId>,
) -> Result<(), EncoderError> {
    let LineLocation {
        ref edges,
        pos_offset,
        neg_offset,
    } = *line;

    if edges.is_empty() {
        return Err(InvalidLocationError::Empty.into());
    } else if !is_path_connected(graph, edges) {
        return Err(InvalidLocationError::NotConnected.into());
    }

    if pos_offset > config.max_lrp_distance || neg_offset > config.max_lrp_distance {
        return Err(InvalidLocationError::InvalidOffsets((pos_offset, neg_offset)).into());
    }

    let path_length = edges.iter().filter_map(|&e| graph.get_edge_length(e)).sum();
    if pos_offset + neg_offset > path_length {
        return Err(InvalidLocationError::InvalidOffsets((pos_offset, neg_offset)).into());
    }

    Ok(())
}
