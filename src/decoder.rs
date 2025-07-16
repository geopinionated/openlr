//! The decoder resolves a (map-dependent) location using its own map.
//! This map might differ from the one used during encoding.
//!
//! 1. Decode physical data and check its validity.
//! 2. For each location reference point find candidate nodes.
//! 3. For each location reference point find candidate lines.
//! 4. Rate candidate lines for each location reference point.
//! 5. Determine shortest-path(s) between two subsequent location reference points.
//! 6. Check validity of the calculated shortest-path(s).
//! 7. Concatenate shortest-path(s) to form the location and trim path according to the offsets.

pub mod candidates;

use base64::Engine;
use base64::prelude::BASE64_STANDARD;
use candidates::find_candidate_nodes;
use tracing::info;

use crate::error::DecodeError;
use crate::model::LineLocation;
use crate::{
    DeserializeError, DirectedGraph, Length, Line, Location, LocationReference,
    deserialize_binary_openlr,
};

#[derive(Debug, Clone, Copy)]
pub struct DecoderConfig {
    /// Maximum distance from the LRP to the nodes of the graph that will be considered.
    pub max_node_distance: Length,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            max_node_distance: Length::from_meters(100.0),
        }
    }
}

/// Decodes an OpenLR Location Reference encoded in Base64.
pub fn decode_base64_openlr<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    data: impl AsRef<[u8]>,
) -> Result<Location, DecodeError> {
    let data = BASE64_STANDARD
        .decode(data)
        .map_err(DeserializeError::from)?;
    decode_binary_openlr(config, graph, &data)
}

/// Decodes an OpenLR Location Reference encoded in binary.
pub fn decode_binary_openlr<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    data: &[u8],
) -> Result<Location, DecodeError> {
    // Step – 1 Decode physical data and check its validity
    let location = deserialize_binary_openlr(data)?;

    match location {
        LocationReference::Line(line) => decode_line(config, graph, line).map(Location::Line),
        _ => Err(DecodeError::LocationTypeNotSupported(
            location.location_type(),
        )),
    }
}

fn decode_line<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line: Line,
) -> Result<LineLocation, DecodeError> {
    info!("Decoding {line:?} with {config:?}");

    // Step – 2 For each location reference point find candidate nodes
    let nodes = find_candidate_nodes(config, graph, &line.points);

    todo!()
}
