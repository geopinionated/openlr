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

use base64::Engine;
use base64::prelude::BASE64_STANDARD;

use crate::error::DecodeError;
use crate::{DeserializeError, DirectedGraph, Location, deserialize_binary_openlr};

/// Decodes an OpenLR Location Reference encoded in Base64.
pub fn decode_base64_openlr<G: DirectedGraph>(
    graph: &G,
    data: impl AsRef<[u8]>,
) -> Result<Location, DecodeError> {
    let data = BASE64_STANDARD
        .decode(data)
        .map_err(DeserializeError::from)?;
    decode_binary_openlr(graph, &data)
}

/// Decodes an OpenLR Location Reference encoded in binary.
pub fn decode_binary_openlr<G: DirectedGraph>(
    _graph: &G,
    data: &[u8],
) -> Result<Location, DecodeError> {
    // Step – 1 Decode physical data and check its validity
    let location = deserialize_binary_openlr(data)?;

    Err(DecodeError::LocationTypeNotSupported(
        location.location_type(),
    ))
}
