//! The OpenLR encoder generates a map-independent location reference for a (map-dependent)
//! location.
//!
//! 1. Check validity of the location and offsets to be encoded.
//! 2. Adjust start and end node of the location to represent valid map nodes.
//! 3. Determine coverage of the location by a shortest-path.
//! 4. Check whether the calculated shortest-path covers the location completely. Go to step 5 if
//!    the location is not covered completely, otherwise go to step 7.
//! 5. Determine the position of a new intermediate location reference point so that the part of the
//!    location between the start of the shortest-path calculation and the new intermediate is
//!    covered completely by a shortest-path.
//! 6. Go to step 3 and restart shortest path calculation between the new intermediate location
//!    reference point and the end of the location.
//! 7. Concatenate the calculated shortest-paths for a complete coverage of the location and form an
//!    ordered list of location reference points.
//! 8. Check validity of the location reference path. If the location reference path is invalid then
//!    go to step 9, if the location reference path is valid then go to step 10.
//! 9. Add a sufficient number of additional intermediate location reference points if the distance
//!    between two location reference points exceeds the maximum distance. Remove the start/ end
//!    LR-point if the positive/ negative offset value exceeds the length of the corresponding path.
//! 10. Create physical representation of the location reference.

mod expansion;
mod line;
mod lrp;
mod resolver;
mod shortest_path;

use base64::Engine;
use base64::prelude::BASE64_STANDARD;

use crate::encoder::line::encode_line;
use crate::{DirectedGraph, EncodeError, Length, Location, serialize_binary_openlr};

#[derive(Debug, Clone, Copy)]
pub struct EncoderConfig {
    /// The maximum distance allowed between consecutive LRPs.
    pub max_lrp_distance: Length,
    /// The length of the segment used to compute the lines bearing (distance from the start of
    /// the segment to its end).
    pub bearing_distance: Length,
}

impl Default for EncoderConfig {
    fn default() -> Self {
        // The smaller the max LRP distance the higher the offsets precision, however a small
        // distance can also negatively affect the decoding step, since having multiple LRPs
        // on the same line will incur in the same line degradation when rating any possible
        // route where 2 LRPs are on the same edge.
        const DEFAULT_MAX_LRP_DISTANCE: Length = Length::from_meters(4000.0);
        debug_assert!(DEFAULT_MAX_LRP_DISTANCE <= Length::MAX_BINARY_LRP_DISTANCE);

        Self {
            max_lrp_distance: DEFAULT_MAX_LRP_DISTANCE,
            bearing_distance: Length::from_meters(20.0),
        }
    }
}

/// Encodes an OpenLR Location Reference into Base64.
pub fn encode_base64_openlr<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    location: Location<G::EdgeId>,
) -> Result<String, EncodeError<G::Error>> {
    let data = encode_binary_openlr(config, graph, location)?;
    Ok(BASE64_STANDARD.encode(data))
}

/// Encodes an OpenLR Location Reference into binary.
pub fn encode_binary_openlr<G: DirectedGraph>(
    config: &EncoderConfig,
    graph: &G,
    location: Location<G::EdgeId>,
) -> Result<Vec<u8>, EncodeError<G::Error>> {
    let location = match location {
        Location::Line(line) => encode_line(config, graph, line)?,
    };

    // Step – 10 Create physical representation of the location reference.
    serialize_binary_openlr(&location).map_err(EncodeError::SerializeError)
}
