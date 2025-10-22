//! The OpenLR encoder generates a map-independent location reference for a (map-dependent)
//! location.

mod expansion;
mod line;
mod lrp;
mod resolver;
mod shortest_path;

use base64::Engine;
use base64::prelude::BASE64_STANDARD;

use crate::encoder::line::{encode_line, encode_point_along_line};
use crate::{
    DirectedGraph, EncodeError, Length, Location, LocationReference, serialize_binary_openlr,
};

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
    use LocationReference::*;
    let location = match location {
        Location::Line(line) => encode_line(config, graph, line).map(Line)?,
        Location::GeoCoordinate(coordinate) => GeoCoordinate(coordinate),
        Location::PointAlongLine(point) => {
            encode_point_along_line(config, graph, point).map(PointAlongLine)?
        }
    };

    // Step – 10 Create physical representation of the location reference.
    serialize_binary_openlr(&location).map_err(EncodeError::SerializeError)
}
