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
pub mod resolver;

use base64::Engine;
use base64::prelude::BASE64_STANDARD;
use candidates::find_candidate_nodes;
use tracing::info;

use crate::error::DecodeError;
use crate::model::{LineLocation, RatingScore};
use crate::{
    Bearing, DeserializeError, DirectedGraph, Length, Line, Location, LocationReference,
    deserialize_binary_openlr, find_candidate_lines, resolve_routes,
};

#[derive(Debug, Clone, Copy)]
pub struct DecoderConfig {
    /// Maximum distance from the LRP to the nodes of the graph that will be considered.
    pub max_node_distance: Length,
    /// The length of the segment used to compute the lines bearing (distance from the start of
    /// the segment to its end).
    pub bearing_distance: Length,
    /// Maximum bearing difference between the candidate line bearing and the LRP bearing for the
    /// candidate to be accepted.
    pub max_bearing_difference: Bearing,
    /// Node weight applied by the rating function.
    pub node_factor: f64,
    /// Line weight applied by the rating function.
    pub line_factor: f64,
    /// Projected line weight applied by the rating function.
    pub projected_line_factor: f64,
    /// Minimum rating score for a line to be accepted as candidate.
    pub min_line_rating: RatingScore,
    /// Maximum number of resolver retries.
    pub max_number_retries: usize,
    /// Variance allowed to the resolver when computing distance between LRPs.
    pub next_point_variance: Length,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            max_node_distance: Length::from_meters(100.0),
            bearing_distance: Length::from_meters(20.0),
            max_bearing_difference: Bearing::from_degrees(90),
            node_factor: 3.0,
            line_factor: 3.0,
            projected_line_factor: 0.95,
            min_line_rating: RatingScore::from(800.0),
            max_number_retries: 3,
            next_point_variance: Length::from_meters(150.0),
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
    let nodes = find_candidate_nodes(config, graph, line.points.iter());

    // Step – 3 For each location reference point find candidate lines
    // Step – 4 Rate candidate lines for each location reference point
    let lines = find_candidate_lines(config, graph, nodes)?;

    // Step – 5 Determine shortest-path(s) between all subsequent location reference points
    // Step – 6 Check validity of the calculated shortest-path(s)
    let routes = resolve_routes(config, graph, &lines)?;

    todo!()
}
