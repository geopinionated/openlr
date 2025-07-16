use base64::DecodeError;

use crate::{CandidateLines, DecoderConfig, DirectedGraph, Length, Point};

/// Shortest path from the LRP to the next one.
#[derive(Debug, Default)]
pub struct Route<EdgeId> {
    pub lrp: Point,
    pub length: Length,
    pub edges: Vec<EdgeId>,
}

/// The decoder needs to compute a shortest-path between each pair of subsequent location reference
/// points. For each pair of location reference points suitable candidate lines must be chosen. The
/// candidate line of the first LRPs of this pair acts as start of the shortest-path calculation.
/// The candidate line of the second location reference point of this pair is the end of the
/// shortest-path calculation. If the chosen lines are equal no shortest-path calculation needs to
/// be started.
///
/// The shortest path algorithm should take the part of the network into account which contains all
/// lines having a functional road class lower than or equal to the lowest functional road class of
/// the first location reference point of the pair. This value might be altered if the decoder
/// anticipates having different functional road class values than the encoder map.
///
/// Additionally the shortest-path algorithm should fulfill the following constraints:
/// - All lengths of the lines should be measured in meters and should also be converted to
///   integer values, float values need to be rounded correctly.
/// - The search is node based and will start at the start node of the first line and will end at
///   the end node of the last line.
/// - The algorithm shall return an ordered list of lines representing the calculated shortest-path.
///
/// If no shortest-path can be calculated for two subsequent location reference points, the decoder
/// might try a different pair of candidate lines or finally fail and report an error. If a
/// different pair of candidate lines is tried it might happen that the start line needs to be
/// changed. In such a case this also affects the end line of the previous shortest-path and this
/// path also needs to be re-calculated and checked again. The number of retries of shortest-path
/// calculations should be limited in order to guarantee a fast decoding process.
pub fn resolve_routes<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
) -> Result<Vec<Route<G::EdgeId>>, DecodeError> {
    let mut routes = Vec::with_capacity(candidate_lines.len() - 1);

    // TODO: check for single route

    for candidates_pair in candidate_lines.windows(2) {
        let [candidates_lrp1, candidates_lrp2] = [&candidates_pair[0], &candidates_pair[1]];

        // TODO: resolve candidate pairs order
        // TODO: for each pair calculate shortest path
    }

    Ok(routes)
}
