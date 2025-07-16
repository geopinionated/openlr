use tracing::debug;

use crate::{DecoderConfig, DirectedGraph, Length, Point};

/// List of candidate nodes for a Location Reference Point.
/// Nodes are sorted based on their distance to the point (closest to farthest).
#[derive(Debug, Clone, PartialEq)]
pub struct CandidateNodes<VertexId> {
    pub lrp: Point,
    pub nodes: Vec<CandidateNode<VertexId>>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CandidateNode<VertexId> {
    pub vertex: VertexId,
    pub distance_to_lrp: Length,
}

/// Each location reference point contains coordinates specifying a node in the encoder map. The
/// decoder should try to find so called candidate nodes in the decoder map whereby the coordinates
/// of the candidate nodes are close to the coordinates of the location reference point coordinates.
/// The straight line distance should be used to identify close-by nodes. Nodes in the decoder map
/// which are far away from the coordinates of the location reference point should not be considered
/// as candidate nodes in the further processing. It might happen that several candidate nodes for
/// one location reference point exist.
///
/// If no candidate node has been determined for a location reference point the decoder should try
/// to determine a candidate line directly. The LRP coordinate can be projected onto lines which are
/// not far away from that coordinate.
pub fn find_candidate_nodes<'a, G, I>(
    config: &DecoderConfig,
    graph: &G,
    points: I,
) -> impl Iterator<Item = CandidateNodes<G::VertexId>>
where
    G: DirectedGraph,
    I: IntoIterator<Item = &'a Point>,
{
    let DecoderConfig {
        max_node_distance, ..
    } = config;

    points.into_iter().map(move |&lrp| {
        debug!("Finding candidate nodes for {lrp:?} at max {max_node_distance}");

        let nodes: Vec<_> = graph
            .nearest_vertices_within_distance(lrp.coordinate, *max_node_distance)
            .map(|(vertex, distance_to_lrp)| CandidateNode {
                vertex,
                distance_to_lrp,
            })
            .collect();

        debug_assert!(nodes.is_sorted_by_key(|n| n.distance_to_lrp));
        CandidateNodes { lrp, nodes }
    })
}
