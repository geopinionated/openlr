use std::cmp::Ordering;
use std::collections::HashMap;
use std::hash::Hash;

use crate::Length;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HeapElement<EdgeId> {
    /// Current shortest distance from origin to this edge.
    pub distance: Length,
    /// The edge entering into the vertex, None for the origin.
    pub edge: EdgeId,
}

// The priority queue depends on the implementation of the Ord trait.
// By default std::BinaryHeap is a max heap.
// Explicitly implement the trait so the queue becomes a min heap.
impl<EdgeId: Ord> Ord for HeapElement<EdgeId> {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .distance
            .cmp(&self.distance)
            // breaking ties in a deterministic way
            .then_with(|| other.edge.cmp(&self.edge))
    }
}

impl<EdgeId: Ord> PartialOrd for HeapElement<EdgeId> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Unpacks the shortest path from destination back to origin.
pub fn unpack_path<EdgeId: Copy + Eq + Hash>(
    previous_edges: &HashMap<EdgeId, EdgeId>,
    destination: EdgeId,
) -> Vec<EdgeId> {
    let mut edges = vec![destination];
    let mut next = destination;

    while let Some(&e) = previous_edges.get(&next) {
        next = e;
        edges.push(e);
    }

    edges.reverse();
    edges
}
