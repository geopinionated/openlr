use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::fmt::Debug;

use tracing::debug;

use crate::{DirectedGraph, Frc, Length};

#[derive(Debug, Clone, Copy)]
pub struct ShortestPathConfig {
    pub lowest_frc: Frc,
    pub max_length: Length,
}

impl Default for ShortestPathConfig {
    fn default() -> Self {
        Self {
            lowest_frc: Frc::Frc7,
            max_length: Length::MAX,
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct ShortestPath<EdgeId> {
    pub length: Length,
    pub edges: Vec<EdgeId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct HeapElement<VertexId> {
    /// Current shortest distance from origin to this vertex.
    distance: Length,
    vertex: VertexId,
}

// The priority queue depends on the implementation of the Ord trait.
// By default std::BinaryHeap is a max heap.
// Explicitly implement the trait so the queue becomes a min heap.
impl<VertexId: Ord> Ord for HeapElement<VertexId> {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .distance
            .cmp(&self.distance)
            // breaking ties in a deterministic way
            .then_with(|| other.vertex.cmp(&self.vertex))
    }
}

impl<VertexId: Ord> PartialOrd for HeapElement<VertexId> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

pub fn shortest_path<G: DirectedGraph>(
    config: &ShortestPathConfig,
    graph: &G,
    origin: G::VertexId,
    destination: G::VertexId,
) -> Option<ShortestPath<G::EdgeId>> {
    debug!("Computing shortest path {origin:?} -> {destination:?} with {config:?}");

    // (current) shortest distance from origin to this vertex
    let mut shortest_distances = HashMap::from([(origin, Length::ZERO)]);

    // previous vertex (value) on the current best known path from origin to this vertex (key)
    let mut prev_vertex: HashMap<G::VertexId, (G::EdgeId, G::VertexId)> = HashMap::new();

    // priority queue of discovered nodes that may need to be visited
    let mut frontier = BinaryHeap::from([HeapElement {
        vertex: origin,
        distance: Length::ZERO,
    }]);

    while let Some(element) = frontier.pop() {
        if element.vertex == destination {
            // Unpacking: the shortest path from destination back to origin
            let mut edges = vec![];
            let mut next = destination;
            while let Some(&(edge, previous)) = prev_vertex.get(&next) {
                next = previous;
                edges.push(edge);
            }
            edges.reverse();

            return Some(ShortestPath {
                length: element.distance,
                edges,
            });
        }

        // check if we already know a cheaper way to get to the end of this path from the origin
        let shortest_distance = *shortest_distances
            .get(&element.vertex)
            .unwrap_or(&Length::MAX);
        if element.distance > shortest_distance {
            continue;
        }

        for (edge, vertex_to) in graph.vertex_exiting_edges(element.vertex) {
            let distance = element.distance + graph.get_edge_length(edge)?;
            if distance > config.max_length {
                continue;
            }

            if graph.get_edge_frc(edge)? > config.lowest_frc {
                continue;
            }

            let shortest_distance = *shortest_distances.get(&vertex_to).unwrap_or(&Length::MAX);
            // check if we can follow the current path to reach the neighbor in a cheaper way
            if distance < shortest_distance {
                let neighbor = HeapElement {
                    vertex: vertex_to,
                    distance,
                };

                // Relax: we have now found a better way that we are going to explore
                shortest_distances.insert(neighbor.vertex, neighbor.distance);
                prev_vertex.insert(neighbor.vertex, (edge, element.vertex));
                frontier.push(neighbor);
            }
        }
    }

    None
}
