use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::fmt::Debug;

use tracing::debug;

use crate::path::Path;
use crate::{DirectedGraph, Frc, Length};

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
    graph: &G,
    origin: G::VertexId,
    destination: G::VertexId,
    lowest_frc: Frc,
    max_length: Length,
) -> Option<Path<G::EdgeId>> {
    debug!("Computing shortest path {origin:?} -> {destination:?}");

    // (current) shortest distance from origin to this vertex
    let mut shortest_distances = HashMap::from([(origin, Length::ZERO)]);

    // previous vertex (value) on the current best known path from origin to this vertex (key)
    let mut previous_map: HashMap<G::VertexId, (G::EdgeId, G::VertexId)> = HashMap::new();

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
            while let Some(&(edge, previous)) = previous_map.get(&next) {
                next = previous;
                edges.push(edge);
            }
            edges.reverse();

            return Some(Path {
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
            if distance > max_length {
                continue;
            }

            if graph.get_edge_frc(edge)? > lowest_frc {
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
                previous_map.insert(neighbor.vertex, (edge, element.vertex));
                frontier.push(neighbor);
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::graph::tests::{EdgeId, NETWORK_GRAPH, NetworkGraph, VertexId};

    #[test]
    fn decoder_shortest_path_001() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, VertexId(68), VertexId(68), Frc::Frc7, Length::MAX).unwrap(),
            Path {
                length: Length::ZERO,
                edges: vec![],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_002() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, VertexId(1), VertexId(2), Frc::Frc7, Length::MAX).unwrap(),
            Path {
                length: Length::from_meters(217.0),
                edges: vec![EdgeId(16218)],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_003() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, VertexId(2), VertexId(1), Frc::Frc7, Length::MAX),
            None
        );
    }

    #[test]
    fn decoder_shortest_path_004() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, VertexId(68), VertexId(20), Frc::Frc7, Length::MAX).unwrap(),
            Path {
                length: Length::from_meters(379.0),
                edges: vec![EdgeId(8717174), EdgeId(8717175), EdgeId(109783)],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_005() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, VertexId(1), VertexId(37), Frc::Frc7, Length::MAX).unwrap(),
            Path {
                length: Length::from_meters(753.0),
                edges: vec![
                    EdgeId(16218),
                    EdgeId(16219),
                    EdgeId(7430347),
                    EdgeId(4232179),
                    EdgeId(961826)
                ],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_006() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(
                graph,
                VertexId(1),
                VertexId(37),
                Frc::Frc7,
                Length::from_meters(752.0)
            ),
            None
        );
    }

    #[test]
    fn decoder_shortest_path_007() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, VertexId(36), VertexId(34), Frc::Frc7, Length::MAX).unwrap(),
            Path {
                length: Length::from_meters(16.0),
                edges: vec![EdgeId(-4232179)],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_008() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, VertexId(1), VertexId(57), Frc::Frc7, Length::MAX).unwrap(),
            Path {
                length: Length::from_meters(1462.0),
                edges: vec![
                    EdgeId(16218),
                    EdgeId(16219),
                    EdgeId(7430347),
                    EdgeId(961825),
                    EdgeId(7531950),
                    EdgeId(7531947),
                    EdgeId(7430351),
                    EdgeId(7430360),
                    EdgeId(7430361),
                    EdgeId(7430362),
                    EdgeId(7430348),
                    EdgeId(-244115),
                    EdgeId(-9497548),
                    EdgeId(-9497547),
                    EdgeId(3227046)
                ],
            }
        );
    }

    #[test]
    fn decoder_shortest_path_009() {
        let graph: &NetworkGraph = &NETWORK_GRAPH;

        assert_eq!(
            shortest_path(graph, VertexId(42), VertexId(68), Frc::Frc7, Length::MAX).unwrap(),
            Path {
                length: Length::from_meters(489.0),
                edges: vec![
                    EdgeId(1653344),
                    EdgeId(4997411),
                    EdgeId(5359424),
                    EdgeId(5359425),
                ],
            }
        );
    }
}
