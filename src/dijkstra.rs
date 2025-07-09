use std::collections::{BinaryHeap, HashMap};
use std::fmt::Debug;

use crate::{Graph, Length};

#[derive(Debug, PartialEq, Eq)]
pub struct ShortestPath<VertexId> {
    pub distance: Length,
    pub path: Vec<VertexId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct State<VertexId, Distance> {
    distance: Distance, // current best cost from origin to this vertex
    vertex: VertexId,
}

fn shortest_path<G>(
    graph: &G,
    origin: G::VertexId,
    destination: G::VertexId,
) -> Option<ShortestPath<G::VertexId>>
where
    G: Graph,
{
    //if self.vertices.get(origin).is_none() || self.vertices.get(destination).is_none() {
    //    return None;
    //}

    // best_cost_from_origin[node]: represents the current known shortest distance from origin to node
    let mut best_cost_from_origin: HashMap<G::VertexId, Length> = HashMap::new();
    best_cost_from_origin.insert(origin, Length::MIN);

    // prev_hop[node]: represents the previous-hop node on the current best known path from origin to node
    let mut prev_hop: HashMap<G::VertexId, G::VertexId> = HashMap::new();

    // The set of discovered nodes that may need to be visited. Initially, only the start node is known.
    let mut frontier = BinaryHeap::from([State {
        vertex: origin,
        distance: Length::MIN,
    }]);

    while let Some(state) = frontier.pop() {
        if state.vertex == destination {
            // Unpacking: the best path from target back to origin
            let mut shortest_path = vec![destination];
            let mut next = &destination;
            while let Some(node) = prev_hop.get(next) {
                next = node;
                shortest_path.push(*node);
                if node == &origin {
                    break;
                }
            }
            shortest_path.reverse();

            return Some(ShortestPath {
                distance: state.distance,
                path: shortest_path,
            });
        }

        // check if we already know a cheaper way to get to the end of this path from the origin
        let best_origin_to_path_end_cost = *best_cost_from_origin
            .get(&state.vertex)
            .unwrap_or(&Length::MAX);
        if state.distance > best_origin_to_path_end_cost {
            continue;
        }

        for (edge, vertex_to) in graph.vertex_exiting_edges(state.vertex) {
            let edge_length = graph.get_edge_length(edge)?;

            let cost_from_origin = state.distance + edge_length;

            let best_origin_to_neighbor_cost = *best_cost_from_origin
                .get(&vertex_to)
                .unwrap_or(&Length::MAX);

            // check if we can follow the current path to reach the neighbor in a cheaper way
            if cost_from_origin < best_origin_to_neighbor_cost {
                let neighbor = State {
                    vertex: vertex_to,
                    distance: cost_from_origin,
                };

                // Relaxation: we have now found a better way that we are going to explore
                best_cost_from_origin.insert(neighbor.vertex, neighbor.distance);
                prev_hop.insert(neighbor.vertex, state.vertex);
                frontier.push(neighbor);
            }
        }
    }

    None
}
