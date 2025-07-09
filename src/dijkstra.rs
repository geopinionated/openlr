use std::collections::{BinaryHeap, HashMap};
use std::fmt::Debug;

use crate::{Graph, Length};

#[derive(Debug, PartialEq, Eq)]
pub struct ShortestPath<N> {
    pub cost: Length,
    pub path: Vec<N>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct State<N> {
    cost: Length, // current best cost from origin to this vertex
    vertex: N,
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
    best_cost_from_origin.insert(origin, Length::from_meters(0));

    // prev_hop[node]: represents the previous-hop node on the current best known path from origin to node
    let mut prev_hop: HashMap<G::VertexId, G::VertexId> = HashMap::new();

    // The set of discovered nodes that may need to be visited. Initially, only the start node is known.
    let mut frontier = BinaryHeap::from([State {
        vertex: origin,
        cost: Length::from_meters(0),
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
                cost: state.cost,
                path: shortest_path,
            });
        }

        // check if we already know a cheaper way to get to the end of this path from the origin
        let best_origin_to_path_end_cost = *best_cost_from_origin
            .get(&state.vertex)
            .unwrap_or(&Length::MAX);
        if state.cost > best_origin_to_path_end_cost {
            continue;
        }

        for (edge, vertex_to) in graph.vertex_exiting_edges(state.vertex) {
            let edge = graph.get_edge_properties(edge)?;

            let cost_from_origin = state.cost + edge.length;
            let best_origin_to_neighbor_cost = *best_cost_from_origin
                .get(&vertex_to)
                .unwrap_or(&Length::MAX);

            // check if we can follow the current path to reach the neighbor in a cheaper way
            if cost_from_origin < best_origin_to_neighbor_cost {
                let neighbor = State {
                    vertex: vertex_to,
                    cost: cost_from_origin,
                };

                // Relaxation: we have now found a better way that we are going to explore
                best_cost_from_origin.insert(neighbor.vertex, neighbor.cost);
                prev_hop.insert(neighbor.vertex, state.vertex);
                frontier.push(neighbor);
            }
        }
    }

    None
}
