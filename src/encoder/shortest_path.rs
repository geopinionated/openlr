use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::fmt::Debug;
use std::hash::Hash;

use tracing::debug;

use crate::{DirectedGraph, EncoderError, Length, LocationError, Path, is_node_valid};

#[derive(Debug, Clone, PartialEq)]
pub enum ShortestRoute<EdgeId> {
    /// Route of the location that doesn't diverge from the shortest path, and therefore
    /// has no intermediate edges.
    Route(Path<EdgeId>),
    /// Route of the location converges to the shortest path up to the intermediate edge.
    Intermediate(IntermediateRoute<EdgeId>),
    /// Route not found.
    NotFound,
}

/// Lines which can be used as intermediates which are good lines to split the location into
/// several shortest-paths.
#[derive(Debug, Clone, PartialEq)]
pub struct IntermediateRoute<EdgeId> {
    /// Edges of the location that form a shortest path.
    pub edges: Vec<EdgeId>,
    /// The edge that represents the first intermediate, where the location can be split at its
    /// start to add a new LRP (since there is a deviation from the location path to the actual
    /// shortest path).
    pub intermediate: EdgeId,
    /// Index of the intermediate edge in the location edges list.
    pub intermediate_index: usize,
    /// The edge of the 2nd intermediate, Some when there is more than one deviation from the
    /// shortest path, otherwise None.
    pub intermediate_2: Option<EdgeId>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct HeapElement<EdgeId> {
    /// Current shortest distance from origin to this edge.
    distance: Length,
    /// The edge entering into the vertex, None for the origin.
    edge: EdgeId,
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

/// Returns the shortest route that follow the given location up until the location path diverges
/// from the shortest path, in which case the path route is split at an intermediate edge.
pub fn shortest_path_location<G: DirectedGraph>(
    graph: &G,
    location: &[G::EdgeId],
) -> Result<ShortestRoute<G::EdgeId>, EncoderError> {
    debug!("Computing shortest path following {location:?}");

    let (origin, destination) = match location.first().zip(location.last()) {
        Some((origin, destination)) => (*origin, *destination),
        _ => return Ok(ShortestRoute::NotFound),
    };

    if origin == destination && location.len() > 1 {
        // origin and destination are equals but there is a path in between
        // skip the origin and proceed with the next line in the location
        return Ok(ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![origin],
            intermediate: location[1],
            intermediate_index: 1,
            intermediate_2: None,
        }));
    }

    // Find the indices of eventual loops at the start and at the end of the location
    let mut postfix = location.get(1..).into_iter().flatten();
    let origin_loop_index = postfix.position(|&e| e == origin);

    let mut prefix = location.get(..location.len() - 1).into_iter().flatten();
    let destination_loop_index = prefix.position(|&e| e == destination);

    if let Some(0) = origin_loop_index {
        // origin loops onto itself
        return Ok(ShortestRoute::Intermediate(IntermediateRoute {
            edges: vec![origin],
            intermediate: location[1],
            intermediate_index: 1,
            intermediate_2: None,
        }));
    }

    let max_length = location
        .iter()
        .filter_map(|&e| graph.get_edge_length(e))
        .sum();

    let origin_length = graph.get_edge_length(origin).unwrap_or(Length::MAX);

    let mut shortest_distances = HashMap::from([(origin, origin_length)]);
    let mut previous_map: HashMap<G::EdgeId, G::EdgeId> = HashMap::new();
    let mut intermediator = Intermediator::new(graph, location)?;

    let mut frontier = BinaryHeap::from([HeapElement {
        distance: origin_length,
        edge: origin,
    }]);

    while let Some(element) = frontier.pop() {
        if location.contains(&element.edge) {
            // check if location the location needs to be split at an intermediate edge
            if let Some(route) = intermediator.get_intermediate_route(element, &previous_map)? {
                return Ok(ShortestRoute::Intermediate(route));
            }
        }

        if element.edge == destination {
            if let Some(loop_index) = destination_loop_index {
                // route found until the destination loop starts
                let last_edge = *previous_map.get(&destination).unwrap_or(&origin);

                return Ok(ShortestRoute::Intermediate(IntermediateRoute {
                    edges: unpack_path(&previous_map, last_edge),
                    intermediate: destination,
                    intermediate_index: loop_index,
                    intermediate_2: None,
                }));
            }

            return Ok(ShortestRoute::Route(Path {
                length: element.distance,
                edges: unpack_path(&previous_map, destination),
            }));
        }

        if let Some(loop_index) = origin_loop_index
            && intermediator.last_edge_index == loop_index
        {
            // the loop starting at origin has been completely followed
            return Ok(ShortestRoute::Intermediate(IntermediateRoute {
                edges: unpack_path(&previous_map, element.edge),
                intermediate: origin,
                intermediate_index: loop_index + 1,
                intermediate_2: None,
            }));
        }

        // check if we already know a cheaper way to get to the end of this path from the origin
        let shortest_distance = *shortest_distances
            .get(&element.edge)
            .unwrap_or(&Length::MAX);
        if element.distance > shortest_distance {
            continue;
        }

        let exiting_edges = graph
            .get_edge_end_vertex(element.edge)
            .into_iter()
            .flat_map(|v| graph.vertex_exiting_edges(v));

        for (edge, _) in exiting_edges {
            let edge_length = graph.get_edge_length(edge).unwrap_or(Length::MAX);
            let distance = element.distance + edge_length;

            if distance > max_length {
                continue;
            }

            let shortest_distance = *shortest_distances.get(&edge).unwrap_or(&Length::MAX);

            // check if we can follow the current path to reach the neighbor in a cheaper way
            if distance < shortest_distance {
                let neighbor = HeapElement { distance, edge };

                // Relax: we have now found a better way that we are going to explore
                shortest_distances.insert(neighbor.edge, neighbor.distance);
                previous_map.insert(neighbor.edge, element.edge);
                frontier.push(neighbor);
            }
        }
    }

    Ok(ShortestRoute::NotFound)
}

/// Breaks the location, if this doesn't follow the shortest path, at intermediate edges.
#[derive(Debug)]
struct Intermediator<'a, G: DirectedGraph> {
    graph: &'a G,
    location: &'a [G::EdgeId],
    last_edge: G::EdgeId,
    last_edge_index: usize,
}

impl<'a, G: DirectedGraph> Intermediator<'a, G> {
    fn new(graph: &'a G, location: &'a [G::EdgeId]) -> Result<Self, EncoderError> {
        let last_edge = location.first().copied().ok_or(LocationError::Empty)?;

        Ok(Self {
            graph,
            location,
            last_edge,
            last_edge_index: 0,
        })
    }

    /// Checks if the route should be split because it diverges from the shortest path.
    /// If it does, returns the intermediate route (up to the diversion), otherwise returns None.
    fn get_intermediate_route(
        &mut self,
        element: HeapElement<G::EdgeId>,
        previous_map: &HashMap<G::EdgeId, G::EdgeId>,
    ) -> Result<Option<IntermediateRoute<G::EdgeId>>, EncoderError> {
        if element.edge == self.location[0] {
            // first line is always found because all paths start from the origin
            return Ok(None);
        }

        if let Some(next_edge) = self.get_location_successor(previous_map, &element) {
            self.last_edge = next_edge;
            self.last_edge_index += 1;
            return Ok(None);
        }

        // The location deviates from the shortest path that would allow to reach this element.
        // Find the start of this deviation along the current path, at least the start line should
        // be found because all the paths go back to the origin.
        let common_edge = find_common_edge(self.location, previous_map, element.edge)
            .ok_or(EncoderError::IntermediateError(self.last_edge_index))?;

        // check if the deviation starts at the last element found so far or earlier in the path
        if common_edge == self.last_edge {
            let intermediate_index = self.last_edge_index + 1;
            let edges = unpack_path(previous_map, self.last_edge);

            let intermediate = *self
                .location
                .get(intermediate_index)
                .ok_or(EncoderError::IntermediateError(self.last_edge_index))?;

            let previous_intermediate = *previous_map
                .get(&intermediate)
                .ok_or(EncoderError::IntermediateError(self.last_edge_index))?;

            let intermediate_2 = if previous_intermediate != self.last_edge {
                // the shortest path to the intermediate (next location edge) does not include
                // the last (location) edge, therefore there is an additional (2nd) deviation
                self.rfind_valid_intermediate(previous_map)
            } else {
                None
            };

            let route = IntermediateRoute {
                edges,
                intermediate,
                intermediate_index,
                intermediate_2,
            };

            Ok(Some(route))
        } else {
            let intermediate = self
                .rfind_valid_intermediate(previous_map)
                .ok_or(EncoderError::IntermediateError(self.last_edge_index))?;

            let intermediate_index = self
                .location
                .iter()
                .position(|&e| e == intermediate)
                .ok_or(EncoderError::IntermediateError(self.last_edge_index))?;

            let previous_intermediate = *previous_map
                .get(&intermediate)
                .ok_or(EncoderError::IntermediateError(self.last_edge_index))?;

            let route = IntermediateRoute {
                edges: unpack_path(previous_map, previous_intermediate),
                intermediate,
                intermediate_index,
                intermediate_2: None,
            };

            Ok(Some(route))
        }
    }

    /// Returns the next edge only if the given element is a direct successor of the last element
    /// found in the location, and therefore the next (location) edge belongs to the shortest
    /// path. Otherwise returns None.
    fn get_location_successor(
        &mut self,
        previous_map: &HashMap<G::EdgeId, G::EdgeId>,
        element: &HeapElement<G::EdgeId>,
    ) -> Option<G::EdgeId> {
        let previous_element_edge = previous_map.get(&element.edge).copied()?;
        let element_edge = Some(element.edge);
        let next_edge = self.location.get(self.last_edge_index + 1).copied();

        if self.last_edge == previous_element_edge && next_edge == element_edge {
            next_edge
        } else {
            None
        }
    }

    /// Find an intermediate which has a valid start node.
    /// The method traverses the path from the last element found in the location back to the start
    /// and searches for a line having a valid start node.
    fn rfind_valid_intermediate(
        &self,
        previous_map: &HashMap<G::EdgeId, G::EdgeId>,
    ) -> Option<G::EdgeId> {
        debug_assert!(!self.location.is_empty());
        let mut edge = self.last_edge;

        loop {
            // Check if we came back to the previous LRP, this means that we had a cycle in our path
            // and cannot find a proper intersection to place the new intermediate.
            // We choose the last element found in location which is not placed at an intersection.
            if edge == self.location[0] {
                return Some(self.last_edge);
            } else if is_node_valid(self.graph, self.graph.get_edge_start_vertex(edge)?) {
                return Some(edge);
            }

            edge = previous_map.get(&edge).copied()?;
        }
    }
}

/// Returns the first element that is part of both the location and the provided given edge path.
fn find_common_edge<EdgeId: Copy + Eq + Hash>(
    location: &[EdgeId],
    previous_map: &HashMap<EdgeId, EdgeId>,
    mut edge: EdgeId,
) -> Option<EdgeId> {
    while let Some(&previous_edge) = previous_map.get(&edge) {
        if location.contains(&previous_edge) {
            return Some(previous_edge);
        }
        edge = previous_edge;
    }
    None
}

/// Unpacks the shortest path from destination back to origin.
fn unpack_path<EdgeId: Copy + Eq + Hash>(
    previous_map: &HashMap<EdgeId, EdgeId>,
    destination: EdgeId,
) -> Vec<EdgeId> {
    let mut edges = vec![destination];
    let mut next = destination;

    while let Some(&e) = previous_map.get(&next) {
        next = e;
        edges.push(e);
    }

    edges.reverse();
    edges
}
