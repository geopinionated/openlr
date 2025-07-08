use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::vec;

use geo::{Distance, Haversine};
use graph::prelude::{DirectedCsrGraph, DirectedNeighborsWithValues};
use rstar::RTree;

use crate::graph::EdgeProperty;
use crate::{Coordinate, Graph, Length};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EdgeId(pub usize);

//#[derive(Debug, Default)]
pub struct NetworkGraph {
    // TODO: VertexID instead of usize? Store edge ID instead of property + Map<EdgeID, EdgeProperty>?
    pub network: DirectedCsrGraph<usize, (), EdgeProperty<EdgeId>>,
    pub geospatial_rtree: RTree<NetworkNode>,
    pub edge_properties: HashMap<EdgeId, EdgeProperty<EdgeId>>,
}

#[derive(Debug)]
pub struct NetworkNode {
    pub location: Coordinate,
    pub vertex: VertexId,
}

impl rstar::RTreeObject for NetworkNode {
    type Envelope = rstar::AABB<geo::Point>;

    fn envelope(&self) -> Self::Envelope {
        geo::Point::new(self.location.lon, self.location.lat).envelope()
    }
}

impl rstar::PointDistance for NetworkNode {
    fn distance_2(&self, point: &geo::Point) -> f64 {
        Haversine.distance(
            geo::Point::new(self.location.lon, self.location.lat),
            *point,
        )
    }
}

impl Graph for NetworkGraph {
    type EdgeId = EdgeId;
    type VertexId = VertexId;

    fn get_edge_property(&self, edge: Self::EdgeId) -> Option<&EdgeProperty<Self::EdgeId>> {
        self.edge_properties.get(&edge)
    }

    fn vertex_exiting_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        self.network
            .out_neighbors_with_values(vertex.0)
            .map(|item| (item.value.id, VertexId(item.target)))
    }

    fn vertex_entering_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        self.network
            .in_neighbors_with_values(vertex.0)
            .map(|item| (item.value.id, VertexId(item.target)))
    }

    fn nearest_vertices_within_distance(
        &self,
        coordinate: crate::Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = Self::VertexId> {
        let max_distance_2: f64 = (max_distance.meters() * max_distance.meters()) as f64;
        let point = geo::Point::new(coordinate.lon, coordinate.lat);

        self.geospatial_rtree
            .nearest_neighbor_iter_with_distance_2(&point)
            .take_while(move |(_, distance_2)| *distance_2 <= max_distance_2)
            .inspect(|(n, d)| println!("{}: {}m", n.vertex.0 + 1, d.sqrt()))
            .map(|(node, _)| node.vertex)
    }
}

#[derive(Debug, Clone, Default)]
pub struct Vertex {
    pub edges: Vec<Edge>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Edge {
    pub id: EdgeId,
    pub cost: Length,
    pub vertex_to: VertexId,
}

impl From<usize> for VertexId {
    fn from(value: usize) -> Self {
        Self(value)
    }
}

impl From<VertexId> for usize {
    fn from(vertex: VertexId) -> Self {
        vertex.0
    }
}

impl From<usize> for EdgeId {
    fn from(value: usize) -> Self {
        Self(value)
    }
}

impl From<EdgeId> for usize {
    fn from(edge: EdgeId) -> Self {
        edge.0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct State {
    cost: Length, // current best cost from origin to this vertex
    vertex: VertexId,
}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .cost
            .partial_cmp(&self.cost)
            .unwrap_or(Ordering::Equal)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, PartialEq, Eq)]
pub struct ShortestPath {
    cost: Length,
    path: Vec<VertexId>,
}

impl NetworkGraph {
    fn shortest_path(&self, origin: VertexId, destination: VertexId) -> Option<ShortestPath> {
        //if self.vertices.get(origin).is_none() || self.vertices.get(destination).is_none() {
        //    return None;
        //}

        // best_cost_from_origin[node]: represents the current known shortest distance from origin to node
        let mut best_cost_from_origin: HashMap<VertexId, Length> = HashMap::new();
        best_cost_from_origin.insert(origin, Length::from_meters(0));

        // prev_hop[node]: represents the previous-hop node on the current best known path from origin to node
        let mut prev_hop: HashMap<VertexId, VertexId> = HashMap::new();

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

            //let edges = &self.vertices[state.vertex].edges;
            let out_vertices: Vec<_> = self
                .network
                .out_neighbors_with_values(state.vertex.0)
                .map(|item| (VertexId(item.target), item.value))
                .collect();

            for (vertex_to, edge) in out_vertices {
                //let EdgeProperty { cost, frc, fow } = self.edge_properties.get(edge).unwrap();
                let cost_from_origin = state.cost + edge.cost;
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
}
