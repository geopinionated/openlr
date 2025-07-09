use std::collections::HashMap;

use geo::{Distance, Haversine, LineString};
use graph::prelude::{DirectedCsrGraph, DirectedNeighborsWithValues};
use rstar::RTree;

use crate::{Coordinate, Fow, Frc, Graph, Length};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EdgeId(pub usize);

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeProperty {
    pub id: EdgeId, // TODO: this should not be needed
    pub length: Length,
    pub frc: Frc,
    pub fow: Fow,
    pub geometry: Vec<Coordinate>,
}

//#[derive(Debug, Default)]
pub struct NetworkGraph {
    // TODO: VertexID instead of usize?
    pub network: DirectedCsrGraph<usize, (), EdgeId>,
    pub geospatial_rtree: RTree<NetworkNode>,
    pub edge_properties: HashMap<EdgeId, EdgeProperty>,
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

    fn get_edge_length(&self, edge: Self::EdgeId) -> Option<Length> {
        self.edge_properties
            .get(&edge)
            .map(|properties| properties.length)
    }
    fn get_edge_frc(&self, edge: Self::EdgeId) -> Option<Frc> {
        self.edge_properties
            .get(&edge)
            .map(|properties| properties.frc)
    }

    fn get_edge_fow(&self, edge: Self::EdgeId) -> Option<Fow> {
        self.edge_properties
            .get(&edge)
            .map(|properties| properties.fow)
    }

    fn get_edge_coordinates(&self, edge: Self::EdgeId) -> impl Iterator<Item = Coordinate> {
        self.edge_properties
            .get(&edge)
            .into_iter()
            .flat_map(|properties| properties.geometry.iter().copied())
    }

    fn vertex_exiting_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        self.network
            .out_neighbors_with_values(vertex.0)
            .map(|item| (item.value, VertexId(item.target)))
    }

    fn vertex_entering_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        self.network
            .in_neighbors_with_values(vertex.0)
            .map(|item| (item.value, VertexId(item.target)))
    }

    fn nearest_vertices_within_distance(
        &self,
        coordinate: crate::Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::VertexId, Length)> {
        let max_distance_2 = max_distance.meters() * max_distance.meters();
        let point = geo::Point::new(coordinate.lon, coordinate.lat);

        self.geospatial_rtree
            .nearest_neighbor_iter_with_distance_2(&point)
            .take_while(move |(_, distance_2)| *distance_2 <= max_distance_2 as f64)
            .inspect(|(n, d)| println!("{}: {}m", n.vertex.0 + 1, d.sqrt()))
            .map(|(node, distance_2)| {
                let length = Length::from_meters(distance_2.sqrt().round() as u32);
                (node.vertex, length)
            })
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
