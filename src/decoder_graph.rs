use core::f64;
use std::collections::HashMap;

use geo::{
    BoundingRect, Contains, Distance, Haversine, HaversineClosestPoint, InterpolatableLine,
    closest_point,
};
use graph::prelude::{DirectedCsrGraph, DirectedNeighborsWithValues};
use rstar::RTree;

use crate::{Bearing, Coordinate, Fow, Frc, Graph, Length};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub i64);

impl VertexId {
    const fn index(&self) -> usize {
        self.0 as usize
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EdgeId(pub i64);

impl EdgeId {
    pub fn is_reversed(&self) -> bool {
        self.0.is_negative()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct EdgeProperty {
    pub id: EdgeId, // TODO: this should not be needed
    pub length: Length,
    pub frc: Frc,
    pub fow: Fow,
    pub geometry: Vec<Coordinate>, // TODO: Store directly LineString?
    pub vertices: [VertexId; 2],   // [start, end] vertices (need to be swapped if edge is reversed)
}

//#[derive(Debug, Default)]
pub struct NetworkGraph {
    // TODO: VertexID instead of usize?
    pub network: DirectedCsrGraph<usize, (), EdgeId>,
    pub geospatial_nodes: RTree<GeospatialNode>,
    pub geospatial_edges: RTree<GeospatialEdge>,
    pub edge_properties: HashMap<EdgeId, EdgeProperty>,
}

#[derive(Debug)]
pub struct GeospatialNode {
    pub vertex: VertexId,
    pub location: Coordinate,
}

impl rstar::RTreeObject for GeospatialNode {
    type Envelope = rstar::AABB<geo::Point>;

    fn envelope(&self) -> Self::Envelope {
        geo::Point::new(self.location.lon, self.location.lat).envelope()
    }
}

impl rstar::PointDistance for GeospatialNode {
    fn distance_2(&self, point: &geo::Point) -> f64 {
        Haversine
            .distance(
                geo::Point::new(self.location.lon, self.location.lat),
                *point,
            )
            .powf(2.0)
    }
}

#[derive(Debug)]
pub struct GeospatialEdge {
    pub edge: EdgeId,
    pub geometry: geo::LineString,
}

impl rstar::RTreeObject for GeospatialEdge {
    type Envelope = rstar::AABB<geo::Point>;

    fn envelope(&self) -> Self::Envelope {
        let bbox = self.geometry.bounding_rect().unwrap();
        rstar::AABB::from_corners(
            geo::Point::new(bbox.min().x, bbox.min().y),
            geo::Point::new(bbox.max().x, bbox.max().y),
        )
    }
}

impl rstar::PointDistance for GeospatialEdge {
    fn distance_2(&self, point: &geo::Point) -> f64 {
        match self.geometry.haversine_closest_point(point) {
            geo::Closest::SinglePoint(p) | geo::Closest::Intersection(p) => {
                Haversine.distance(p, *point).powf(2.0)
            }
            geo::Closest::Indeterminate => f64::INFINITY,
        }
    }
}

impl Graph for NetworkGraph {
    type EdgeId = EdgeId;
    type VertexId = VertexId;

    fn get_edge_start_vertex(&self, edge: Self::EdgeId) -> Option<Self::VertexId> {
        self.edge_properties
            .get(&EdgeId(edge.0.abs()))
            .map(|properties| {
                if edge.is_reversed() {
                    properties.vertices[1]
                } else {
                    properties.vertices[0]
                }
            })
    }

    fn get_edge_end_vertex(&self, edge: Self::EdgeId) -> Option<Self::VertexId> {
        self.edge_properties
            .get(&EdgeId(edge.0.abs()))
            .map(|properties| {
                if edge.is_reversed() {
                    properties.vertices[0]
                } else {
                    properties.vertices[1]
                }
            })
    }

    fn get_edge_length(&self, edge: Self::EdgeId) -> Option<Length> {
        self.edge_properties
            // edge properties do not change if the edge is reversed
            .get(&EdgeId(edge.0.abs()))
            .map(|properties| properties.length)
    }

    fn get_edge_frc(&self, edge: Self::EdgeId) -> Option<Frc> {
        self.edge_properties
            // edge properties do not change if the edge is reversed
            .get(&EdgeId(edge.0.abs()))
            .map(|properties| properties.frc)
    }

    fn get_edge_fow(&self, edge: Self::EdgeId) -> Option<Fow> {
        self.edge_properties
            // edge properties do not change if the edge is reversed
            .get(&EdgeId(edge.0.abs()))
            .map(|properties| properties.fow)
    }

    fn get_edge_coordinates(&self, edge: Self::EdgeId) -> impl Iterator<Item = Coordinate> {
        self.edge_properties
            // edge properties do not change if the edge is reversed
            .get(&EdgeId(edge.0.abs()))
            .into_iter()
            .flat_map(move |properties| {
                let mut geometry: Vec<_> = properties.geometry.clone();

                if edge.is_reversed() {
                    geometry.reverse();
                }

                geometry.into_iter()
            })
    }

    // TODO: bearing should be calculated from a start offset
    // OpenLR Java takes the next point at (BEAR_DIST=20m) after the first point
    fn get_edge_bearing(&self, edge: Self::EdgeId) -> Option<Bearing> {
        let coordinates: Vec<_> = self.get_edge_coordinates(edge).collect();

        let first = coordinates.first()?;
        let first = geo::point!(x: first.lon, y: first.lat);

        let last = coordinates.last()?;
        let last = geo::point!(x: last.lon, y: last.lat);

        use geo::Bearing;
        let bearing = Haversine.bearing(first, last);

        Some(crate::Bearing::from_degrees(bearing.round() as u16))
    }

    fn get_edge_bearing_between(
        &self,
        edge: Self::EdgeId,
        distance_start: Length,
        offset: Length,
    ) -> Option<Bearing> {
        let edge_length = self.get_edge_length(edge)?;
        if distance_start >= edge_length {
            return None;
        }

        let distance_end = (distance_start + offset).min(edge_length);

        let ratio_p1 = distance_start.meters() as f64 / edge_length.meters() as f64;
        let ratio_p2 = distance_end.meters() as f64 / edge_length.meters() as f64;

        let geometry = geo::LineString::from_iter(
            self.get_edge_coordinates(edge)
                .map(|coordinate| geo::coord! { x: coordinate.lon, y: coordinate.lat }),
        );

        let p1 = geometry.point_at_ratio_from_start(&Haversine, ratio_p1)?;
        let p2 = geometry.point_at_ratio_from_start(&Haversine, ratio_p2)?;

        use geo::Bearing;
        let bearing = Haversine.bearing(p1, p2);

        Some(crate::Bearing::from_degrees(bearing.round() as u16))
    }

    fn get_distance_from_start_vertex(
        &self,
        edge: Self::EdgeId,
        coordinate: Coordinate,
    ) -> Option<Length> {
        let geometry = geo::LineString::from_iter(
            self.get_edge_coordinates(edge)
                .map(|coordinate| geo::coord! { x: coordinate.lon, y: coordinate.lat }),
        );

        let mut closest_distance = f64::INFINITY;
        let mut distance_from_start = 0.0;
        let mut distance_acc = 0.0;

        let point = geo::Point::new(coordinate.lon, coordinate.lat);

        for line in geometry.lines() {
            match line.haversine_closest_point(&point) {
                geo::Closest::SinglePoint(p) | geo::Closest::Intersection(p) => {
                    let distance_to_line = Haversine.distance(point, p);

                    if distance_to_line < closest_distance {
                        // this is the closest line segment of the whole geometry (so far)
                        closest_distance = distance_to_line;
                        distance_from_start =
                            distance_acc + Haversine.distance(line.start_point(), p);
                    }

                    use geo::Length;
                    distance_acc += Haversine.length(&line);
                }
                geo::Closest::Indeterminate => return None,
            }
        }

        Some(Length::from_meters(distance_from_start.round() as u32))
    }

    fn vertex_exiting_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        self.network
            .out_neighbors_with_values(vertex.index())
            .map(|item| (item.value, VertexId(item.target as i64)))
    }

    fn vertex_entering_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        self.network
            .in_neighbors_with_values(vertex.index())
            .map(|item| (item.value, VertexId(item.target as i64)))
    }

    fn nearest_vertices_within_distance(
        &self,
        coordinate: crate::Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::VertexId, Length)> {
        let max_distance_2 = max_distance.meters() * max_distance.meters();
        let point = geo::Point::new(coordinate.lon, coordinate.lat);

        self.geospatial_nodes
            .nearest_neighbor_iter_with_distance_2(&point)
            .take_while(move |(_, distance_2)| *distance_2 <= max_distance_2 as f64)
            //.inspect(|(n, d)| println!("{:?}: {}m", n.vertex, d.sqrt()))
            .map(|(node, distance_2)| {
                let length = Length::from_meters(distance_2.sqrt().round() as u32);
                (node.vertex, length)
            })
    }

    fn nearest_edges_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::EdgeId, Length)> {
        let max_distance_2 = max_distance.meters() * max_distance.meters();
        let point = geo::Point::new(coordinate.lon, coordinate.lat);

        self.geospatial_edges
            .nearest_neighbor_iter_with_distance_2(&point)
            .take_while(move |(_, distance_2)| *distance_2 <= max_distance_2 as f64)
            //.inspect(|(n, d)| println!("{:?}: {}m", n.edge, d.sqrt()))
            .map(|(node, distance_2)| {
                let length = Length::from_meters(distance_2.sqrt().round() as u32);
                (node.edge, length)
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
        Self(value as i64)
    }
}

impl From<VertexId> for usize {
    fn from(vertex: VertexId) -> Self {
        vertex.index()
    }
}
