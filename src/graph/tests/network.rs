use std::collections::{HashMap, HashSet};
use std::sync::LazyLock;

use geo::{
    BoundingRect, Closest, Distance, Haversine, HaversineClosestPoint, InterpolatableLine,
    LineString, Point, coord,
};
use graph::prelude::{DirectedCsrGraph, DirectedNeighborsWithValues};
use rstar::{AABB, PointDistance, RTree, RTreeObject};

use crate::graph::tests::geojson::{GEOJSON_GRAPH, GeojsonGraph};
use crate::{Bearing, Coordinate, DirectedGraph, Fow, Frc, Length};

pub static NETWORK_GRAPH: LazyLock<NetworkGraph> =
    LazyLock::new(|| NetworkGraph::from_geojson_graph(&GEOJSON_GRAPH));

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VertexId(pub u64);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct EdgeId(pub i64);

impl EdgeId {
    const fn is_reversed(&self) -> bool {
        self.0.is_negative()
    }

    const fn undirected(&self) -> Self {
        Self(self.0.abs())
    }
}

pub struct NetworkGraph {
    network: DirectedCsrGraph<u64, (), EdgeId>,
    geospatial_nodes: RTree<GeospatialNode>,
    geospatial_edges: RTree<GeospatialEdge>,
    edge_properties: HashMap<EdgeId, EdgeProperties>,
}

#[derive(Debug, Clone, PartialEq)]
struct EdgeProperties {
    length: Length,
    frc: Frc,
    fow: Fow,
    geometry: LineString,
    vertices: [VertexId; 2],
}

#[derive(Debug)]
struct GeospatialNode {
    vertex: VertexId,
    coordinate: Coordinate,
}

impl RTreeObject for GeospatialNode {
    type Envelope = AABB<Point>;
    fn envelope(&self) -> Self::Envelope {
        Point::new(self.coordinate.lon, self.coordinate.lat).envelope()
    }
}

impl PointDistance for GeospatialNode {
    fn distance_2(&self, destination: &Point) -> f64 {
        let origin = Point::new(self.coordinate.lon, self.coordinate.lat);
        Haversine.distance(origin, *destination).powf(2.0)
    }
}

#[derive(Debug)]
struct GeospatialEdge {
    edge: EdgeId,
    geometry: LineString,
}

impl RTreeObject for GeospatialEdge {
    type Envelope = AABB<Point>;
    fn envelope(&self) -> Self::Envelope {
        let bbox = self.geometry.bounding_rect().unwrap();
        AABB::from_corners(
            Point::new(bbox.min().x, bbox.min().y),
            Point::new(bbox.max().x, bbox.max().y),
        )
    }
}

impl PointDistance for GeospatialEdge {
    fn distance_2(&self, point: &Point) -> f64 {
        use Closest::*;
        match self.geometry.haversine_closest_point(point) {
            SinglePoint(p) | Intersection(p) => Haversine.distance(p, *point).powf(2.0),
            Indeterminate => f64::INFINITY,
        }
    }
}

impl DirectedGraph for NetworkGraph {
    type EdgeId = EdgeId;
    type VertexId = VertexId;

    fn get_vertex_coordinate(&self, vertex: Self::VertexId) -> Option<Coordinate> {
        self.vertex_exiting_edges(vertex).next().and_then(|(e, _)| {
            debug_assert_eq!(self.get_edge_start_vertex(e), Some(vertex));
            self.get_edge_coordinates(e).next()
        })
    }

    fn get_edge_start_vertex(&self, edge: Self::EdgeId) -> Option<Self::VertexId> {
        self.edge_properties
            .get(&edge.undirected())
            .map(|EdgeProperties { vertices, .. }| {
                if edge.is_reversed() {
                    vertices[1]
                } else {
                    vertices[0]
                }
            })
    }

    fn get_edge_end_vertex(&self, edge: Self::EdgeId) -> Option<Self::VertexId> {
        self.edge_properties
            .get(&edge.undirected())
            .map(|EdgeProperties { vertices, .. }| {
                if edge.is_reversed() {
                    vertices[0]
                } else {
                    vertices[1]
                }
            })
    }

    fn get_edge_length(&self, edge: Self::EdgeId) -> Option<Length> {
        self.edge_properties
            .get(&edge.undirected())
            .map(|EdgeProperties { length, .. }| *length)
    }

    fn get_edge_frc(&self, edge: Self::EdgeId) -> Option<Frc> {
        self.edge_properties
            .get(&edge.undirected())
            .map(|EdgeProperties { frc, .. }| *frc)
    }

    fn get_edge_fow(&self, edge: Self::EdgeId) -> Option<Fow> {
        self.edge_properties
            .get(&edge.undirected())
            .map(|EdgeProperties { fow, .. }| *fow)
    }

    fn get_edge_coordinates(&self, edge: Self::EdgeId) -> impl Iterator<Item = Coordinate> {
        self.edge_properties
            .get(&edge.undirected())
            .into_iter()
            .flat_map(move |EdgeProperties { geometry, .. }| {
                let geometry = geometry.coords().map(|coordinate| Coordinate {
                    lon: coordinate.x,
                    lat: coordinate.y,
                });

                let geometry: Box<dyn Iterator<Item = Coordinate>> = if edge.is_reversed() {
                    Box::new(geometry.into_iter().rev())
                } else {
                    Box::new(geometry.into_iter())
                };

                geometry
            })
    }

    fn vertex_exiting_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        let mut edges: Vec<_> = self
            .network
            .out_neighbors_with_values(vertex.0)
            .map(|item| (item.value, VertexId(item.target)))
            .collect();

        // edges returned in a deterministic order
        edges.sort();
        edges.into_iter()
    }

    fn vertex_entering_edges(
        &self,
        vertex: Self::VertexId,
    ) -> impl Iterator<Item = (Self::EdgeId, Self::VertexId)> {
        let mut edges: Vec<_> = self
            .network
            .in_neighbors_with_values(vertex.0)
            .map(|item| (item.value, VertexId(item.target)))
            .collect();

        // edges returned in a deterministic order
        edges.sort();
        edges.into_iter()
    }

    fn nearest_vertices_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::VertexId, Length)> {
        let max_distance_2 = max_distance.meters() * max_distance.meters();
        let point = Point::new(coordinate.lon, coordinate.lat);

        self.geospatial_nodes
            .nearest_neighbor_iter_with_distance_2(&point)
            .take_while(move |(_, distance_2)| *distance_2 <= max_distance_2)
            .map(|(node, distance_2)| {
                let length = Length::from_meters(distance_2.sqrt());
                (node.vertex, length)
            })
    }

    fn nearest_edges_within_distance(
        &self,
        coordinate: Coordinate,
        max_distance: Length,
    ) -> impl Iterator<Item = (Self::EdgeId, Length)> {
        let max_distance_2 = max_distance.meters() * max_distance.meters();
        let point = Point::new(coordinate.lon, coordinate.lat);

        self.geospatial_edges
            .nearest_neighbor_iter_with_distance_2(&point)
            .take_while(move |(_, distance_2)| *distance_2 <= max_distance_2)
            .map(|(node, distance_2)| {
                let length = Length::from_meters(distance_2.sqrt());
                (node.edge, length)
            })
    }

    fn get_distance_along_edge(
        &self,
        edge: Self::EdgeId,
        coordinate: Coordinate,
    ) -> Option<Length> {
        let mut closest_distance = f64::INFINITY;
        let mut distance_along_edge = 0.0;
        let mut distance_acc = 0.0;

        let point = Point::new(coordinate.lon, coordinate.lat);

        for line in self.edge_line_string(edge).lines() {
            match line.haversine_closest_point(&point) {
                Closest::SinglePoint(p) | Closest::Intersection(p) => {
                    let distance_to_line = Haversine.distance(point, p);

                    if distance_to_line < closest_distance {
                        // this is the closest line segment of the whole geometry (so far)
                        closest_distance = distance_to_line;
                        let distance = Haversine.distance(line.start_point(), p);
                        distance_along_edge = distance_acc + distance;
                    }

                    use geo::Length;
                    distance_acc += Haversine.length(&line);
                }
                Closest::Indeterminate => return None,
            }
        }

        Some(Length::from_meters(distance_along_edge).min(self.get_edge_length(edge)?))
    }

    fn get_coordinate_along_edge(
        &self,
        edge: Self::EdgeId,
        distance: Length,
    ) -> Option<Coordinate> {
        let ratio = distance.meters() / self.get_edge_length(edge)?.meters();

        let geometry = self.edge_line_string(edge);
        let point = geometry.point_at_ratio_from_start(&Haversine, ratio)?;

        Some(Coordinate {
            lon: point.x(),
            lat: point.y(),
        })
    }

    fn get_edge_bearing(
        &self,
        edge: Self::EdgeId,
        distance_from_start: Length,
        segment_length: Length,
    ) -> Option<Bearing> {
        let edge_length = self.get_edge_length(edge)?;
        let distance_start = distance_from_start.clamp(Length::ZERO, edge_length);
        let distance_end = (distance_start + segment_length).clamp(Length::ZERO, edge_length);

        let c1 = self.get_coordinate_along_edge(edge, distance_start)?;
        let p1 = Point::new(c1.lon, c1.lat);

        let c2 = self.get_coordinate_along_edge(edge, distance_end)?;
        let p2 = Point::new(c2.lon, c2.lat);

        let degrees = {
            use geo::Bearing;
            Haversine.bearing(p1, p2).round() as u16
        };

        Some(Bearing::from_degrees(degrees))
    }

    fn is_turn_restricted(&self, _start: Self::EdgeId, _end: Self::EdgeId) -> bool {
        false
    }
}

impl NetworkGraph {
    fn edge_line_string(&self, edge: EdgeId) -> LineString {
        LineString::from_iter(
            self.get_edge_coordinates(edge)
                .map(|coordinate| coord! { x: coordinate.lon, y: coordinate.lat }),
        )
    }

    fn from_geojson_graph(graph: &GeojsonGraph) -> NetworkGraph {
        let edge_properties = graph
            .lines
            .iter()
            .map(|(&line_id, line)| {
                let property = EdgeProperties {
                    length: line.length,
                    frc: line.frc,
                    fow: line.fow,
                    geometry: line.geometry.clone(),
                    vertices: [VertexId(line.start_node_id), VertexId(line.end_node_id)],
                };

                (EdgeId(line_id), property)
            })
            .collect();

        let network_edges = graph.nodes.iter().flat_map(|(&from_id, node)| {
            node.exiting_lines
                .iter()
                .map(move |&(line_id, to_id)| (from_id, to_id, EdgeId(line_id)))
        });

        let geospatial_nodes: Vec<GeospatialNode> = graph
            .nodes
            .iter()
            .map(|(&from_id, node)| GeospatialNode {
                vertex: VertexId(from_id),
                coordinate: node.coordinate,
            })
            .collect();

        let directed_edges: HashSet<EdgeId> = graph
            .nodes
            .iter()
            .flat_map(|(_, node)| {
                node.exiting_lines
                    .iter()
                    .map(|&(line_id, _)| EdgeId(line_id))
            })
            .collect();

        let geospatial_edges: Vec<GeospatialEdge> = directed_edges
            .into_iter()
            .map(|edge_id| {
                let line = graph.lines.get(&edge_id.undirected().0).unwrap();
                GeospatialEdge {
                    edge: edge_id,
                    geometry: line.geometry.clone(),
                }
            })
            .collect();

        NetworkGraph {
            network: graph::prelude::GraphBuilder::new()
                .edges_with_values(network_edges)
                .build(),
            geospatial_nodes: RTree::bulk_load(geospatial_nodes),
            geospatial_edges: RTree::bulk_load(geospatial_edges),
            edge_properties,
        }
    }
}

#[test]
fn network_graph_coordinate_along_edge() {
    let graph = &NETWORK_GRAPH;

    assert_eq!(
        graph
            .get_coordinate_along_edge(EdgeId(16218), Length::ZERO)
            .unwrap(),
        Coordinate {
            lon: 13.454214,
            lat: 52.5157088
        }
    );

    assert_eq!(
        graph
            .get_coordinate_along_edge(EdgeId(16218), graph.get_edge_length(EdgeId(16218)).unwrap())
            .unwrap(),
        Coordinate {
            lon: 13.457386,
            lat: 52.5153814
        }
    );

    assert_eq!(
        graph
            .get_coordinate_along_edge(EdgeId(16218), Length::from_meters(10.0))
            .unwrap(),
        Coordinate {
            lon: 13.454360,
            lat: 52.515693
        }
    );

    assert_eq!(
        graph
            .get_coordinate_along_edge(EdgeId(16218), Length::from_meters(100.0))
            .unwrap(),
        Coordinate {
            lon: 13.455676,
            lat: 52.515561
        }
    );

    assert_eq!(
        graph
            .get_coordinate_along_edge(EdgeId(16218), Length::from_meters(-1.0))
            .unwrap(),
        Coordinate {
            lon: 13.454214,
            lat: 52.5157088
        }
    );

    assert_eq!(
        graph
            .get_coordinate_along_edge(
                EdgeId(16218),
                graph.get_edge_length(EdgeId(16218)).unwrap() + Length::from_meters(1.0)
            )
            .unwrap(),
        Coordinate {
            lon: 13.457386,
            lat: 52.5153814
        }
    );
}

#[test]
fn network_graph_vertex_coordinate() {
    let graph = &NETWORK_GRAPH;

    assert_eq!(
        graph.get_vertex_coordinate(VertexId(1)).unwrap(),
        Coordinate {
            lon: 13.454214,
            lat: 52.5157088
        }
    );

    assert_eq!(
        graph.get_vertex_coordinate(VertexId(2)).unwrap(),
        Coordinate {
            lon: 13.457386,
            lat: 52.5153814
        }
    );

    assert_eq!(
        graph.get_vertex_coordinate(VertexId(58)).unwrap(),
        Coordinate {
            lon: 13.4572516,
            lat: 52.5149212
        }
    );

    assert_eq!(
        graph.get_vertex_coordinate(VertexId(105)).unwrap(),
        Coordinate {
            lon: 13.4551048,
            lat: 52.5152531
        }
    );

    assert_eq!(
        graph.get_vertex_coordinate(VertexId(134)).unwrap(),
        Coordinate {
            lon: 13.4587361,
            lat: 52.516543
        }
    );
}

#[test]
fn network_graph_vertex_degree() {
    let graph = &NETWORK_GRAPH;

    assert_eq!(graph.vertex_degree(VertexId(1)), 1);
    assert_eq!(graph.vertex_degree(VertexId(2)), 4);
    assert_eq!(graph.vertex_degree(VertexId(58)), 6);
    assert_eq!(graph.vertex_degree(VertexId(105)), 2);
    assert_eq!(graph.vertex_degree(VertexId(68)), 8);
    assert_eq!(graph.vertex_degree(VertexId(134)), 4);
    assert_eq!(graph.vertex_degree(VertexId(77)), 3);
}

#[test]
fn network_graph_edge_bearing_between() {
    let graph = &NETWORK_GRAPH;

    assert_eq!(
        graph
            .get_edge_bearing(EdgeId(109783), Length::ZERO, Length::from_meters(20.0))
            .unwrap(),
        Bearing::from_degrees(200)
    );

    assert_eq!(
        graph
            .get_edge_bearing(EdgeId(-109783), Length::ZERO, Length::from_meters(20.0))
            .unwrap(),
        Bearing::from_degrees(18)
    );

    assert_eq!(
        graph
            .get_edge_bearing(
                EdgeId(5359425),
                graph.get_edge_length(EdgeId(5359425)).unwrap() - Length::from_meters(1.0),
                Length::from_meters(20.0)
            )
            .unwrap(),
        Bearing::from_degrees(197)
    );

    assert_eq!(
        graph
            .get_edge_bearing(EdgeId(-5359425), Length::ZERO, Length::from_meters(20.0))
            .unwrap(),
        Bearing::from_degrees(17)
    );

    assert_eq!(
        graph
            .get_edge_bearing(EdgeId(5104156), Length::ZERO, Length::from_meters(10.0))
            .unwrap(),
        Bearing::from_degrees(139)
    );

    assert_eq!(
        graph
            .get_edge_bearing(
                EdgeId(5104156),
                Length::from_meters(15.0),
                Length::from_meters(5.0)
            )
            .unwrap(),
        Bearing::from_degrees(97)
    );

    assert_eq!(
        graph
            .get_edge_bearing(
                EdgeId(109783),
                graph.get_edge_length(EdgeId(109783)).unwrap(),
                Length::from_meters(-20.0)
            )
            .unwrap(),
        Bearing::from_degrees(18)
    );
}

#[test]
fn network_graph_edge_bearing() {
    let graph = &NETWORK_GRAPH;

    let get_edge_bearing = |edge| {
        graph
            .get_edge_bearing(edge, Length::ZERO, graph.get_edge_length(edge).unwrap())
            .unwrap()
    };

    assert_eq!(
        get_edge_bearing(EdgeId(-5359425)),
        Bearing::from_degrees(17)
    );
    assert_eq!(
        get_edge_bearing(EdgeId(5359425)),
        Bearing::from_degrees(17 + 180)
    );

    assert_eq!(
        get_edge_bearing(EdgeId(8717174)),
        Bearing::from_degrees(106)
    );

    assert_eq!(
        get_edge_bearing(EdgeId(5359426)),
        Bearing::from_degrees(197)
    );

    assert_eq!(
        get_edge_bearing(EdgeId(-4925291)),
        Bearing::from_degrees(286)
    );

    assert_eq!(
        get_edge_bearing(EdgeId(7531947)),
        Bearing::from_degrees(100)
    );

    assert_eq!(
        get_edge_bearing(EdgeId(6770340)),
        Bearing::from_degrees(192)
    );

    assert_eq!(
        get_edge_bearing(EdgeId(-6770340)),
        Bearing::from_degrees(12)
    );
}

#[test]
fn network_graph_distance_along_edge() {
    let graph = &NETWORK_GRAPH;

    assert_eq!(
        graph
            .get_distance_along_edge(
                EdgeId(6770340),
                Coordinate {
                    lon: 13.462836552352906,
                    lat: 52.51499534095764,
                }
            )
            .unwrap()
            .round(),
        Length::from_meters(0.0)
    );

    assert_eq!(
        graph
            .get_distance_along_edge(
                EdgeId(-109783),
                Coordinate {
                    lon: 13.462836552352906,
                    lat: 52.51499534095764,
                }
            )
            .unwrap()
            .round(),
        Length::from_meters(1.0)
    );

    assert_eq!(
        graph
            .get_distance_along_edge(
                EdgeId(109783),
                Coordinate {
                    lon: 13.462836552352906,
                    lat: 52.51499534095764,
                }
            )
            .unwrap()
            .round(),
        Length::from_meters(191.0)
    );

    assert_eq!(
        graph
            .get_distance_along_edge(
                EdgeId(4925291),
                Coordinate {
                    lon: 13.461116552352905,
                    lat: 52.51710534095764,
                }
            )
            .unwrap()
            .round(),
        Length::from_meters(142.0)
    );

    assert_eq!(
        graph
            .get_distance_along_edge(
                EdgeId(-4925291),
                Coordinate {
                    lon: 13.461116552352905,
                    lat: 52.51710534095764,
                }
            )
            .unwrap()
            .round(),
        Length::from_meters(1.0)
    );

    assert_eq!(
        graph
            .get_distance_along_edge(
                EdgeId(8717174),
                Coordinate {
                    lon: 13.461951,
                    lat: 52.51700,
                }
            )
            .unwrap()
            .round(),
        Length::from_meters(57.0)
    );

    assert_eq!(
        graph
            .get_distance_along_edge(
                EdgeId(-8717174),
                Coordinate {
                    lon: 13.461951,
                    lat: 52.51700,
                }
            )
            .unwrap()
            .round(),
        Length::from_meters(80.0)
    );
}

#[test]
fn network_graph_edge_vertices() {
    let graph = &NETWORK_GRAPH;

    assert_eq!(
        graph.get_edge_start_vertex(EdgeId(-6770340)).unwrap(),
        VertexId(102)
    );

    assert_eq!(
        graph.vertex_exiting_edges(VertexId(1)).collect::<Vec<_>>(),
        vec![(EdgeId(16218), VertexId(2))]
    );
    assert_eq!(
        graph.get_edge_start_vertex(EdgeId(16218)).unwrap(),
        VertexId(1)
    );
    assert_eq!(
        graph.get_edge_end_vertex(EdgeId(16218)).unwrap(),
        VertexId(2)
    );

    assert_eq!(
        graph.vertex_exiting_edges(VertexId(68)).collect::<Vec<_>>(),
        [
            (EdgeId(-5359425), VertexId(12)),
            (EdgeId(-4925291), VertexId(67)),
            (EdgeId(5359426), VertexId(60)),
            (EdgeId(8717174), VertexId(95))
        ]
    );

    assert_eq!(
        graph.get_edge_start_vertex(EdgeId(-5359425)).unwrap(),
        VertexId(68)
    );
    assert_eq!(
        graph.get_edge_end_vertex(EdgeId(-5359425)).unwrap(),
        VertexId(12)
    );
    assert_eq!(
        graph.get_edge_start_vertex(EdgeId(-5359425)).unwrap(),
        graph.get_edge_end_vertex(EdgeId(5359425)).unwrap()
    );
    assert_eq!(
        graph.get_edge_end_vertex(EdgeId(-5359425)).unwrap(),
        graph.get_edge_start_vertex(EdgeId(5359425)).unwrap()
    );
}

#[test]
fn network_graph_edge_coordinates() {
    let graph = &NETWORK_GRAPH;

    assert_eq!(
        graph
            .get_edge_coordinates(EdgeId(5359425))
            .collect::<Vec<_>>(),
        [
            Coordinate {
                lon: 13.4615934,
                lat: 52.5180374
            },
            Coordinate {
                lon: 13.4611206,
                lat: 52.5170944
            }
        ]
    );

    assert_eq!(
        graph
            .get_edge_coordinates(EdgeId(8717174))
            .collect::<Vec<_>>(),
        [
            Coordinate {
                lon: 13.4611206,
                lat: 52.5170944
            },
            Coordinate {
                lon: 13.4630579,
                lat: 52.5167465
            }
        ]
    );

    assert_eq!(
        graph
            .get_edge_coordinates(EdgeId(-8717174))
            .collect::<Vec<_>>(),
        [
            Coordinate {
                lon: 13.4630579,
                lat: 52.5167465
            },
            Coordinate {
                lon: 13.4611206,
                lat: 52.5170944
            },
        ]
    );

    assert_eq!(
        graph
            .get_edge_coordinates(EdgeId(7531948))
            .collect::<Vec<_>>(),
        [
            Coordinate {
                lon: 13.463911,
                lat: 52.5148731
            },
            Coordinate {
                lon: 13.4631576,
                lat: 52.5149491
            },
            Coordinate {
                lon: 13.4628442,
                lat: 52.5149807
            }
        ]
    );
}

#[test]
fn network_graph_nearest_edges() {
    let graph = &NETWORK_GRAPH;

    let coordinate = Coordinate {
        lon: 13.461951,
        lat: 52.51700,
    };

    const MAX_DISTANCE: Length = Length::from_meters(100.0);

    let edges: Vec<_> = graph
        .nearest_edges_within_distance(coordinate, MAX_DISTANCE)
        .map(|(edge, distance)| {
            assert!(distance <= MAX_DISTANCE);
            (edge, distance)
        })
        .collect();
    assert!(edges.is_sorted_by_key(|(_, d)| *d));

    // lines with both directions have the same distance, sort by edge ID to make it deterministic
    let mut edges = edges.into_iter().map(|(e, _)| e).collect::<Vec<_>>();
    edges.sort_unstable_by_key(|e| e.0);

    assert_eq!(
        edges,
        [
            EdgeId(-8717175),
            EdgeId(-8717174),
            EdgeId(-5707439),
            EdgeId(-5707436),
            EdgeId(-5707435),
            EdgeId(-5359426),
            EdgeId(-5359425),
            EdgeId(-4925291),
            EdgeId(4925291),
            EdgeId(5359425),
            EdgeId(5359426),
            EdgeId(5707435),
            EdgeId(5707436),
            EdgeId(5707439),
            EdgeId(8717174),
            EdgeId(8717175)
        ]
    );
}

#[test]
fn network_graph_nearest_vertices() {
    let graph = &NETWORK_GRAPH;

    let node_75_location = Coordinate {
        lon: 13.459407,
        lat: 52.5143601,
    };

    const MAX_DISTANCE: Length = Length::from_meters(90.0);

    let neighbours: Vec<VertexId> = graph
        .nearest_vertices_within_distance(node_75_location, MAX_DISTANCE)
        .map(|(vertex, distance)| {
            assert!(distance <= MAX_DISTANCE);
            vertex
        })
        .collect();

    assert_eq!(
        neighbours,
        [
            VertexId(75),
            VertexId(140),
            VertexId(138),
            VertexId(139),
            VertexId(59),
            VertexId(72),
            VertexId(73),
            VertexId(74)
        ]
    );
}

#[test]
fn network_graph_edge_properties() {
    let graph = &NETWORK_GRAPH;

    let get_exiting_edge_properties = |vertex| {
        graph
            .vertex_exiting_edges(vertex)
            .map(|(edge, vertex_to)| {
                (
                    edge,
                    graph.get_edge_length(edge).unwrap(),
                    graph.get_edge_frc(edge).unwrap(),
                    graph.get_edge_fow(edge).unwrap(),
                    vertex_to,
                )
            })
            .collect::<Vec<_>>()
    };

    let get_entering_edge_properties = |vertex| {
        graph
            .vertex_entering_edges(vertex)
            .map(|(edge, vertex_to)| {
                (
                    edge,
                    graph.get_edge_length(edge).unwrap(),
                    graph.get_edge_frc(edge).unwrap(),
                    graph.get_edge_fow(edge).unwrap(),
                    vertex_to,
                )
            })
            .collect::<Vec<_>>()
    };

    assert_eq!(graph.vertex_entering_edges(VertexId(1)).count(), 0);
    assert_eq!(
        get_exiting_edge_properties(VertexId(1)),
        vec![(
            EdgeId(16218),
            Length::from_meters(217.0),
            Frc::Frc2,
            Fow::SingleCarriageway,
            VertexId(2)
        )]
    );

    assert_eq!(
        get_entering_edge_properties(VertexId(126)),
        vec![
            (
                EdgeId(-8323953),
                Length::from_meters(16.0),
                Frc::Frc6,
                Fow::SingleCarriageway,
                VertexId(127)
            ),
            (
                EdgeId(8323959),
                Length::from_meters(11.0),
                Frc::Frc6,
                Fow::SingleCarriageway,
                VertexId(129)
            )
        ]
    );
    assert_eq!(
        get_exiting_edge_properties(VertexId(126)),
        vec![
            (
                EdgeId(-8323959),
                Length::from_meters(11.0),
                Frc::Frc6,
                Fow::SingleCarriageway,
                VertexId(129)
            ),
            (
                EdgeId(8323953),
                Length::from_meters(16.0),
                Frc::Frc6,
                Fow::SingleCarriageway,
                VertexId(127)
            )
        ]
    );

    assert_eq!(
        get_entering_edge_properties(VertexId(134)),
        vec![
            (
                EdgeId(-8345026),
                Length::from_meters(31.0),
                Frc::Frc6,
                Fow::SingleCarriageway,
                VertexId(123)
            ),
            (
                EdgeId(8345025),
                Length::from_meters(199.0),
                Frc::Frc6,
                Fow::SingleCarriageway,
                VertexId(134)
            )
        ]
    );
    assert_eq!(
        get_exiting_edge_properties(VertexId(134)),
        vec![
            (
                EdgeId(8345025),
                Length::from_meters(199.0),
                Frc::Frc6,
                Fow::SingleCarriageway,
                VertexId(134)
            ),
            (
                EdgeId(8345026),
                Length::from_meters(31.0),
                Frc::Frc6,
                Fow::SingleCarriageway,
                VertexId(123)
            )
        ]
    );
}
