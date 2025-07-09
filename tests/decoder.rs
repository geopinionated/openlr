use std::collections::HashMap;
use std::str::FromStr;

use geojson::{Feature, FeatureCollection, Value};
use graph::prelude::DirectedNeighborsWithValues;
use openlr::decoder_graph::{EdgeId, NetworkGraph, NetworkNode, VertexId};
use openlr::{Coordinate, EdgeProperty, Fow, Frc, Graph, Length, decode_base64_openlr};
use rstar::RTree;
use strum::IntoEnumIterator;

type NodeId = i64;
type LineId = i64;

#[derive(Debug)]
struct Node {
    id: NodeId,
    location: Coordinate,
    lines: Vec<(LineId, NodeId)>, // outgoing
}

#[derive(Debug)]
struct Line {
    id: LineId,
    // start and end have meaning only if direction != both ways
    start_id: NodeId,
    end_id: NodeId,
    length: u32,
    fow: Fow,
    frc: Frc,
    geometry: Vec<Coordinate>,
    direction: u8,
}

#[derive(Debug, Default)]
struct GeojsonGraph {
    nodes: HashMap<NodeId, Node>,
    lines: HashMap<LineId, Line>,
}

#[test]
fn decode_line_location_reference() {
    let geojson = include_str!("data/graph.geojson");
    let geojson_graph = GeojsonGraph::parse_geojson(geojson);
    let graph: NetworkGraph = geojson_graph.into_network_graph();

    let location = decode_base64_openlr(&graph, "CwmShiVYczPJBgCs/y0zAQ==").unwrap();
}

#[test]
fn geojson_graph_connected_vertices() {
    let geojson = include_str!("data/graph.geojson");
    let geojson_graph = GeojsonGraph::parse_geojson(geojson);
    let graph: NetworkGraph = geojson_graph.into_network_graph();

    let connected: Vec<VertexId> = graph.connected_vertices(VertexId(68)).collect();
    assert_eq!(
        connected,
        vec![VertexId(12), VertexId(60), VertexId(67), VertexId(95)]
    );

    let connected: Vec<VertexId> = graph.connected_vertices(VertexId(125)).collect();
    assert_eq!(connected, vec![VertexId(122)]);

    let connected: Vec<VertexId> = graph.connected_vertices(VertexId(134)).collect();
    assert_eq!(connected, vec![VertexId(123)]);

    let connected: Vec<VertexId> = graph.connected_vertices(VertexId(116)).collect();
    assert_eq!(connected, vec![VertexId(4), VertexId(114), VertexId(135)]);
}

#[test]
fn geojson_graph_nearest_neighbours() {
    let geojson = include_str!("data/graph.geojson");
    let geojson_graph = GeojsonGraph::parse_geojson(geojson);

    let graph: NetworkGraph = geojson_graph.into_network_graph();

    let node_75_location = Coordinate {
        lon: 13.459407,
        lat: 52.5143601,
    };

    const MAX_DISTANCE: Length = Length::from_meters(9);

    let neighbours: Vec<VertexId> = graph
        .nearest_vertices_within_distance(node_75_location, MAX_DISTANCE)
        .map(|(vertex, distance)| {
            assert!(distance.meters() <= MAX_DISTANCE.meters());
            vertex
        })
        .collect();
    println!("{neighbours:?}");

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
fn graph_into_directed() {
    let geojson = include_str!("data/graph.geojson");
    let geojson_graph = GeojsonGraph::parse_geojson(geojson);
    let graph: NetworkGraph = geojson_graph.into_network_graph();

    let get_exiting_edges = |vertex| {
        let mut edges = graph
            .vertex_exiting_edges(vertex)
            .map(|(edge, vertex_to)| (graph.get_edge_properties(edge).cloned().unwrap(), vertex_to))
            .collect::<Vec<_>>();
        edges.sort_unstable_by_key(|(_, v)| *v);
        edges
    };

    let get_entering_edges = |vertex| {
        let mut edges = graph
            .vertex_entering_edges(vertex)
            .map(|(edge, vertex_to)| (graph.get_edge_properties(edge).cloned().unwrap(), vertex_to))
            .collect::<Vec<_>>();
        edges.sort_unstable_by_key(|(_, v)| *v);
        edges
    };

    assert_eq!(graph.vertex_entering_edges(VertexId(1)).count(), 0);
    assert_eq!(
        get_exiting_edges(VertexId(1)),
        vec![(
            EdgeProperty {
                id: EdgeId(16218),
                length: Length::from_meters(217),
                frc: Frc::Frc2,
                fow: Fow::SingleCarriageway,
            },
            VertexId(2)
        )]
    );

    assert_eq!(
        get_entering_edges(VertexId(126)),
        vec![
            (
                EdgeProperty {
                    id: EdgeId(8323953),
                    length: Length::from_meters(16),
                    frc: Frc::Frc6,
                    fow: Fow::SingleCarriageway,
                },
                VertexId(127)
            ),
            (
                EdgeProperty {
                    id: EdgeId(8323959),
                    length: Length::from_meters(11),
                    frc: Frc::Frc6,
                    fow: Fow::SingleCarriageway,
                },
                VertexId(129)
            )
        ]
    );
    assert_eq!(
        get_exiting_edges(VertexId(126)),
        vec![
            (
                EdgeProperty {
                    id: EdgeId(8323953),
                    length: Length::from_meters(16),
                    frc: Frc::Frc6,
                    fow: Fow::SingleCarriageway,
                },
                VertexId(127)
            ),
            (
                EdgeProperty {
                    id: EdgeId(8323959),
                    length: Length::from_meters(11),
                    frc: Frc::Frc6,
                    fow: Fow::SingleCarriageway,
                },
                VertexId(129)
            )
        ]
    );

    assert_eq!(
        get_entering_edges(VertexId(134)),
        vec![
            (
                EdgeProperty {
                    id: EdgeId(8345026),
                    length: Length::from_meters(31),
                    frc: Frc::Frc6,
                    fow: Fow::SingleCarriageway,
                },
                VertexId(123)
            ),
            (
                EdgeProperty {
                    id: EdgeId(8345025),
                    length: Length::from_meters(199),
                    frc: Frc::Frc6,
                    fow: Fow::SingleCarriageway,
                },
                VertexId(134)
            )
        ]
    );
    assert_eq!(
        get_exiting_edges(VertexId(134)),
        vec![
            (
                EdgeProperty {
                    id: EdgeId(8345026),
                    length: Length::from_meters(31),
                    frc: Frc::Frc6,
                    fow: Fow::SingleCarriageway,
                },
                VertexId(123)
            ),
            (
                EdgeProperty {
                    id: EdgeId(8345025),
                    length: Length::from_meters(199),
                    frc: Frc::Frc6,
                    fow: Fow::SingleCarriageway,
                },
                VertexId(134)
            )
        ]
    );
}

#[test]
fn geojson_graph() {
    let geojson = include_str!("data/graph.geojson");
    let graph = GeojsonGraph::parse_geojson(geojson);

    let line = graph.lines.get(&16218).unwrap();
    assert_eq!(line.id, 16218);
    assert_eq!(line.start_id, 1);
    assert_eq!(line.end_id, 2);
    assert_eq!(line.length, 217);
    assert_eq!(line.frc, Frc::Frc2);
    assert_eq!(line.fow, Fow::SingleCarriageway);
    assert_eq!(line.direction, 2); // forward
    assert_eq!(line.geometry.len(), 7);

    let node = graph.nodes.get(&1).unwrap();
    assert_eq!(node.id, 1);
    assert_eq!(
        node.location,
        Coordinate {
            lon: 13.454214,
            lat: 52.5157088
        }
    );
    assert_eq!(node.lines, vec![(16218, 2)]);

    let node = graph.nodes.get(&2).unwrap();
    assert_eq!(node.id, 2);
    assert_eq!(node.lines, vec![(16219, 3), (3622025, 58)]);

    let node = graph.nodes.get(&29).unwrap();
    assert_eq!(node.id, 29);
    assert_eq!(node.lines, vec![(580854, 30), (2711304, 51), (2711305, 48)]);

    let node = graph.nodes.get(&126).unwrap();
    assert_eq!(node.id, 126);
    assert_eq!(node.lines, vec![(8323953, 127), (8323959, 129)]);

    let line = graph.lines.get(&8323959).unwrap();
    assert_eq!(line.id, 8323959);
    assert_eq!(line.start_id, 129);
    assert_eq!(line.end_id, 126);
    assert_eq!(line.length, 11);
    assert_eq!(line.frc, Frc::Frc6);
    assert_eq!(line.fow, Fow::SingleCarriageway);
    assert_eq!(line.direction, 1); // both ways
    assert_eq!(line.geometry.len(), 2);

    // loop
    let node = graph.nodes.get(&134).unwrap();
    assert_eq!(node.id, 134);
    assert_eq!(node.lines, vec![(8345025, 134), (8345026, 123)]);
}

impl GeojsonGraph {
    fn parse_geojson(geojson: &str) -> Self {
        let FeatureCollection { features, .. } =
            geojson::FeatureCollection::from_str(geojson).unwrap();

        let mut graph = GeojsonGraph::default();

        let fow_values: Vec<Fow> = Fow::iter().collect();
        let frc_values: Vec<Frc> = Frc::iter().collect();

        for Feature {
            geometry,
            properties,
            ..
        } in &features
        {
            let geometry = geometry.as_ref().unwrap();
            let properties = properties.as_ref().unwrap();

            if let Value::Point(point) = &geometry.value {
                let id = properties.get("id").unwrap().as_i64().unwrap();

                let location = Coordinate {
                    lon: point[0],
                    lat: point[1],
                };

                graph.nodes.insert(
                    id,
                    Node {
                        id,
                        location,
                        lines: vec![],
                    },
                );
            }
        }

        for Feature {
            geometry,
            properties,
            ..
        } in &features
        {
            let geometry = geometry.as_ref().unwrap();
            let properties = properties.as_ref().unwrap();

            if let Value::LineString(line) = &geometry.value {
                let id = properties.get("id").unwrap().as_i64().unwrap();

                let mut start_id = properties.get("startId").unwrap().as_i64().unwrap();
                let mut end_id = properties.get("endId").unwrap().as_i64().unwrap();
                let length = properties.get("length").unwrap().as_i64().unwrap() as u32;
                let frc = properties.get("frc").unwrap().as_i64().unwrap() as usize;
                let fow = properties.get("fow").unwrap().as_i64().unwrap() as usize;

                let mut geometry: Vec<Coordinate> = line
                    .iter()
                    .map(|line| Coordinate {
                        lon: line[0],
                        lat: line[1],
                    })
                    .collect();

                let direction = properties.get("direction").unwrap().as_i64().unwrap() as u8;

                if direction == 3 {
                    // backward direction
                    geometry.reverse();
                    std::mem::swap(&mut start_id, &mut end_id);
                }

                let node = graph.nodes.get_mut(&start_id).unwrap();
                node.lines.push((id, end_id));

                if direction == 1 && start_id != end_id {
                    // both directions
                    let node = graph.nodes.get_mut(&end_id).unwrap();
                    node.lines.push((id, start_id));
                }

                let line = Line {
                    id,
                    start_id,
                    end_id,
                    length,
                    frc: frc_values[frc],
                    fow: fow_values[fow],
                    geometry,
                    direction,
                };
                graph.lines.insert(id, line);
            }
        }

        graph
    }

    fn into_network_graph(self) -> NetworkGraph {
        let edge_properties = self
            .lines
            .iter()
            .map(|(&line_id, line)| {
                let edge_id: usize = line_id.try_into().unwrap();
                let edge_id: EdgeId = edge_id.into();

                let property = EdgeProperty {
                    id: edge_id,
                    length: Length::from_meters(line.length),
                    frc: line.frc,
                    fow: line.fow,
                };

                (edge_id, property)
            })
            .collect();

        let edges: Vec<(usize, usize, EdgeProperty<_>)> = self
            .nodes
            .iter()
            .flat_map(|(&from_id, node)| {
                let from_id: usize = from_id.try_into().unwrap();

                node.lines
                    .iter()
                    .map(|&(line_id, to_id)| {
                        let to_id: usize = to_id.try_into().unwrap();
                        let edge_id: usize = line_id.try_into().unwrap();

                        let line = self.lines.get(&line_id).unwrap();
                        let property = EdgeProperty {
                            id: EdgeId(edge_id),
                            length: Length::from_meters(line.length),
                            frc: line.frc,
                            fow: line.fow,
                        };

                        (to_id, property)
                    })
                    .map(move |(to_id, property)| (from_id, to_id, property))
            })
            .collect();

        let nodes: Vec<NetworkNode> = self
            .nodes
            .iter()
            .map(|(&from_id, node)| {
                let vertex: usize = from_id.try_into().unwrap();
                NetworkNode {
                    vertex: vertex.into(),
                    location: node.location,
                }
            })
            .collect();

        NetworkGraph {
            geospatial_rtree: RTree::bulk_load(nodes),
            network: graph::prelude::GraphBuilder::new()
                .edges_with_values(edges)
                .build(),
            edge_properties,
        }

        /*
        for (&node_id, node) in &self.nodes {
            let vertex_id: usize = node_id.try_into().unwrap();
            let vertex_id: VertexId = (vertex_id - 1).into();

            graph.geospatial_rtree.insert(NetworkNode {
                vertex: vertex_id,
                location: node.location,
            });

            let vertex = &mut graph.vertices[vertex_id];

            for &(line_id, node_id_to) in &node.lines {
                let line = self.lines.get(&line_id).unwrap();
                //if line.direction == 2 {
                //    assert_eq!(node_id, line.start_id);
                //} else if line.direction == 3 {
                //    assert_eq!(node_id, line.end_id);
                //}

                let edge_id: usize = line_id.try_into().unwrap();
                let edge_id: EdgeId = (edge_id - 1).into();

                let vertex_to: usize = node_id_to.try_into().unwrap();
                let vertex_to: VertexId = (vertex_to - 1).into();

                vertex.edges.push(Edge {
                    id: edge_id,
                    cost: Length::from_meters(line.length),
                    vertex_to,
                });
            }
        }
        */
    }
}
