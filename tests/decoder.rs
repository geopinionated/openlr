use std::collections::HashMap;
use std::str::FromStr;

use geojson::{Feature, FeatureCollection, Value};
use openlr::decoder_graph::{Edge, EdgeId, NetworkGraph, NetworkNode, Vertex, VertexId};
use openlr::{Coordinate, Fow, Frc, Graph, Length};
use rstar::RTree;
use strum::IntoEnumIterator;
use typed_index_collections::ti_vec;

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
fn geojson_graph_nearest_neighbours() {
    let geojson = include_str!("data/graph.geojson");
    let geojson_graph = GeojsonGraph::parse_geojson(geojson);

    let graph: NetworkGraph = geojson_graph.into_network_graph();

    let node_75_location = Coordinate {
        lon: 13.459407,
        lat: 52.5143601,
    };

    let neighbours: Vec<VertexId> = graph
        .nearest_neighbours_within_distance(node_75_location, Length::from_meters(10))
        .collect();
    println!("{neighbours:?}");

    assert_eq!(neighbours[0], VertexId(75 - 1));
    assert_eq!(neighbours[1], VertexId(140 - 1));
    assert_eq!(neighbours[2], VertexId(138 - 1));
    assert_eq!(neighbours[3], VertexId(139 - 1));
    assert_eq!(neighbours[4], VertexId(59 - 1));
    assert_eq!(neighbours[5], VertexId(72 - 1));
    assert_eq!(neighbours[6], VertexId(73 - 1));
    assert_eq!(neighbours[7], VertexId(74 - 1));
}

#[test]
fn graph_into_directed() {
    let geojson = include_str!("data/graph.geojson");
    let geojson_graph = GeojsonGraph::parse_geojson(geojson);

    let graph: NetworkGraph = geojson_graph.into_network_graph();

    let vertex = &graph.vertices[VertexId(1 - 1)];
    assert_eq!(
        vertex.edges,
        vec![Edge {
            id: EdgeId(16218 - 1),
            cost: Length::from_meters(217),
            vertex_to: VertexId(2 - 1)
        }]
    );

    let vertex = &graph.vertices[VertexId(126 - 1)];
    assert_eq!(
        vertex.edges,
        vec![
            Edge {
                id: EdgeId(8323953 - 1),
                cost: Length::from_meters(16),
                vertex_to: VertexId(127 - 1)
            },
            Edge {
                id: EdgeId(8323959 - 1),
                cost: Length::from_meters(11),
                vertex_to: VertexId(129 - 1)
            }
        ]
    );

    let vertex = &graph.vertices[VertexId(134 - 1)];
    assert_eq!(
        vertex.edges,
        vec![
            Edge {
                id: EdgeId(8345025 - 1),
                cost: Length::from_meters(199),
                vertex_to: VertexId(134 - 1)
            },
            Edge {
                id: EdgeId(8345026 - 1),
                cost: Length::from_meters(31),
                vertex_to: VertexId(123 - 1)
            }
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
        let mut graph = NetworkGraph {
            vertices: ti_vec![Vertex::default(); self.nodes.len()],
            geospatial_rtree: RTree::default(),
        };

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

        graph
    }
}
