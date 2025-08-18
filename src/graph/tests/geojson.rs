use std::collections::HashMap;
use std::sync::LazyLock;

use geo::{CoordsIter, LineString, coord};
use geojson::{Feature, FeatureCollection, Value};

use crate::{Coordinate, Fow, Frc, Length};

pub static GEOJSON_GRAPH: LazyLock<GeojsonGraph> =
    LazyLock::new(|| GeojsonGraph::parse_geojson(include_str!("graph.geojson")));

type NodeId = u64;

/// Identify a directed line (negative value represents a reversed edge).
type LineId = i64;

#[derive(Debug, Default)]
pub struct GeojsonGraph {
    pub nodes: HashMap<NodeId, Node>,
    pub lines: HashMap<LineId, Line>,
}

#[derive(Debug)]
pub struct Node {
    pub coordinate: Coordinate,
    pub exiting_lines: Vec<(LineId, NodeId)>,
}

#[derive(Debug)]
pub struct Line {
    pub start_node_id: NodeId,
    pub end_node_id: NodeId,
    pub length: Length,
    pub fow: Fow,
    pub frc: Frc,
    pub geometry: LineString,
}

impl GeojsonGraph {
    fn parse_geojson(geojson: &str) -> Self {
        let FeatureCollection { features, .. } = geojson.parse().unwrap();

        let mut graph = GeojsonGraph::default();

        for Feature {
            geometry,
            properties,
            ..
        } in &features
        {
            let geometry = geometry.as_ref().unwrap();
            let properties = properties.as_ref().unwrap();

            if let Value::Point(point) = &geometry.value {
                let id = properties.get("id").unwrap().as_i64().unwrap() as u64;

                let coordinate = Coordinate {
                    lon: point[0],
                    lat: point[1],
                };

                graph.nodes.insert(
                    id,
                    Node {
                        coordinate,
                        exiting_lines: vec![],
                    },
                );
            }
        }

        for Feature {
            geometry,
            properties,
            ..
        } in features
        {
            let geometry = geometry.as_ref().unwrap();
            let properties = properties.as_ref().unwrap();

            if let Value::LineString(lines) = &geometry.value {
                let id = properties.get("id").unwrap().as_i64().unwrap();
                let length = properties.get("length").unwrap().as_i64().unwrap() as f64;
                let frc = properties.get("frc").unwrap().as_i64().unwrap() as i8;
                let fow = properties.get("fow").unwrap().as_i64().unwrap() as i8;
                let direction = properties.get("direction").unwrap().as_i64().unwrap();
                let geometry = lines.iter().map(|line| coord! { x: line[0], y: line[1] });

                let mut start_node_id = properties.get("startId").unwrap().as_i64().unwrap() as u64;
                let mut end_node_id = properties.get("endId").unwrap().as_i64().unwrap() as u64;

                let geometry = if direction == 3 {
                    // backward direction
                    std::mem::swap(&mut start_node_id, &mut end_node_id);
                    LineString::from_iter(geometry.rev())
                } else {
                    LineString::from_iter(geometry)
                };

                let node = graph.nodes.get_mut(&start_node_id).unwrap();
                node.exiting_lines.push((id, end_node_id));

                if direction == 1 && start_node_id != end_node_id {
                    // both directions: add also the line in the opposite direction
                    let node = graph.nodes.get_mut(&end_node_id).unwrap();
                    node.exiting_lines.push((-id, start_node_id));
                }

                graph.lines.insert(
                    id,
                    Line {
                        start_node_id,
                        end_node_id,
                        length: Length::from_meters(length),
                        frc: Frc::from_value(frc).unwrap(),
                        fow: Fow::from_value(fow).unwrap(),
                        geometry,
                    },
                );
            }
        }

        graph
    }
}

#[test]
fn geojson_graph_line_attributes() {
    let graph = &GEOJSON_GRAPH;

    let line = graph.lines.get(&16218).unwrap();
    assert_eq!(line.start_node_id, 1);
    assert_eq!(line.end_node_id, 2);
    assert_eq!(line.length, Length::from_meters(217.0));
    assert_eq!(line.frc, Frc::Frc2);
    assert_eq!(line.fow, Fow::SingleCarriageway);
    assert_eq!(line.geometry.coords_count(), 7);

    let line = graph.lines.get(&8323959).unwrap();
    assert_eq!(line.start_node_id, 129);
    assert_eq!(line.end_node_id, 126);
    assert_eq!(line.length, Length::from_meters(11.0));
    assert_eq!(line.frc, Frc::Frc6);
    assert_eq!(line.fow, Fow::SingleCarriageway);
    assert_eq!(line.geometry.coords_count(), 2);
}

#[test]
fn geojson_graph_node_attributes() {
    let graph = &GEOJSON_GRAPH;

    let node = graph.nodes.get(&1).unwrap();
    assert_eq!(node.coordinate.lon, 13.454214);
    assert_eq!(node.coordinate.lat, 52.5157088);
    assert_eq!(node.exiting_lines, vec![(16218, 2)]);

    let node = graph.nodes.get(&2).unwrap();
    assert_eq!(node.exiting_lines, vec![(16219, 3), (-3622025, 58)]);

    let node = graph.nodes.get(&29).unwrap();
    assert_eq!(
        node.exiting_lines,
        vec![(580854, 30), (-2711304, 51), (2711305, 48)]
    );

    let node = graph.nodes.get(&126).unwrap();
    assert_eq!(node.exiting_lines, vec![(8323953, 127), (-8323959, 129)]);

    let node = graph.nodes.get(&134).unwrap(); // loop
    assert_eq!(node.exiting_lines, vec![(8345025, 134), (8345026, 123)]);
}
