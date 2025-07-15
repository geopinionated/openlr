use std::collections::HashMap;
use std::sync::LazyLock;

use geojson::{Feature, FeatureCollection, Value};
use openlr::{Coordinate, Fow, Frc, Length};

static GEOJSON_GRAPH: LazyLock<GeojsonGraph> = LazyLock::new(|| {
    let geojson = include_str!("../data/graph.geojson");
    GeojsonGraph::parse_geojson(geojson)
});

type NodeId = i64;

/// Identify a directed line (negative value represent a reversed edge).
type LineId = i64;

#[derive(Debug, Default)]
struct GeojsonGraph {
    nodes: HashMap<NodeId, Node>,
    lines: HashMap<LineId, Line>,
}

#[derive(Debug)]
struct Node {
    id: NodeId,
    location: Coordinate,
    outgoing_lines: Vec<(LineId, NodeId)>,
}

#[derive(Debug)]
struct Line {
    id: LineId,
    start_id: NodeId,
    end_id: NodeId,
    length: Length,
    fow: Fow,
    frc: Frc,
    geometry: Vec<Coordinate>,
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
                        outgoing_lines: vec![],
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

            if let Value::LineString(line) = &geometry.value {
                let id = properties.get("id").unwrap().as_i64().unwrap();
                let mut start_id = properties.get("startId").unwrap().as_i64().unwrap();
                let mut end_id = properties.get("endId").unwrap().as_i64().unwrap();
                let length = properties.get("length").unwrap().as_i64().unwrap() as f64;
                let frc = properties.get("frc").unwrap().as_i64().unwrap() as i8;
                let fow = properties.get("fow").unwrap().as_i64().unwrap() as i8;
                let direction = properties.get("direction").unwrap().as_i64().unwrap();

                let mut geometry: Vec<Coordinate> = line
                    .iter()
                    .map(|line| Coordinate {
                        lon: line[0],
                        lat: line[1],
                    })
                    .collect();

                if direction == 3 {
                    // backward direction
                    geometry.reverse();
                    std::mem::swap(&mut start_id, &mut end_id);
                }

                let node = graph.nodes.get_mut(&start_id).unwrap();
                node.outgoing_lines.push((id, end_id));

                if direction == 1 && start_id != end_id {
                    // both directions
                    let node = graph.nodes.get_mut(&end_id).unwrap();
                    node.outgoing_lines.push((-id, start_id));
                }

                graph.lines.insert(
                    id,
                    Line {
                        id,
                        start_id,
                        end_id,
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
    assert_eq!(line.id, 16218);
    assert_eq!(line.start_id, 1);
    assert_eq!(line.end_id, 2);
    assert_eq!(line.length, Length::from_meters(217.0));
    assert_eq!(line.frc, Frc::Frc2);
    assert_eq!(line.fow, Fow::SingleCarriageway);
    assert_eq!(line.geometry.len(), 7);

    let line = graph.lines.get(&8323959).unwrap();
    assert_eq!(line.id, 8323959);
    assert_eq!(line.start_id, 129);
    assert_eq!(line.end_id, 126);
    assert_eq!(line.length, Length::from_meters(11.0));
    assert_eq!(line.frc, Frc::Frc6);
    assert_eq!(line.fow, Fow::SingleCarriageway);
    assert_eq!(line.geometry.len(), 2);
}

#[test]
fn geojson_graph_node_attributes() {
    let graph = &GEOJSON_GRAPH;

    let node = graph.nodes.get(&1).unwrap();
    assert_eq!(node.id, 1);
    assert_eq!(node.location.lon, 13.454214);
    assert_eq!(node.location.lat, 52.5157088);
    assert_eq!(node.outgoing_lines, vec![(16218, 2)]);

    let node = graph.nodes.get(&2).unwrap();
    assert_eq!(node.id, 2);
    assert_eq!(node.outgoing_lines, vec![(16219, 3), (-3622025, 58)]);

    let node = graph.nodes.get(&29).unwrap();
    assert_eq!(node.id, 29);
    assert_eq!(
        node.outgoing_lines,
        vec![(580854, 30), (-2711304, 51), (2711305, 48)]
    );

    let node = graph.nodes.get(&126).unwrap();
    assert_eq!(node.id, 126);
    assert_eq!(node.outgoing_lines, vec![(8323953, 127), (-8323959, 129)]);

    let node = graph.nodes.get(&134).unwrap(); // loop
    assert_eq!(node.id, 134);
    assert_eq!(node.outgoing_lines, vec![(8345025, 134), (8345026, 123)]);
}
