use std::collections::HashMap;
use std::path;

use thiserror::Error;

use crate::{
    DeserializeError, EdgeProperty, Frc, Graph, Length, Line, LocationReference, Orientation, Poi,
    Point, deserialize_base64_openlr,
};

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum DecodeError {
    #[error("Cannot decode: {0}")]
    InvalidData(DeserializeError),
    #[error("Candidate line cannot be accepted: {0:?}")]
    InvalidCandidateLine(Point),
}

impl From<DeserializeError> for DecodeError {
    fn from(error: DeserializeError) -> Self {
        Self::InvalidData(error)
    }
}

/// TODO
pub struct Location;

pub fn decode_base64_openlr<G: Graph>(
    graph: &G,
    data: impl AsRef<[u8]>,
) -> Result<Location, DecodeError> {
    // Step – 1 Decode physical data and check its validity
    let location = deserialize_base64_openlr(data)?;

    match location {
        LocationReference::Line(line) => decode_line(graph, line),
        _ => unimplemented!(),
    }
}

fn decode_line<G: Graph>(graph: &G, line: Line) -> Result<Location, DecodeError> {
    // Step – 2 For each location reference point find candidate nodes
    let nodes = find_candidate_nodes(graph, &line.points);
    //dbg!(&nodes);

    // Step – 3 For each location reference point find candidate lines
    let lines = find_candidate_lines(graph, &nodes)?;

    // TODO!!!
    Ok(Location)
}

/// List of candidate nodes for a Location Reference Point.
/// Nodes are sorted based on their distance to the point (closest to farthest).
#[derive(Debug)]
struct CandidateNodes<VertexId, Distance> {
    point: Point,
    nodes: Vec<(VertexId, Distance)>,
}

/// List of candidate ways for a Location Reference Point.
/// Ways are sorted based on their rating computed using their attributes (FRC, FOW, ..).
#[derive(Debug)]
struct CandidateLines<EdgeId> {
    point: Point,
    ways: Vec<EdgeId>,
}

/// For each location reference point the decoder tries to determine lines which should fulfill the
/// following constraints:
/// - The start node, end node for the last location reference point or projection point shall be
///   close to the coordinates of the location reference point.
/// - The candidate lines should be outgoing lines (incoming lines for the last location reference
///   point) of the candidate nodes or projection points determined in the previous step.
/// - The candidate lines should match the attributes functional road class, form of way and
///   bearing as extracted from the physical data. Slight variances are allowed and shall be taken
///   into account in step 4.
/// The direct search of lines using a projection point may also be executed even if candidate nodes are
/// found. This might increase the number of candidate nodes but it could help to determine the correct
/// candidate line in the next step if the nodes in the encoder and decoder map differ significantly.
/// If no candidate line can be found for a location reference point, the decoder should report an error
/// and stop further processing.
fn find_candidate_lines<'a, G>(
    graph: &G,
    nodes: &[CandidateNodes<G::VertexId, G::Meter>],
) -> Result<Vec<CandidateLines<G::EdgeId>>, DecodeError>
where
    G: Graph,
{
    let mut lines: Vec<CandidateLines<_>> = vec![];

    for CandidateNodes { point, nodes } in nodes {
        println!("\nNEW POINT with {} candidate nodes", nodes.len());
        for &(node, distance) in nodes {
            println!("CANDIDATE {node:?}");
            // only outgoing lines are accepted for the LRPs
            // except the last LRP where only incoming lines are accepted
            let edges: Vec<_> = if point.is_last() {
                graph
                    .vertex_entering_edges(node)
                    .inspect(|(id, from_vertex)| {
                        println!("Entering {id:?}: {from_vertex:?} -> {node:?}");
                    })
                    .map(|(edge, _)| edge)
                    .collect()
            } else {
                // last line
                graph
                    .vertex_exiting_edges(node)
                    .inspect(|(id, to_vertex)| {
                        println!("Exiting {id:?}: {node:?} -> {to_vertex:?}");
                    })
                    .map(|(edge, _)| edge)
                    .collect()
            };

            //edges.sort_unstable();
            //edges.dedup();

            for edge in edges {
                rate_line(graph, point, edge, distance)?;
            }

            panic!();
        }
    }

    Ok(lines)
}

fn rate_line<G>(
    graph: &G,
    point: &Point,      // current Location Reference Point of the OpenLR code
    edge: G::EdgeId,    // outgoing (or ingoing of point is the last) graph edge from/into the LRP
    distance: G::Meter, // distance between the LRP coordinate and the graph (edge) vertex
) -> Result<(), DecodeError>
where
    G: Graph,
{
    let EdgeProperty {
        id,
        length,
        frc,
        fow,
    } = graph
        .get_edge_properties(edge)
        .expect("Every edge that belongs to the graph should have properties");
    assert_eq!(*id, edge);

    println!(
        "last? {} {edge:?} {distance:.2?}m {frc:?} {fow:?} {length:?}",
        point.is_last()
    );

    if !point.is_last() && !frc.is_within_variance(&point.line.frc) {
        panic!("OUT OF VARIANCE!!!");
        return Err(DecodeError::InvalidCandidateLine(*point));
    }

    // compute rating
    let (orientation, projection) = if point.is_last() {
        (Orientation::Backward, *length)
    } else {
        (Orientation::Forward, G::Meter::default())
    };

    //let distance_rating =

    Ok(())
}

/// Each location reference point contains coordinates specifying a node in the encoder map. The
/// decoder should try to find so called candidate nodes in the decoder map whereby the coordinates of
/// the candidate nodes are close to the coordinates of the location reference point coordinates. The
/// straight line distance should be used to identify close-by nodes. Nodes in the decoder map which
/// are far away from the coordinates of the location reference point should not be considered as
/// candidate nodes in the further processing. It might happen that several candidate nodes for one
/// location reference point exist.
/// If no candidate node has been determined for a location reference point the decoder should try to
/// determine a candidate line directly. The LRP coordinate can be projected onto lines which are not far
/// away from that coordinate.
fn find_candidate_nodes<'a, G, I>(
    graph: &G,
    points: I,
) -> Vec<CandidateNodes<G::VertexId, G::Meter>>
where
    G: Graph,
    I: IntoIterator<Item = &'a Point>,
{
    let max_distance: G::Meter = G::Meter::from(10.0); // TODO: MaxNodeDistance 100m?

    points
        .into_iter()
        .map(|&point| {
            println!("LRP {:?}", point.coordinate);
            let nodes = graph
                .nearest_vertices_within_distance(point.coordinate, max_distance)
                .collect();

            CandidateNodes { point, nodes }
        })
        .collect()
}
