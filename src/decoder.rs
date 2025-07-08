use thiserror::Error;

use crate::{
    DeserializeError, Graph, Length, Line, LocationReference, Point, deserialize_base64_openlr,
};

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum DecodeError {
    #[error("Cannot decode: {0}")]
    InvalidData(DeserializeError),
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
    dbg!(&nodes);

    // Step – 3 For each location reference point find candidate lines
    let lines = find_candidate_lines(graph, &line.points, &nodes)?;

    // TODO!!!
    Ok(Location)
}

/// List of candidate nodes for a Location Reference Point.
/// Nodes are sorted based on their distance to the point (closest to farthest).
#[derive(Debug)]
struct CandidateNodes<N> {
    point: Point,
    nodes: Vec<N>,
}

/// List of candidate ways for a Location Reference Point.
/// Ways are sorted based on their rating computed using their attributes (FRC, FOW, ..).
#[derive(Debug)]
struct CandidateLines<W> {
    point: Point,
    ways: Vec<W>,
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
fn find_candidate_lines<'a, G, I>(
    graph: &G,
    points: I,
    nodes: &[CandidateNodes<G::VertexId>],
) -> Result<Vec<CandidateLines<G::EdgeId>>, DecodeError>
where
    G: Graph,
    I: IntoIterator<Item = &'a Point>,
{
    let mut lines: Vec<CandidateLines<_>> = vec![];

    for CandidateNodes { point, nodes } in nodes {
        for &node in nodes {
            // get all outgoing and incoming edges
            let mut edges: Vec<_> = graph
                .vertex_exiting_edges(node)
                .chain(graph.vertex_entering_edges(node))
                .map(|(edge, _)| edge)
                .collect();
            edges.sort_unstable();
            edges.dedup();

            println!("edges count = {}", edges.len());
        }
    }

    Ok(lines)
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
fn find_candidate_nodes<'a, G, I>(graph: &G, points: I) -> Vec<CandidateNodes<G::VertexId>>
where
    G: Graph,
    I: IntoIterator<Item = &'a Point>,
{
    const MAX_DISTANCE: Length = Length::from_meters(10); // TODO: MaxNodeDistance 100m?

    points
        .into_iter()
        .map(|&point| {
            let nodes: Vec<_> = graph
                .nearest_vertices_within_distance(point.coordinate, MAX_DISTANCE)
                .collect();

            CandidateNodes { point, nodes }
        })
        .collect()
}
