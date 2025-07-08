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
    let candidates = find_candidate_nodes(graph, line.points);
    dbg!(&candidates);

    // Step – 3 For each location reference point find candidate lines

    // TODO!!!
    Ok(Location)
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
fn find_candidate_nodes<G, I>(graph: &G, points: I) -> Vec<CandidateNode<G::Node>>
where
    G: Graph,
    I: IntoIterator<Item = Point>,
{
    const MAX_DISTANCE: Length = Length::from_meters(10); // TODO: MaxNodeDistance 100m?

    points
        .into_iter()
        .map(|point| {
            let nodes: Vec<_> = graph
                .nearest_neighbours_within_distance(point.coordinate, MAX_DISTANCE)
                .collect();

            CandidateNode { point, nodes }
        })
        .collect()
}

/// List of candidate nodes for a Location Reference Point.
/// Nodes are sorted based on their distance to the point (closest to farthest).
#[derive(Debug)]
struct CandidateNode<N> {
    point: Point,
    nodes: Vec<N>,
}
