use thiserror::Error;
use tracing::{debug, info};

use crate::{
    Bearing, DeserializeError, DirectedGraph, Fow, Frc, Length, Line, LocationReference, Point,
    deserialize_base64_openlr,
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

#[derive(Debug, Clone, Copy)]
struct DecoderConfig {
    max_node_distance: Length,
    max_bearing_difference: Bearing,
    min_line_rating: f64,
    node_factor: f64,
    line_factor: f64,
    non_junction_factor: f64,
    projected_line_factor: f64,
    bearing_distance: Length,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            max_node_distance: Length::from_meters(10.0), // TODO: MaxNodeDistance 100m?
            max_bearing_difference: Bearing::from_degrees(90),
            min_line_rating: 800.0,
            node_factor: 3.0,
            line_factor: 3.0,
            non_junction_factor: 0.8,
            projected_line_factor: 0.95,
            bearing_distance: Length::from_meters(20.0),
        }
    }
}

/// TODO
pub struct Location;

pub fn decode_base64_openlr<G: DirectedGraph>(
    graph: &G,
    data: impl AsRef<[u8]>,
) -> Result<Location, DecodeError> {
    // Step – 1 Decode physical data and check its validity
    let location = deserialize_base64_openlr(data)?;

    let config = DecoderConfig::default();

    match location {
        LocationReference::Line(line) => decode_line(&config, graph, line),
        _ => unimplemented!(),
    }
}

fn decode_line<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line: Line,
) -> Result<Location, DecodeError> {
    debug!("Decoding {line:?} with {config:?}");

    // Step – 2 For each location reference point find candidate nodes
    let nodes = find_candidate_nodes(config, graph, &line.points);
    //dbg!(&nodes);

    // Step – 3 For each location reference point find candidate lines
    let lines = find_candidate_lines_from_nodes(config, graph, &nodes)?;

    // TODO!!!
    Ok(Location)
}

/// List of candidate nodes for a Location Reference Point.
/// Nodes are sorted based on their distance to the point (closest to farthest).
#[derive(Debug)]
struct CandidateNodes<VertexId> {
    lrp: Point,
    nodes: Vec<CandidateNode<VertexId>>,
}

#[derive(Debug, Clone, Copy)]
struct CandidateNode<VertexId> {
    vertex: VertexId,
    distance_to_lrp: Length,
}

/// List of candidate ways for a Location Reference Point.
/// Ways are sorted based on their rating computed using their attributes (FRC, FOW, ..).
#[derive(Debug)]
struct CandidateLines<EdgeId> {
    lrp: Point,
    lines: Vec<AcceptedCandidateLine<EdgeId>>,
}

#[derive(Debug, Clone, Copy)]
struct AcceptedCandidateLine<EdgeId> {
    edge: EdgeId,
    rating: f64,
    distance_to_projection: Length,
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
///
/// The direct search of lines using a projection point may also be executed even if candidate nodes are
/// found. This might increase the number of candidate nodes but it could help to determine the correct
/// candidate line in the next step if the nodes in the encoder and decoder map differ significantly.
///
/// If no candidate line can be found for a location reference point, the decoder should report an error
/// and stop further processing.
fn find_candidate_lines_from_nodes<G>(
    config: &DecoderConfig,
    graph: &G,
    nodes: &[CandidateNodes<G::VertexId>],
) -> Result<Vec<CandidateLines<G::EdgeId>>, DecodeError>
where
    G: DirectedGraph,
{
    let lines: Vec<CandidateLines<_>> = vec![];

    for CandidateNodes { lrp: point, nodes } in nodes {
        debug!(
            "Evaluating candidate {point:?} with {} candidate nodes",
            nodes.len()
        );

        let mut candidate_lines = CandidateLines {
            lrp: *point,
            lines: vec![],
        };

        println!("\n\nFinding candidate lines from nodes");
        for candidate_node in nodes {
            let is_junction = graph.is_junction(candidate_node.vertex);
            // only outgoing lines are accepted for the LRPs, except the last LRP where only
            // incoming lines are accepted
            let edges: Box<dyn Iterator<Item = _>> = if point.is_last() {
                Box::new(graph.vertex_entering_edges(candidate_node.vertex).inspect(
                    |(id, from_vertex)| {
                        debug!("{id:?}: {from_vertex:?} -> {:?}", candidate_node.vertex);
                    },
                ))
            } else {
                Box::new(graph.vertex_exiting_edges(candidate_node.vertex).inspect(
                    |(id, to_vertex)| {
                        debug!("{id:?}: {:?} -> {to_vertex:?}", candidate_node.vertex);
                    },
                ))
            };

            // TODO: we don't need the vertex in the API?
            let lines = edges.into_iter().filter_map(|(edge, _)| {
                let bearing = if point.is_last() {
                    graph.get_edge_bearing_between(
                        edge,
                        graph.get_edge_length(edge)?,
                        config.bearing_distance.reverse(),
                    )?
                } else {
                    graph.get_edge_bearing_between(edge, Length::ZERO, config.bearing_distance)?
                };

                Some(CandidateLine {
                    edge,
                    vertex: candidate_node.vertex,
                    distance_to_lrp: candidate_node.distance_to_lrp,
                    distance_to_projection: Length::ZERO,
                    is_junction: Some(is_junction),
                    frc: graph.get_edge_frc(edge)?,
                    fow: graph.get_edge_fow(edge)?,
                    length: graph.get_edge_length(edge)?,
                    bearing,
                })
            });

            candidate_lines.lines.extend(
                lines
                    .into_iter()
                    .filter_map(|line| rate_line::<G>(config, *point, line)),
            );
        }

        assert_eq!(candidate_lines.lines.len(), 1);

        println!("\n\nFinding candidate lines from projected lines");
        let mut projected_lines =
            find_projected_candidate_lines(config, graph, &mut candidate_lines);
        dbg!(projected_lines.len());

        candidate_lines.lines.append(&mut projected_lines);
        dbg!(candidate_lines.lines.len());

        panic!();
    }

    Ok(lines)
}

fn find_projected_candidate_lines<G>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &mut CandidateLines<G::EdgeId>,
) -> Vec<AcceptedCandidateLine<G::EdgeId>>
where
    G: DirectedGraph,
{
    // TODO???
    // only outgoing lines are accepted for the LRPs, except the last LRP where only
    // incoming lines are accepted
    graph
        .nearest_edges_within_distance(candidate_lines.lrp.coordinate, config.max_node_distance)
        .filter_map(|(edge, distance_to_lrp)| {
            let vertex = if candidate_lines.lrp.is_last() {
                graph.get_edge_end_vertex(edge)?
            } else {
                graph.get_edge_start_vertex(edge)?
            };

            let distance_to_projection =
                graph.get_distance_from_start_vertex(edge, candidate_lines.lrp.coordinate)?;

            let bearing = if candidate_lines.lrp.is_last() {
                graph.get_edge_bearing_between(
                    edge,
                    graph.get_edge_length(edge)? - distance_to_projection,
                    config.bearing_distance.reverse(),
                )?
            } else {
                graph.get_edge_bearing_between(
                    edge,
                    distance_to_projection,
                    config.bearing_distance,
                )?
            };

            Some(CandidateLine {
                edge,
                vertex,
                distance_to_lrp,
                distance_to_projection,
                is_junction: None,
                frc: graph.get_edge_frc(edge)?,
                fow: graph.get_edge_fow(edge)?,
                length: graph.get_edge_length(edge)?,
                bearing,
            })
        })
        .filter_map(|line| {
            let mut projected_candidate = rate_line::<G>(config, candidate_lines.lrp, line)?;

            if !candidate_lines.lines.is_empty() {
                projected_candidate.rating *= config.projected_line_factor;
                if projected_candidate.rating < config.min_line_rating {
                    return None;
                }
            }

            if let Some(candidate) = candidate_lines
                .lines
                .iter_mut()
                .find(|candidate| candidate.edge == line.edge)
            {
                if candidate.rating < projected_candidate.rating {
                    // TODO: update candidate offset with projected line offset
                    candidate.rating = projected_candidate.rating;
                }

                None
            } else {
                Some(projected_candidate)
            }
        })
        .collect()
}

#[derive(Debug, Clone, Copy)]
struct CandidateLine<EdgeId, VertexId> {
    /// Edge of the line that exits the LRP (or enters the last LRP).
    edge: EdgeId,
    /// Start vertex of the line (or end vertex if it's the last LRP).
    vertex: VertexId,
    /// Distance from the LRP to the vertex (or to the edge if the LRP was projected).
    distance_to_lrp: Length,
    /// Distance from the vertex to the projected LRP into the edge.
    /// If the LRP is not projected it will be 0.
    distance_to_projection: Length,
    /// True only if the LRP was not projected, and the vertex of the edge that corresponds to the
    /// LRP connects to more than other 2 nodes.
    is_junction: Option<bool>,
    /// Functional Road CLass.
    frc: Frc,
    /// Form of Way.
    fow: Fow,
    /// Length of the full line.
    length: Length,
    /// Bearing of the part of the line that will be considered starting from the distance to the
    /// LRP projection.
    bearing: Bearing,
}

fn rate_line<G: DirectedGraph>(
    config: &DecoderConfig,
    lrp: Point,
    line: CandidateLine<G::EdgeId, G::VertexId>,
) -> Option<AcceptedCandidateLine<G::EdgeId>> {
    debug!("Rating: {line:?}");

    if let Some(path) = &lrp.path
        && !line.frc.is_within_variance(&path.lfrcnp)
    {
        debug_assert!(!lrp.is_last());
        debug!("Candidate FRC variance out of bounds: {line:?}");
        return None;
    }

    #[derive(Debug)]
    struct Ratings {
        distance: f64,
        bearing: f64,
        frc: f64,
        fow: f64,
    }

    let ratings = Ratings {
        distance: (config.max_node_distance - line.distance_to_lrp).meters() as f64,
        bearing: Bearing::rating_score(line.bearing.rating(&lrp.line.bearing)),
        frc: Frc::rating_score(line.frc.rating(&lrp.line.frc)),
        fow: Fow::rating_score(line.fow.rating(&lrp.line.fow)),
    };

    let node_rating = {
        debug_assert!(ratings.distance.is_sign_positive());
        let rating = config.node_factor * ratings.distance;
        if let Some(false) = line.is_junction {
            rating * config.non_junction_factor
        } else {
            rating
        }
    };

    let line_rating = {
        let rating = ratings.bearing + ratings.frc + ratings.fow;
        config.line_factor * rating
    };

    let rating = node_rating + line_rating;
    debug!("Rating={rating:.1} {ratings:?}");

    if rating < config.min_line_rating {
        None
    } else {
        let accepted_line = AcceptedCandidateLine {
            edge: line.edge,
            rating,
            distance_to_projection: line.distance_to_projection,
        };
        debug!("Accepted: {accepted_line:?}");
        Some(accepted_line)
    }
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
    config: &DecoderConfig,
    graph: &G,
    points: I,
) -> Vec<CandidateNodes<G::VertexId>>
where
    G: DirectedGraph,
    I: IntoIterator<Item = &'a Point>,
{
    points
        .into_iter()
        .map(|&lrp| {
            let nodes = graph
                .nearest_vertices_within_distance(lrp.coordinate, config.max_node_distance)
                .map(|(vertex, distance_to_lrp)| CandidateNode {
                    vertex,
                    distance_to_lrp,
                })
                .collect();

            CandidateNodes { lrp, nodes }
        })
        .collect()
}
