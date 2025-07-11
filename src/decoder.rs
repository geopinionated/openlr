use thiserror::Error;
use tracing::debug;

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
            max_node_distance: Length::from_meters(10), // TODO: MaxNodeDistance 100m?
            max_bearing_difference: Bearing::from_degrees(90),
            min_line_rating: 800.0,
            node_factor: 3.0,
            line_factor: 3.0,
            non_junction_factor: 0.8,
            projected_line_factor: 0.95,
            bearing_distance: Length::from_meters(20),
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
    point: Point,
    nodes: Vec<CandidateNode<VertexId>>,
}

#[derive(Debug, Clone, Copy)]
struct CandidateNode<VertexId> {
    node: VertexId,
    distance_to_lrp: Length,
}

/// List of candidate ways for a Location Reference Point.
/// Ways are sorted based on their rating computed using their attributes (FRC, FOW, ..).
#[derive(Debug)]
struct CandidateLines<EdgeId> {
    point: Point,
    lines: Vec<CandidateLine<EdgeId>>,
}

#[derive(Debug, Clone, Copy)]
struct CandidateLine<EdgeId> {
    line: EdgeId,
    rating: f64,
    // offset: Length, // The projection along the line in meter (from start)
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

    for CandidateNodes { point, nodes } in nodes {
        debug!(
            "Evaluating candidate {point:?} with {} candidate nodes",
            nodes.len()
        );

        let mut candidate_lines = CandidateLines {
            point: *point,
            lines: vec![],
        };

        println!("Finding candidate lines from nodes");
        for candidate_node in nodes {
            let is_junction = graph.is_junction(candidate_node.node);
            // only outgoing lines are accepted for the LRPs, except the last LRP where only
            // incoming lines are accepted
            let edges: Box<dyn Iterator<Item = _>> = if point.is_last() {
                Box::new(graph.vertex_entering_edges(candidate_node.node).inspect(
                    |(id, from_vertex)| {
                        debug!("{id:?}: {from_vertex:?} -> {:?}", candidate_node.node);
                    },
                ))
            } else {
                Box::new(graph.vertex_exiting_edges(candidate_node.node).inspect(
                    |(id, to_vertex)| {
                        debug!("{id:?}: {:?} -> {to_vertex:?}", candidate_node.node);
                    },
                ))
            };

            let lines: Vec<_> = if point.is_last() {
                graph
                    .vertex_entering_edges(candidate_node.node)
                    .inspect(|(id, from_vertex)| {
                        println!(
                            "Entering {id:?}: {from_vertex:?} -> {:?}",
                            candidate_node.node
                        );
                    })
                    .filter_map(|(edge, _)| {
                        Some(NetworkLine {
                            edge,
                            vertex: candidate_node.node,
                            distance_to_lrp: candidate_node.distance_to_lrp,
                            distance_to_projection: graph.get_edge_length(edge)?,
                            is_junction,
                            frc: graph.get_edge_frc(edge)?,
                            fow: graph.get_edge_fow(edge)?,
                            length: graph.get_edge_length(edge)?,
                            bearing: graph.get_edge_bearing_between(
                                edge,
                                Length::ZERO,
                                config.bearing_distance,
                            )?,
                        })
                    })
                    .collect()
            } else {
                graph
                    .vertex_exiting_edges(candidate_node.node)
                    .inspect(|(id, to_vertex)| {
                        println!("Exiting {id:?}: {:?} -> {to_vertex:?}", candidate_node.node);
                    })
                    .filter_map(|(edge, _)| {
                        Some(NetworkLine {
                            edge,
                            vertex: candidate_node.node,
                            distance_to_lrp: candidate_node.distance_to_lrp,
                            distance_to_projection: Length::ZERO,
                            is_junction,
                            frc: graph.get_edge_frc(edge)?,
                            fow: graph.get_edge_fow(edge)?,
                            length: graph.get_edge_length(edge)?,
                            bearing: graph.get_edge_bearing_between(
                                edge,
                                Length::ZERO,
                                config.bearing_distance,
                            )?,
                        })
                    })
                    .collect()
            };

            //edges.sort_unstable();
            //edges.dedup();

            candidate_lines.lines.extend(
                lines
                    .into_iter()
                    .filter_map(|line| rate_line(config, graph, *point, line))
                    .inspect(|candidate| println!("{candidate:?}")),
            );

            //panic!();
        }

        println!("\nFinding candidate lines from projected lines");
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
) -> Vec<CandidateLine<G::EdgeId>>
where
    G: DirectedGraph,
{
    graph
        // TODO?
        // only outgoing lines are accepted for the LRPs, except the last LRP
        // where only incoming lines are accepted
        .nearest_edges_within_distance(candidate_lines.point.coordinate, config.max_node_distance)
        .filter_map(|(edge, distance_to_lrp)| {
            let vertex = if candidate_lines.point.is_last() {
                graph.get_edge_end_vertex(edge)?
            } else {
                graph.get_edge_start_vertex(edge)?
            };

            let distance_to_projection =
                graph.get_distance_from_start_vertex(edge, candidate_lines.point.coordinate)?;

            Some(NetworkLine {
                edge,
                vertex,
                distance_to_lrp,
                distance_to_projection,
                is_junction: false,
                frc: graph.get_edge_frc(edge)?,
                fow: graph.get_edge_fow(edge)?,
                length: graph.get_edge_length(edge)?,
                bearing: graph.get_edge_bearing_between(
                    edge,
                    distance_to_projection,
                    config.bearing_distance,
                )?,
            })
        })
        .filter_map(|line| {
            let mut projected_candidate = rate_line(config, graph, candidate_lines.point, line)?;

            if !candidate_lines.lines.is_empty() {
                projected_candidate.rating *= config.projected_line_factor;
                if projected_candidate.rating < config.min_line_rating {
                    return None;
                }
            }

            if let Some(candidate) = candidate_lines
                .lines
                .iter_mut()
                .find(|candidate| candidate.line == line.edge)
            {
                if candidate.rating < projected_candidate.rating {
                    // TODO: update candidate offset with projected line offset
                    candidate.rating = projected_candidate.rating;
                }

                None
            } else {
                println!("PROJECTED: {projected_candidate:?}");
                Some(projected_candidate)
            }
        })
        .collect()
}

// TODO
// offset: Length, // distance from the node to the part of the line that should be considered
// (AKA: projection along line)
#[derive(Debug, Clone, Copy)]
struct NetworkLine<EdgeId, VertexId> {
    /// Edge of the line that exits the LRP (or enters the last LRP).
    edge: EdgeId,
    /// Start vertex of the line (or end vertex if it's the last LRP).
    vertex: VertexId,
    /// Distance from the LRP to the vertex (or to the edge if the LRP was projected).
    distance_to_lrp: Length,
    /// Distance from the vertex to the projected LRP into the edge.
    /// If the LRP is not projected it will be 0 (or equal to the edge length for the last LRP).
    distance_to_projection: Length,
    /// True only if the LRP was not projected and the vertex connects to more than other 2 nodes.
    is_junction: bool,
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

fn rate_line<G>(
    config: &DecoderConfig,
    graph: &G,
    lrp: Point,
    line: NetworkLine<G::EdgeId, G::VertexId>,
) -> Option<CandidateLine<G::EdgeId>>
where
    G: DirectedGraph,
{
    //let frc = graph.get_edge_frc(line.edge)?;
    //let fow = graph.get_edge_fow(line.edge)?;
    //let length = graph.get_edge_length(line.edge)?;
    //let bearing = graph.get_edge_bearing_between(
    //    line.edge,
    //    line.distance_to_projection,
    //    config.bearing_distance,
    //)?;

    debug!("Rating candidate: {line:?}");

    if let Some(path) = &lrp.path
        && !line.frc.is_within_variance(&path.lfrcnp)
    {
        debug!("Candidate line variance out of bounds: {line:?}");
        return None;
    }

    let mut node_rating = (config.max_node_distance - line.distance_to_lrp).meters() as f64;
    debug_assert!(node_rating.is_sign_positive());

    if !line.is_junction {
        node_rating *= config.non_junction_factor;
    }

    let bearing_rating = Bearing::rating_score(line.bearing.rating(&lrp.line.bearing));

    let frc_rating = Frc::rating_score(line.frc.rating(&lrp.line.frc));
    let fow_rating = Fow::rating_score(line.fow.rating(&lrp.line.fow));

    let line_rating = bearing_rating + frc_rating + fow_rating;
    let rating = config.node_factor * node_rating + config.line_factor * line_rating;
    debug!("Line rating = {rating:.1}");

    if rating < config.min_line_rating {
        None
    } else {
        Some(CandidateLine {
            line: line.edge,
            rating,
        })
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
        .map(|&point| {
            let nodes = graph
                .nearest_vertices_within_distance(point.coordinate, config.max_node_distance)
                .map(|(node, distance_to_lrp)| CandidateNode {
                    node,
                    distance_to_lrp,
                })
                .collect();

            CandidateNodes { point, nodes }
        })
        .collect()
}
