use std::{io, vec};

use thiserror::Error;

use crate::{
    Bearing, DeserializeError, Fow, Frc, Graph, Length, Line, LocationReference, Orientation,
    Point, deserialize_base64_openlr,
};

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum DecodeError {
    #[error("Cannot decode: {0}")]
    InvalidData(DeserializeError),
    //#[error("Candidate line cannot be accepted: {0:?}")]
    //InvalidCandidateLine(Point),
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
    non_junction_factor: f64, // TODO: can we remove for now and use 1.0?
    projected_line_factor: f64,
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
        }
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

    let config = DecoderConfig::default();

    match location {
        LocationReference::Line(line) => decode_line(&config, graph, line),
        _ => unimplemented!(),
    }
}

fn decode_line<G: Graph>(
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
    G: Graph,
{
    let lines: Vec<CandidateLines<_>> = vec![];

    for CandidateNodes { point, nodes } in nodes {
        println!(
            "\nNEW POINT {point:?} with {} candidate nodes\n",
            nodes.len()
        );

        let mut candidate_lines = CandidateLines {
            point: *point,
            lines: vec![],
        };

        println!("Finding candidate lines from nodes");
        for candidate_node in nodes {
            // only outgoing lines are accepted for the LRPs
            // except the last LRP where only incoming lines are accepted
            // TODO: Box<Iterator>?
            let lines: Vec<_> = if point.is_last() {
                graph
                    .vertex_entering_edges(candidate_node.node)
                    .inspect(|(id, from_vertex)| {
                        println!(
                            "Entering {id:?}: {from_vertex:?} -> {:?}",
                            candidate_node.node
                        );
                    })
                    .map(|(edge, _)| NetworkLine {
                        lrp: *point,
                        edge,
                        vertex: candidate_node.node,
                        distance_to_lrp: candidate_node.distance_to_lrp,
                    })
                    .collect()
            } else {
                graph
                    .vertex_exiting_edges(candidate_node.node)
                    .inspect(|(id, to_vertex)| {
                        println!("Exiting {id:?}: {:?} -> {to_vertex:?}", candidate_node.node);
                    })
                    .map(|(edge, _)| NetworkLine {
                        lrp: *point,
                        edge,
                        vertex: candidate_node.node,
                        distance_to_lrp: candidate_node.distance_to_lrp,
                    })
                    .collect()
            };

            //edges.sort_unstable();
            //edges.dedup();

            candidate_lines.lines.extend(
                lines
                    .into_iter()
                    .filter_map(|line| rate_line(config, graph, line))
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
    G: Graph,
{
    graph
        .nearest_edges_within_distance(candidate_lines.point.coordinate, config.max_node_distance)
        .filter_map(|(edge, distance_to_lrp)| {
            let vertex = if candidate_lines.point.is_last() {
                graph.get_edge_end_vertex(edge)?
            } else {
                graph.get_edge_start_vertex(edge)?
            };

            Some(NetworkLine {
                lrp: candidate_lines.point,
                edge,
                vertex,
                distance_to_lrp,
            })
        })
        .filter_map(|line| {
            let mut projected_candidate = rate_line(config, graph, line)?;

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
    /// the Location Reference Point (LRP)
    lrp: Point,
    /// edge ID of the line that exits the LRP (or enters the last LRP)
    edge: EdgeId,
    /// start vertex of the line (or end vertex if it's the last LRP)
    vertex: VertexId,
    /// distance from the LRP to the node (or to the line if the LRP was projected)
    distance_to_lrp: Length,
}

fn rate_line<G>(
    config: &DecoderConfig,
    graph: &G,
    line: NetworkLine<G::EdgeId, G::VertexId>,
) -> Option<CandidateLine<G::EdgeId>>
where
    G: Graph,
{
    let frc = graph.get_edge_frc(line.edge)?;
    let fow = graph.get_edge_fow(line.edge)?;
    let length = graph.get_edge_length(line.edge)?;
    let bearing = graph.get_edge_bearing(line.edge)?;

    println!(
        "\nRATE LINE\n{:?} {:?} {frc:?} {fow:?} {length:?} {bearing:?} distance-LRP={:?} last-LRP={}",
        line.edge,
        line.vertex,
        line.distance_to_lrp,
        line.lrp.is_last()
    );

    if let Some(path) = &line.lrp.path
        && !frc.is_within_variance(&path.lfrcnp)
    {
        return None;
    }

    // compute rating
    let (orientation, projection_length) = if line.lrp.is_last() {
        (Orientation::Backward, length)
    } else {
        (Orientation::Forward, Length::ZERO)
    };

    // Calculates the node value based on the distance between the LRP position and the corresponding node.
    let mut node_rating = (config.max_node_distance - line.distance_to_lrp).meters() as f64;

    // Determine whether to apply the non-junction node factor to the node score
    // Only apply the non-junction node factor when the LRP matches a node and not a line directly
    let is_junction = if projection_length > Length::ZERO && projection_length < length {
        panic!("when can this be true?");
        false
    } else {
        // TODO: do we need to compute it every time for the same node?
        graph.is_junction(line.vertex)
    };
    //dbg!(is_junction);

    if !is_junction {
        node_rating *= config.non_junction_factor;
    }

    let bearing_rating = Bearing::rating_score(bearing.rating(&line.lrp.line.bearing));
    let frc_rating = Frc::rating_score(frc.rating(&line.lrp.line.frc));
    let fow_rating = Fow::rating_score(fow.rating(&line.lrp.line.fow));

    let line_rating = bearing_rating + frc_rating + fow_rating;
    let rating = config.node_factor * node_rating + config.line_factor * line_rating;
    dbg!(rating);

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
    G: Graph,
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
