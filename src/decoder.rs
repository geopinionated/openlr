use thiserror::Error;
use tracing::{debug, info, trace, warn};

use crate::{
    Bearing, DeserializeError, DirectedGraph, Fow, Frc, Length, Line, LocationReference, Point,
    RatingScore, deserialize_base64_openlr,
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
    min_line_rating: RatingScore,
    node_factor: f64,
    line_factor: f64,
    non_junction_factor: f64,
    projected_line_factor: f64,
    bearing_distance: Length,
    max_bearing_difference: Bearing,
}

impl Default for DecoderConfig {
    fn default() -> Self {
        Self {
            max_node_distance: Length::from_meters(10.0), // TODO: MaxNodeDistance 100m?
            max_bearing_difference: Bearing::from_degrees(90),
            min_line_rating: RatingScore::from(800.0),
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
    info!("Decoding {line:?} with {config:?}");

    // Step – 2 For each location reference point find candidate nodes
    let nodes = find_candidate_nodes(config, graph, &line.points);

    // Step – 3 For each location reference point find candidate lines
    let lines = find_candidate_lines(config, graph, &nodes)?;

    println!("\n\n");
    for CandidateLines { lrp, lines } in lines {
        println!("\n{:?}", lrp.coordinate);
        for line in lines {
            println!("\t{line:?}");
        }
    }

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
    rating: RatingScore,
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
/// The direct search of lines using a projection point may also be executed even if candidate nodes
/// are found. This might increase the number of candidate nodes but it could help to determine the
/// correct candidate line in the next step if the nodes in the encoder and decoder map differ
/// significantly.
///
/// If no candidate line can be found for a location reference point, the decoder should report an
/// error and stop further processing.
fn find_candidate_lines<G>(
    config: &DecoderConfig,
    graph: &G,
    candidate_nodes: &[CandidateNodes<G::VertexId>],
) -> Result<Vec<CandidateLines<G::EdgeId>>, DecodeError>
where
    G: DirectedGraph,
{
    let mut candidate_lines = Vec::with_capacity(candidate_nodes.len());

    for lrp_nodes in candidate_nodes {
        println!("\n\n");
        //

        let mut lrp_lines = find_line_candidates_from_nodes(config, graph, lrp_nodes);
        //assert_eq!(lrp_lines.lines.len(), 1); // TODO

        append_projected_lines(config, graph, &mut lrp_lines);
        //assert_eq!(lrp_lines.lines.len(), 2); // TODO

        candidate_lines.push(lrp_lines);
    }

    Ok(candidate_lines)
}

fn find_line_candidates_from_nodes<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_nodes: &CandidateNodes<G::VertexId>,
) -> CandidateLines<G::EdgeId> {
    let CandidateNodes { lrp, nodes } = candidate_nodes;
    info!("Finding lines from {} nodes of {lrp:?}", nodes.len());

    let mut candidate_lines = CandidateLines {
        lrp: *lrp,
        lines: vec![],
    };

    for &CandidateNode {
        vertex,
        distance_to_lrp,
    } in nodes
    {
        let is_junction = graph.is_junction(vertex);

        // only outgoing lines are accepted for the LRPs
        // except the last LRP where only incoming lines are accepted
        let edges: Box<dyn Iterator<Item = _>> = if lrp.is_last() {
            debug!("Finding candidate lines to {vertex:?}");
            Box::new(graph.vertex_entering_edges(vertex))
        } else {
            debug!("Finding candidate lines from {vertex:?}");
            Box::new(graph.vertex_exiting_edges(vertex))
        };

        let candidates = edges.into_iter().filter_map(|(edge, _)| {
            let bearing = if lrp.is_last() {
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
                distance_to_lrp,
                distance_to_projection: Length::ZERO,
                is_junction: Some(is_junction),
                frc: graph.get_edge_frc(edge)?,
                fow: graph.get_edge_fow(edge)?,
                bearing,
            })
        });

        candidate_lines.lines.extend(
            candidates
                .filter_map(|line| rate_line::<G>(config, *lrp, line))
                .inspect(|line| debug!("{line:?}")),
        );
    }

    candidate_lines
}

fn append_projected_lines<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &mut CandidateLines<G::EdgeId>,
) {
    let lrp = candidate_lines.lrp;
    info!("Finding candidates from projected lines of {lrp:?}");

    let projected_lines = graph
        .nearest_edges_within_distance(lrp.coordinate, config.max_node_distance)
        .filter_map(|(edge, distance_to_lrp)| {
            println!("===== PROJECTED: {edge:?}");
            let distance_to_projection =
                graph.get_distance_from_start_vertex(edge, lrp.coordinate)?;

            // TODO!!
            let length = graph.get_edge_length(edge).unwrap();
            if distance_to_projection > length {
                warn!("TOO LONG! {distance_to_projection:?} > {length:?}");
                //return None;
            }

            let bearing = if lrp.is_last() {
                dbg!(distance_to_projection);
                dbg!(graph.get_edge_length(edge)? - distance_to_projection);
                graph.get_edge_bearing_between(
                    edge,
                    //dbg!(graph.get_edge_length(edge)? - distance_to_projection),
                    distance_to_projection,
                    dbg!(config.bearing_distance.reverse()),
                )?
            } else {
                graph.get_edge_bearing_between(
                    edge,
                    distance_to_projection,
                    config.bearing_distance,
                )?
            };

            let line = CandidateLine {
                edge,
                distance_to_lrp,
                distance_to_projection,
                is_junction: None,
                frc: graph.get_edge_frc(edge)?,
                fow: graph.get_edge_fow(edge)?,
                bearing,
            };

            rate_line::<G>(config, lrp, line)
        });

    for mut projected_line in projected_lines {
        if !candidate_lines.lines.is_empty() {
            projected_line.rating *= config.projected_line_factor;
            if projected_line.rating < config.min_line_rating {
                info!("DISCARDING because projected rating became too low");
                continue;
            }
        }

        println!(
            "\nPROJECTED RATED: {:?} {:?}",
            projected_line.edge, projected_line.rating
        );

        if let Some(candidate) = candidate_lines
            .lines
            .iter_mut()
            .find(|candidate| candidate.edge == projected_line.edge)
        {
            if candidate.rating < projected_line.rating {
                info!("UPDATING with {projected_line:?}");
                candidate.rating = projected_line.rating;
                candidate.distance_to_projection = projected_line.distance_to_projection;
            }
            info!(
                "DISCARDING because already exists with better rating {:?} > {:?}",
                candidate.rating, projected_line.rating
            );
        } else {
            info!("ACCEPTED PROJECTION: {projected_line:?}");
            candidate_lines.lines.push(projected_line);
        }

        println!();
    }
}

#[derive(Debug, Clone, Copy)]
struct CandidateLine<EdgeId> {
    /// Edge of the candidate line.
    /// When not projected, this edge exits the LRP (or enters the last LRP).
    edge: EdgeId,
    /// Distance from the LRP to the candidate line: candidate node vertex or to candidate line
    /// edge if the LRP was projected.
    distance_to_lrp: Length,
    /// Distance from the start of the edge to the projected LRP into the edge.
    /// If the LRP is not projected it will be 0.
    distance_to_projection: Length,
    /// True only if the LRP was not projected, and candidate node connects to more than 2
    /// other nodes.
    is_junction: Option<bool>,
    /// Functional Road Class of the line.
    frc: Frc,
    /// Form of Way of the line.
    fow: Fow,
    /// Bearing of the part of the line that will be considered starting from the distance to the
    /// LRP projection (of a fixed length).
    bearing: Bearing,
}

fn rate_line<G: DirectedGraph>(
    config: &DecoderConfig,
    lrp: Point,
    line: CandidateLine<G::EdgeId>,
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
        distance: RatingScore,
        bearing: RatingScore,
        frc: RatingScore,
        fow: RatingScore,
    }

    let ratings = Ratings {
        distance: RatingScore::from(config.max_node_distance - line.distance_to_lrp),
        bearing: Bearing::rating_score(line.bearing.rating(&lrp.line.bearing)),
        frc: Frc::rating_score(line.frc.rating(&lrp.line.frc)),
        fow: Fow::rating_score(line.fow.rating(&lrp.line.fow)),
    };

    let node_rating = {
        let rating = config.node_factor * ratings.distance;
        if let Some(false) = line.is_junction {
            config.non_junction_factor * rating
        } else {
            rating
        }
    };

    let line_rating = {
        let rating = ratings.bearing + ratings.frc + ratings.fow;
        config.line_factor * rating
    };

    let rating = node_rating + line_rating;
    debug!("Rating = {rating:?} {ratings:?}");

    if rating < config.min_line_rating {
        None
    } else {
        Some(AcceptedCandidateLine {
            edge: line.edge,
            rating,
            distance_to_projection: line.distance_to_projection,
        })
    }
}

/// Each location reference point contains coordinates specifying a node in the encoder map. The
/// decoder should try to find so called candidate nodes in the decoder map whereby the coordinates
/// of the candidate nodes are close to the coordinates of the location reference point coordinates.
/// The straight line distance should be used to identify close-by nodes. Nodes in the decoder map
/// which are far away from the coordinates of the location reference point should not be considered
/// as candidate nodes in the further processing. It might happen that several candidate nodes for
/// one location reference point exist.
///
/// If no candidate node has been determined for a location reference point the decoder should try
/// to determine a candidate line directly. The LRP coordinate can be projected onto lines which are
/// not far away from that coordinate.
fn find_candidate_nodes<'a, G, I>(
    config: &DecoderConfig,
    graph: &G,
    points: I,
) -> Vec<CandidateNodes<G::VertexId>>
where
    G: DirectedGraph,
    I: IntoIterator<Item = &'a Point>,
{
    let DecoderConfig {
        max_node_distance, ..
    } = config;

    points
        .into_iter()
        .map(|&lrp| {
            info!("Finding candidates for {lrp:?} at max distance {max_node_distance}");

            let nodes = graph
                .nearest_vertices_within_distance(lrp.coordinate, *max_node_distance)
                .map(|(vertex, distance_to_lrp)| CandidateNode {
                    vertex,
                    distance_to_lrp,
                })
                .inspect(|candidate| debug!("Found {candidate:?}"))
                .collect();

            CandidateNodes { lrp, nodes }
        })
        .collect()
}
