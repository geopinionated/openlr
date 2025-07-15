use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::fmt::{self, Debug};

use thiserror::Error;
use tracing::{debug, info};

use crate::{
    Bearing, DeserializeError, DirectedGraph, Fow, Frc, Length, Line, LocationReference, Point,
    RatingScore, ShortestPath, ShortestPathConfig, deserialize_base64_openlr, shortest_path,
};

#[derive(Error, Debug, PartialEq, Clone, Copy)]
pub enum DecodeError {
    #[error("Cannot decode: {0}")]
    InvalidData(DeserializeError),
    #[error("Cannot find candidate lines for: {0:?}")]
    CandidatesNotFound(Point),
    #[error("Cannot find route between LRPs: {0:?}")]
    RouteNotFound((Point, Point)),
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
    max_number_retries: usize,
    distance_np_variance: Length,
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
            max_number_retries: 3,
            distance_np_variance: Length::from_meters(150.0),
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Location<EdgeId> {
    Line(LineLocation<EdgeId>),
}

// TODO: Offsets!!!
#[derive(Debug, Clone, PartialEq)]
pub struct LineLocation<EdgeId> {
    pub edges: Vec<EdgeId>,
}

pub fn decode_base64_openlr<G: DirectedGraph>(
    graph: &G,
    data: impl AsRef<[u8]>,
) -> Result<Location<G::EdgeId>, DecodeError> {
    // Step – 1 Decode physical data and check its validity
    let location = deserialize_base64_openlr(data)?;

    let config = DecoderConfig::default();

    match location {
        LocationReference::Line(line) => decode_line(&config, graph, line).map(Location::Line),
        _ => unimplemented!(),
    }
}

fn decode_line<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line: Line,
) -> Result<LineLocation<G::EdgeId>, DecodeError> {
    info!("Decoding {line:?} with {config:?}");

    // Step – 2 For each location reference point find candidate nodes
    let nodes = find_candidate_nodes(config, graph, &line.points);

    // Step – 3 For each location reference point find candidate lines
    // Step – 4 Rate candidate lines for each location reference point
    let lines = find_candidate_lines(config, graph, &nodes)?;
    println!("\n\nCANDIDATE LINES:\n");
    for CandidateLines { lrp, lines } in &lines {
        println!("\n{:?}", lrp.coordinate);
        for line in lines {
            println!("\t{line:?}");
        }
    }
    println!("\n\n");

    // Step – 5 Determine shortest-path(s) between all subsequent location reference points
    let routes = resolve_routes(config, graph, &lines)?;
    println!("\n\nLRPs ROUTES:\n");
    for (i, route) in routes.iter().enumerate() {
        println!("{i}) {:?}", route.edges);
    }
    println!("\n\n");

    // TODO: Offsets!!!
    // Step – 7 Concatenate shortest-path(s) and trim path according to the offsets
    let edges: Vec<_> = routes.into_iter().flat_map(|route| route.edges).collect();

    // check that the whole path is properly connected
    for pair in edges.windows(2) {
        let [edge1, edge2] = [pair[0], pair[1]];
        let vertex = graph.get_edge_end_vertex(edge1).unwrap();
        assert!(
            graph.vertex_exiting_edges(vertex).any(|(e, _)| e == edge2),
            "{edge1:?} {edge2:?} {vertex:?}"
        );
    }

    Ok(LineLocation { edges })
}

/// Shortest path from the LRP to the next one.
#[derive(Debug, Default)]
struct Route<EdgeId> {
    lrp: Point,
    length: Length,
    edges: Vec<EdgeId>,
}

/// The decoder needs to compute a shortest-path between each pair of subsequent location reference
/// points. For each pair of location reference points suitable candidate lines must be chosen. The
/// candidate line of the first LRPs of this pair acts as start of the shortest-path calculation.
/// The candidate line of the second location reference point of this pair is the end of the
/// shortest-path calculation. If the chosen lines are equal no shortest-path calculation needs to
/// be started.
///
/// The shortest path algorithm should take the part of the network into account which contains all
/// lines having a functional road class lower than or equal to the lowest functional road class of
/// the first location reference point of the pair. This value might be altered if the decoder
/// anticipates having different functional road class values than the encoder map.
///
/// Additionally the shortest-path algorithm should fulfill the following constraints:
/// - All lengths of the lines should be measured in meters and should also be converted to
///   integer values, float values need to be rounded correctly.
/// - The search is node based and will start at the start node of the first line and will end at
///   the end node of the last line.
/// - The algorithm shall return an ordered list of lines representing the calculated shortest-path.
///
/// If no shortest-path can be calculated for two subsequent location reference points, the decoder
/// might try a different pair of candidate lines or finally fail and report an error. If a
/// different pair of candidate lines is tried it might happen that the start line needs to be
/// changed. In such a case this also affects the end line of the previous shortest-path and this
/// path also needs to be re-calculated and checked again. The number of retries of shortest-path
/// calculations should be limited in order to guarantee a fast decoding process.
fn resolve_routes<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
) -> Result<Vec<Route<G::EdgeId>>, DecodeError> {
    let mut routes = Vec::with_capacity(candidate_lines.len() - 1);

    for candidates_pair in candidate_lines.windows(2) {
        let [candidates_lrp1, candidates_lrp2] = [&candidates_pair[0], &candidates_pair[1]];
        let pairs = resolve_candidate_pairs::<G>(config, candidates_lrp1, candidates_lrp2)?;

        let CandidateLines { lrp: lrp1, .. } = candidates_lrp1;
        let CandidateLines { lrp: lrp2, .. } = candidates_lrp2;

        let lowest_frc_value = lrp1.lfrcnp().value() + Frc::variance(&lrp1.lfrcnp());
        let lowest_frc = Frc::from_value(lowest_frc_value).unwrap_or(Frc::Frc7);

        if let Some(path) = resolve_candidate_pairs_path(config, graph, &pairs, lowest_frc) {
            routes.push(Route {
                lrp: *lrp1,
                length: path.length,
                edges: path.edges,
            });
        } else {
            return Err(DecodeError::RouteNotFound((*lrp1, *lrp2)));
        }
    }

    Ok(routes)
}

fn resolve_candidate_pairs_path<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    pairs: &[CandidateLinePair<G::EdgeId>],
    lowest_frc: Frc,
) -> Option<ShortestPath<G::EdgeId>> {
    debug!("Resolving pairs {pairs:?} with lowest {lowest_frc:?}");

    for CandidateLinePair {
        line_lrp1,
        line_lrp2,
    } in pairs
    {
        if line_lrp1.edge == line_lrp2.edge {
            todo!("handle start == end");
        }

        let origin = graph.get_edge_start_vertex(line_lrp1.edge)?;
        let destination = if line_lrp2.lrp.is_last() {
            graph.get_edge_end_vertex(line_lrp2.edge)?
        } else {
            graph.get_edge_start_vertex(line_lrp2.edge)?
        };

        let path_config = ShortestPathConfig {
            lowest_frc,
            max_length: max_route_length(config, graph, line_lrp1, line_lrp2),
        };

        if let Some(path) = shortest_path(&path_config, graph, origin, destination) {
            // Step – 6 Check validity of the calculated shortest-path(s)
            debug_assert!(path.length <= path_config.max_length);
            let min_length = line_lrp1.lrp.dnp() - config.distance_np_variance;

            if path.length >= min_length {
                return Some(path);
            }
        }
    }

    None
}

fn max_route_length<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    line_lrp1: &AcceptedCandidateLine<G::EdgeId>,
    line_lrp2: &AcceptedCandidateLine<G::EdgeId>,
) -> Length {
    let mut max_distance = line_lrp1.lrp.dnp() + config.distance_np_variance;

    // shortest path can only stop at distances between real nodes, therefore we need to
    // add the complete length when computing max distance bound if the lines were projected
    if line_lrp1.is_projected() {
        max_distance += graph
            .get_edge_length(line_lrp1.edge)
            .unwrap_or(Length::ZERO);
    }
    if line_lrp2.is_projected() {
        max_distance += graph
            .get_edge_length(line_lrp2.edge)
            .unwrap_or(Length::ZERO);
    }

    max_distance
}

// TODO: we should consider previous candidate line as well!!!
// TODO: apply same line degradation to LRP 2 candidate line
fn resolve_candidate_pairs<G: DirectedGraph>(
    config: &DecoderConfig,
    line_lrp1: &CandidateLines<G::EdgeId>,
    line_lrp2: &CandidateLines<G::EdgeId>,
) -> Result<Vec<CandidateLinePair<G::EdgeId>>, DecodeError> {
    let max_size = line_lrp1.lines.len() * line_lrp2.lines.len();
    let size = max_size.min(config.max_number_retries + 1);
    debug!("Resolving candidate pair ratings with size={size}");

    let mut pair_ratings: BinaryHeap<Reverse<RatingScore>> = BinaryHeap::with_capacity(size + 1);
    let mut rating_pairs: HashMap<RatingScore, Vec<_>> = HashMap::with_capacity(size + 1);

    for &line_lrp1 in &line_lrp1.lines {
        for &line_lrp2 in &line_lrp2.lines {
            let pair_rating = line_lrp1.rating * line_lrp2.rating;
            pair_ratings.push(Reverse(pair_rating));

            if pair_ratings.len() <= size {
                rating_pairs
                    .entry(pair_rating)
                    .or_default()
                    .push(CandidateLinePair {
                        line_lrp1,
                        line_lrp2,
                    });

                continue;
            }

            let worst_rating = match pair_ratings.pop() {
                Some(Reverse(rating)) if pair_rating <= rating => continue,
                Some(Reverse(rating)) => rating,
                None => continue,
            };

            rating_pairs
                .entry(pair_rating)
                .or_default()
                .push(CandidateLinePair {
                    line_lrp1,
                    line_lrp2,
                });

            if let Some(pairs) = rating_pairs.get_mut(&worst_rating)
                && pairs.len() > 1
            {
                pairs.pop();
            } else {
                rating_pairs.remove(&worst_rating);
            }
        }
    }

    let mut candidates = Vec::with_capacity(size);
    while let Some(Reverse(rating)) = pair_ratings.pop() {
        candidates.extend(rating_pairs.get(&rating).into_iter().flatten());
    }
    candidates.reverse();
    debug_assert_eq!(candidates.len(), size);

    Ok(candidates)
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
struct CandidateLinePair<EdgeId> {
    line_lrp1: AcceptedCandidateLine<EdgeId>,
    line_lrp2: AcceptedCandidateLine<EdgeId>,
}

#[derive(Clone, Copy)]
struct AcceptedCandidateLine<EdgeId> {
    lrp: Point,
    edge: EdgeId,
    distance_to_projection: Option<Length>,
    rating: RatingScore,
}

impl<EdgeId: Debug> fmt::Debug for AcceptedCandidateLine<EdgeId> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AcceptedCandidateLine")
            .field("edge", &self.edge)
            .field("distance_to_projection", &self.distance_to_projection)
            .field("rating", &self.rating)
            .finish()
    }
}

impl<EdgeId> AcceptedCandidateLine<EdgeId> {
    const fn is_projected(&self) -> bool {
        self.distance_to_projection.is_some()
    }
}

#[derive(Debug, Clone)]
struct CandidateLine<EdgeId> {
    /// Location Reference Point associated to the line.
    lrp: Point,
    /// Edge of the candidate line.
    /// When not projected, this edge exits the LRP (or enters the last LRP).
    edge: EdgeId,
    /// Distance from the LRP to the candidate line: candidate node vertex or to candidate line
    /// edge if the LRP was projected.
    distance_to_lrp: Length,
    /// Distance from the start of the edge to the projected LRP into the edge.
    /// If the LRP is not projected it will be None.
    distance_to_projection: Option<Length>,
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
fn find_candidate_lines<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_nodes: &[CandidateNodes<G::VertexId>],
) -> Result<Vec<CandidateLines<G::EdgeId>>, DecodeError> {
    let mut candidate_lines = Vec::with_capacity(candidate_nodes.len());

    for lrp_nodes in candidate_nodes {
        let mut lrp_lines = find_line_candidates_from_nodes(config, graph, lrp_nodes);
        append_projected_lines(config, graph, &mut lrp_lines);

        if lrp_lines.lines.is_empty() {
            return Err(DecodeError::CandidatesNotFound(lrp_lines.lrp));
        }

        lrp_lines
            .lines
            .sort_unstable_by_key(|line| Reverse(line.rating));
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
                lrp: *lrp,
                edge,
                distance_to_lrp,
                distance_to_projection: None,
                is_junction: Some(is_junction),
                frc: graph.get_edge_frc(edge)?,
                fow: graph.get_edge_fow(edge)?,
                bearing,
            })
        });

        candidate_lines.lines.extend(
            candidates
                .filter_map(|line| rate_line::<G>(config, *lrp, line))
                .inspect(|line| debug!("Accepted candidate: {line:?}")),
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
            let distance_to_projection =
                graph.get_distance_from_start_vertex(edge, lrp.coordinate)?;

            let bearing = if lrp.is_last() {
                graph.get_edge_bearing_between(
                    edge,
                    distance_to_projection,
                    config.bearing_distance.reverse(),
                )?
            } else {
                graph.get_edge_bearing_between(
                    edge,
                    distance_to_projection,
                    config.bearing_distance,
                )?
            };

            let line = CandidateLine {
                lrp,
                edge,
                distance_to_lrp,
                distance_to_projection: Some(distance_to_projection),
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
                debug!("Discarding {projected_line:?} because rating became too low");
                continue;
            }
        }

        if let Some(candidate) = candidate_lines
            .lines
            .iter_mut()
            .find(|candidate| candidate.edge == projected_line.edge)
        {
            if candidate.rating < projected_line.rating {
                debug!("Overriding candidate line with {projected_line:?}");
                candidate.rating = projected_line.rating;
                candidate.distance_to_projection = projected_line.distance_to_projection;
            } else {
                debug!("Discarding {projected_line:?}: already exists with better rating");
            }
        } else {
            debug!("Accepted candidate: {projected_line:?}");
            candidate_lines.lines.push(projected_line);
        }
    }
}

/// All candidate lines for a location reference point shall be rated according to the following
/// criteria:
/// - The start node, end node for the last location reference point or projection point shall be as
///   close as possible to the coordinates of the location reference point.
/// - The functional road class of the candidate line should match the functional road class of the
///   location reference point
/// - The form of way of the candidate line should match the form of way of the location reference
///   point.
/// - The bearing of the candidate line should match indicated bearing angles of the location
///   reference point.
///
/// Slight variances in the concrete values are allowed and shall be considered in the rating
/// function.
///
/// The candidate lines should be ordered in a way that the best matching line comes first.
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

    if rating < config.min_line_rating {
        debug!("Rating (too low) = {rating:?} {ratings:?}");
        None
    } else {
        debug!("Rating (accepted) = {rating:?} {ratings:?}");
        Some(AcceptedCandidateLine {
            lrp: line.lrp,
            edge: line.edge,
            distance_to_projection: line.distance_to_projection,
            rating,
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
