use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};
use std::fmt::Debug;

use base64::DecodeError;
use tracing::debug;

use crate::{
    CandidateLinePair, CandidateLines, DecoderConfig, DirectedGraph, Length, Point, RatingScore,
};

/// Shortest path from the LRP to the next one.
#[derive(Debug, Default)]
pub struct Route<EdgeId> {
    pub lrp: Point,
    pub length: Length,
    pub edges: Vec<EdgeId>,
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
pub fn resolve_routes<G: DirectedGraph>(
    config: &DecoderConfig,
    graph: &G,
    candidate_lines: &[CandidateLines<G::EdgeId>],
) -> Result<Vec<Route<G::EdgeId>>, DecodeError> {
    let mut routes = Vec::with_capacity(candidate_lines.len() - 1);

    // TODO: check for single route

    for candidates_pair in candidate_lines.windows(2) {
        let [candidates_lrp1, candidates_lrp2] = [&candidates_pair[0], &candidates_pair[1]];
        let pairs = resolve_top_k_candidate_pairs(config, candidates_lrp1, candidates_lrp2)?;

        // TODO: for each pair calculate shortest path
    }

    Ok(routes)
}

fn resolve_top_k_candidate_pairs<EdgeId: Debug + Copy>(
    config: &DecoderConfig,
    lines_lrp1: &CandidateLines<EdgeId>,
    lines_lrp2: &CandidateLines<EdgeId>,
) -> Result<Vec<CandidateLinePair<EdgeId>>, DecodeError> {
    let max_size = lines_lrp1.lines.len() * lines_lrp2.lines.len();
    let k_size = max_size.min(config.max_number_retries + 1);
    debug!("Resolving candidate pair ratings with size={k_size}");

    let mut pair_ratings: BinaryHeap<Reverse<RatingScore>> = BinaryHeap::with_capacity(k_size + 1);
    let mut rating_pairs: HashMap<RatingScore, Vec<_>> = HashMap::with_capacity(k_size + 1);

    for &line_lrp1 in &lines_lrp1.lines {
        for &line_lrp2 in &lines_lrp2.lines {
            let pair_rating = line_lrp1.rating * line_lrp2.rating;
            pair_ratings.push(Reverse(pair_rating));

            if pair_ratings.len() <= k_size {
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

    let mut candidates = Vec::with_capacity(k_size);
    while let Some(Reverse(rating)) = pair_ratings.pop() {
        candidates.extend(rating_pairs.remove(&rating).into_iter().flatten());
    }
    candidates.reverse();

    debug_assert_eq!(candidates.len(), k_size);
    debug_assert!(candidates.is_sorted_by_key(|pair| Reverse(pair.rating())));

    Ok(candidates)
}

#[cfg(test)]
mod tests {
    use test_log::test;

    use super::*;
    use crate::CandidateLine;

    #[test]
    fn resolve_top_k_candidate_pairs_001() {
        let config = DecoderConfig {
            max_number_retries: 3,
            ..Default::default()
        };

        let line1 = CandidateLine {
            lrp: Point::default(),
            edge: 1,
            distance_to_projection: None,
            rating: RatingScore::from(926.3),
        };

        let line2 = CandidateLine {
            lrp: Point::default(),
            edge: 2,
            distance_to_projection: Some(Length::from_meters(141.6)),
            rating: RatingScore::from(880.4),
        };

        let line3 = CandidateLine {
            lrp: Point::default(),
            edge: 3,
            distance_to_projection: None,
            rating: RatingScore::from(924.9),
        };

        let line4 = CandidateLine {
            lrp: Point::default(),
            edge: 4,
            distance_to_projection: None,
            rating: RatingScore::from(100.0),
        };

        let line5 = CandidateLine {
            lrp: Point::default(),
            edge: 5,
            distance_to_projection: None,
            rating: RatingScore::from(10.0),
        };

        let pairs = resolve_top_k_candidate_pairs(
            &config,
            &CandidateLines {
                lrp: Point::default(),
                lines: vec![line1, line2],
            },
            &CandidateLines {
                lrp: Point::default(),
                lines: vec![line3, line4, line5],
            },
        )
        .unwrap();

        assert_eq!(
            pairs,
            [
                CandidateLinePair {
                    line_lrp1: line1,
                    line_lrp2: line3
                },
                CandidateLinePair {
                    line_lrp1: line2,
                    line_lrp2: line3
                },
                CandidateLinePair {
                    line_lrp1: line1,
                    line_lrp2: line4
                },
                CandidateLinePair {
                    line_lrp1: line2,
                    line_lrp2: line4
                }
            ]
        );
    }
}
