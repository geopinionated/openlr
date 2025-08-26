use std::hash::Hash;

use radix_heap::Radix;
use rustc_hash::FxHashMap;

use crate::Length;

impl Radix for Length {
    const RADIX_BITS: u32 = u64::RADIX_BITS;
    fn radix_similarity(&self, other: &Self) -> u32 {
        (self.round().meters().to_bits()).radix_similarity(&(other.round().meters().to_bits()))
    }
}

/// Unpacks the shortest path from destination back to origin.
pub fn unpack_path<EdgeId: Copy + Eq + Hash>(
    previous_edges: &FxHashMap<EdgeId, EdgeId>,
    destination: EdgeId,
) -> Vec<EdgeId> {
    let mut edges = vec![destination];
    let mut next = destination;

    while let Some(&e) = previous_edges.get(&next) {
        next = e;
        edges.push(e);
    }

    edges.reverse();
    edges
}
