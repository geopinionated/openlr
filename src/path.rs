use crate::{DirectedGraph, Length};

#[derive(Debug, Clone, PartialEq)]
pub struct Path<EdgeId> {
    pub length: Length,
    pub edges: Vec<EdgeId>,
}

impl<EdgeId> Default for Path<EdgeId> {
    fn default() -> Self {
        Self {
            length: Length::ZERO,
            edges: vec![],
        }
    }
}

/// Returns true only if all the edges of the path are sequentially connected in the given graph.
/// If turning between any of the sequentlay edges is not allowed returns false.
pub fn is_path_connected<G: DirectedGraph>(graph: &G, path: &[G::EdgeId]) -> bool {
    for window in path.windows(2) {
        let [e1, e2] = [window[0], window[1]];

        if graph.is_turn_restricted(e1, e2) {
            return false;
        }

        match graph.get_edge_end_vertex(e1) {
            Some(v) if !graph.vertex_exiting_edges(v).any(|(e, _)| e == e2) => return false,
            None => return false,
            Some(_) => (),
        };
    }

    true
}
