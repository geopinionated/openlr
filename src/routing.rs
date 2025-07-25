mod dijkstra; // TODO: rename shortest_path.rs and move to decoder module
mod route;

pub use dijkstra::{ShortestPathConfig, shortest_path};
pub use route::{Path, Route, Routes};
