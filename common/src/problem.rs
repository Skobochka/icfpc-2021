use serde_derive::{
    Serialize,
    Deserialize,
};

#[derive(Serialize, Deserialize)]
pub struct Point(pub i64, pub i64);

#[derive(Serialize, Deserialize)]
pub struct Edge(pub usize, pub usize);

#[derive(Serialize, Deserialize)]
pub struct Problem {
    pub hole: Vec<Point>,
    pub figure: Figure,
    pub epsilon: u64,
}

#[derive(Serialize, Deserialize)]
pub struct Figure {
    pub edges: Vec<Edge>,
    pub vertices: Vec<Point>,
}
