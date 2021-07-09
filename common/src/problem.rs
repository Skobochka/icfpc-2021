use std::{
    fs,
    io,
    path::Path,
};

use serde_derive::{
    Serialize,
    Deserialize,
};

#[derive(Serialize, Deserialize, Debug)]
pub struct Point(pub i64, pub i64);

#[derive(Serialize, Deserialize, Debug)]
pub struct Edge(pub usize, pub usize);

#[derive(Serialize, Deserialize, Debug)]
pub struct Problem {
    pub hole: Vec<Point>,
    pub figure: Figure,
    pub epsilon: u64,
}

#[derive(Serialize, Deserialize, Debug)]
pub struct Figure {
    pub edges: Vec<Edge>,
    pub vertices: Vec<Point>,
}

#[derive(Debug)]
pub enum FromFileError {
    OpenFile(io::Error),
    Deserialize(serde_json::Error),
}

impl Problem {
    pub fn from_file<P>(filename: P) -> Result<Problem, FromFileError> where P: AsRef<Path> {
        let file = fs::File::open(filename)
            .map_err(FromFileError::OpenFile)?;
        let reader = io::BufReader::new(file);
        serde_json::from_reader(reader)
            .map_err(FromFileError::Deserialize)
    }
}
