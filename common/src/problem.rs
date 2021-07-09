use std::{
    fs,
    io,
    path::Path,
};

use serde_derive::{
    Serialize,
    Deserialize,
};

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
pub struct Point(pub i64, pub i64);

#[derive(Clone, Copy, Serialize, Deserialize, Debug)]
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

#[derive(Serialize, Deserialize, Debug)]
pub struct Pose {
    pub vertices: Vec<Point>,
}

#[derive(Debug)]
pub enum FromFileError {
    OpenFile(io::Error),
    Deserialize(serde_json::Error),
}

#[derive(Debug)]
pub enum WriteFileError {
    CreateFile(io::Error),
    Serialize(serde_json::Error),
}

impl Problem {
    pub fn from_file<P>(filename: P) -> Result<Problem, FromFileError> where P: AsRef<Path> {
        let file = fs::File::open(filename)
            .map_err(FromFileError::OpenFile)?;
        let reader = io::BufReader::new(file);
        serde_json::from_reader(reader)
            .map_err(FromFileError::Deserialize)
    }

    pub fn export_pose(&self) -> Pose {
        Pose {
            vertices: self.figure.vertices.clone(),
        }
    }
}

impl Pose {
    pub fn write_to_file<P>(&self, filename: P) -> Result<(), WriteFileError> where P: AsRef<Path> {
        let file = fs::File::create(filename)
            .map_err(WriteFileError::CreateFile)?;
        let writer = io::BufWriter::new(file);
        serde_json::to_writer(writer, self)
            .map_err(WriteFileError::Serialize)
    }
}
