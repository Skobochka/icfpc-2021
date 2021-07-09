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

    pub fn hole_polygon(&self) -> geo::Polygon<i64> {
        geo::Polygon::new(self.hole.clone().into(), vec![])
    }
}

#[derive(Debug)]
pub enum GeoExportError {
    InvalidEdgeSourceIndex { edge: Edge, index: usize, },
    InvalidEdgeTargetIndex { edge: Edge, index: usize, },
}

#[derive(Debug)]
pub enum GeoImportError {
    EdgesMismatch { expected: usize, provided: usize, },
    PointsInEdgeMismatch { expected: usize, provided: usize, },
}

impl Figure {
    pub fn export_to_geo(&self) -> Result<geo::MultiLineString<f64>, GeoExportError> {

        let mut line_strings = Vec::with_capacity(self.edges.len());
        for &edge in &self.edges {
            let source_point = self.vertices.get(edge.0)
                .ok_or(GeoExportError::InvalidEdgeSourceIndex { edge, index: edge.0, })?;
            let target_point = self.vertices.get(edge.1)
                .ok_or(GeoExportError::InvalidEdgeTargetIndex { edge, index: edge.1, })?;
            let line_string = geo::LineString(vec![
                geo::Coordinate { x: source_point.0 as f64, y: source_point.1 as f64, },
                geo::Coordinate { x: target_point.0 as f64, y: target_point.1 as f64, },
            ]);
            line_strings.push(line_string);
        }

        Ok(geo::MultiLineString(line_strings))
    }

    pub fn import_from_geo(&mut self, geo_figure: geo::MultiLineString<f64>) -> Result<(), GeoImportError> {
        if geo_figure.0.len() != self.edges.len() {
            return Err(GeoImportError::EdgesMismatch {
                expected: self.edges.len(),
                provided: geo_figure.0.len(),
            });
        }

        for (multi_line, edge) in geo_figure.into_iter().zip(self.edges.iter()) {
            let mut points_iter = multi_line.points_iter();
            let source_point = points_iter.next()
                .ok_or(GeoImportError::PointsInEdgeMismatch {
                    expected: 2,
                    provided: 0,
                })?;
            let target_point = points_iter.next()
                .ok_or(GeoImportError::PointsInEdgeMismatch {
                    expected: 2,
                    provided: 1,
                })?;
            if !points_iter.next().is_none() {
                return Err(GeoImportError::PointsInEdgeMismatch {
                    expected: 2,
                    provided: 3,
                });
            }

            self.vertices[edge.0].0 = source_point.x() as i64;
            self.vertices[edge.0].1 = source_point.y() as i64;
            self.vertices[edge.1].0 = target_point.x() as i64;
            self.vertices[edge.1].1 = target_point.y() as i64;
        }

        Ok(())
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

impl From<Point> for geo::Point<i64> {
    fn from(point: Point) -> Self {
        geo::Point(geo::Coordinate::<i64> { x: point.0, y: point.1 })
    }
}

impl From<Point> for geo::Coordinate<i64> {
    fn from(point: Point) -> Self {
        geo::Coordinate::<i64> { x: point.0, y: point.1 }
    }
}

impl From<&Point> for geo::Coordinate<i64> {
    fn from(point: &Point) -> Self {
        geo::Coordinate::<i64> { x: point.0, y: point.1 }
    }
}

impl geo::algorithm::contains::Contains<Point> for geo::Polygon<i64> {
    fn contains(&self, point: &Point) -> bool {
        self.contains(&geo::Coordinate::from(point))
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    use geo::algorithm::contains::Contains;

    #[test]
    fn geo_type_conversion_test() {
        let problem = Problem {
            epsilon: 0,
            hole: vec![ Point(0, 0), Point(10, 0), Point(10, 10), Point(0, 10) ],
            figure: Figure {
                edges: vec![],
                vertices: vec![],
            },
        };

        let p = |x, y| geo::Coordinate { x: x, y: y };
        let ref_hole_polygon = geo::Polygon::new(
            geo::LineString(vec![p(0, 0), p(10, 0), p(10, 10), p(0, 10) ]),
            vec![]);
        assert_eq!(problem.hole_polygon(), ref_hole_polygon);
    }

    #[test]
    fn polygon_contains_test() {
        let problem1 = Problem {
            epsilon: 0,
            hole: vec![ Point(0, 0), Point(10, 0), Point(10, 10), Point(0, 10) ],
            figure: Figure {
                edges: vec![],
                vertices: vec![],
            },
        };

        let hole1 = problem1.hole_polygon();
        assert_eq!(hole1.contains(&geo::Point::from(Point(20, 20))), false);
        assert_eq!(hole1.contains(&geo::Point::from(Point(5, 5))), true);
        assert_eq!(hole1.contains(&Point(20, 20)), false);
        assert_eq!(hole1.contains(&Point(5, 5)), true);

    }
}
