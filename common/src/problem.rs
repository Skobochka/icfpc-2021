use std::{
    fs,
    io,
    path::Path,
};

use geo::{
    algorithm::{
        contains::{
            Contains,
        },
        centroid::{
            Centroid,
        },
    },
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

#[derive(Debug)]
pub enum PoseValidationError {
    VericeCountMismatch,
    BrokenEdgesFound(Vec<Edge>),
    EdgesNotFitHole(Vec<Edge>),
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

    pub fn import_pose(&mut self, pose: Pose) {
        self.figure.vertices = pose.vertices;
    }

    pub fn hole_polygon(&self) -> geo::Polygon<i64> {
        geo::Polygon::new(self.hole.clone().into(), vec![])
    }

    pub fn hole_polygon_f64(&self) -> geo::Polygon<f64> {
        geo::Polygon::new(self.hole.clone().into(), vec![])
    }

    pub fn score_pose<'a>(&self, pose: &'a Pose) -> Result<i64, PoseValidationError> {
        // Check (a): connectivity. As our app does not change include edges in Pose,
        // we just check that the new Pose inclues the same number of vertices as the original
        if self.figure.vertices.len() != pose.vertices.len() {
            return Err(PoseValidationError::VericeCountMismatch)
        }

        // Check stretching
        let vertices_buf: Vec<(&Point, &Point)> = self.figure.vertices.iter().zip(pose.vertices.iter()).collect();
        let broken_edges: Vec<Edge> = self.figure.edges.iter().filter(|edge| {
            match edge {
                Edge(from_idx, to_idx) => {
                    let d_before = distance(vertices_buf[*from_idx].0, vertices_buf[*to_idx].0);
                    let d_after = distance(vertices_buf[*from_idx].1, vertices_buf[*to_idx].1);

                    ((d_after as f64) / (d_before as f64) - 1_f64).abs() > self.epsilon as f64 / 1000000_f64
                }
            }
        }).map(|item| item.clone()).collect();
        if broken_edges.len() > 0 {
            return Err(PoseValidationError::BrokenEdgesFound(broken_edges));
        }

        // Check hole
        let geo_hole = self.hole_polygon_f64();
        let edges_out_of_hole: Vec<Edge> = self.figure.edges.iter()
            .filter_map(|edge| {
                let Edge(from_idx, to_idx) = edge;
                let geo_edge = geo::Line {
                    start: geo::Coordinate::from(pose.vertices[*from_idx]),
                    end: geo::Coordinate::from(pose.vertices[*to_idx])
                };
                if geo_hole.contains(&geo_edge) {
                    None
                }
                else {
                    Some(edge)
                }
            }).map(|item| item.clone()).collect();
        if edges_out_of_hole.len() > 0 {
            return Err(PoseValidationError::EdgesNotFitHole(broken_edges));
        }

        Ok(0)
    }
}

#[derive(Debug)]
pub enum GeoExportError {
    NoCentroidBuilt,
}

#[derive(Debug)]
pub enum GeoImportError {
    PointsCountMismatch { expected: usize, provided: usize, },
    PointsInEdgeMismatch { expected: usize, provided: usize, },
}

pub struct GeoFigure {
    pub points: Vec<geo::Point<f64>>,
    pub centroid: geo::Point<f64>,
}

impl Figure {
    pub fn export_to_geo(&self) -> Result<GeoFigure, GeoExportError> {
        let mut geo_set = geo::GeometryCollection(Vec::with_capacity(self.vertices.len()));
        for &vertex in &self.vertices {
            geo_set.0.push(geo::Geometry::Point(geo::Point::new(vertex.0 as f64, vertex.1 as f64)));
        }

        let centroid = geo_set.centroid()
            .ok_or(GeoExportError::NoCentroidBuilt)?;

        Ok(GeoFigure {
            centroid,
            points: geo_set
                .into_iter()
                .flat_map(|g| if let geo::Geometry::Point(p) = g { Some(p) } else { None })
                .collect()
        })
    }

    pub fn import_from_geo(&mut self, geo_figure: Vec<geo::Point<f64>>) -> Result<(), GeoImportError> {
        if geo_figure.len() != self.vertices.len() {
            return Err(GeoImportError::PointsCountMismatch {
                expected: self.vertices.len(),
                provided: geo_figure.len(),
            });
        }

        for (point, vertex) in geo_figure.into_iter().zip(self.vertices.iter_mut()) {
            vertex.0 = point.x() as i64;
            vertex.1 = point.y() as i64;
        }

        Ok(())
    }
}

impl Pose {
    pub fn from_file<P>(filename: P) -> Result<Pose, FromFileError> where P: AsRef<Path> {
        let file = fs::File::open(filename)
            .map_err(FromFileError::OpenFile)?;
        let reader = io::BufReader::new(file);
        serde_json::from_reader(reader)
            .map_err(FromFileError::Deserialize)
    }

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

impl From<Point> for geo::Coordinate<f64> {
    fn from(point: Point) -> Self {
        geo::Coordinate::<f64> { x: point.0 as f64, y: point.1 as f64 }
    }
}

impl From<&Point> for geo::Coordinate<i64> {
    fn from(point: &Point) -> Self {
        geo::Coordinate::<i64> { x: point.0, y: point.1 }
    }
}

impl geo::algorithm::contains::Contains<Point> for geo::Polygon<i64> {
    fn contains(&self, point: &Point) -> bool {
        let geo_point = geo::Coordinate::from(point);

        // Present either inside polygon or on it's boundaries
        self.exterior().contains(&geo_point) || self.contains(&geo_point)
    }
}

pub fn distance(p: &Point, q: &Point) -> i64 {
    (p.0 - q.0) * (p.0 - q.0) + (p.1 - q.1) * (p.1 - q.1)
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

        // check corners
        assert_eq!(hole1.contains(&Point(0, 0)), true);
        assert_eq!(hole1.contains(&Point(10, 0)), true);
        assert_eq!(hole1.contains(&Point(10, 10)), true);
        assert_eq!(hole1.contains(&Point(0, 10)), true);

    }
}
