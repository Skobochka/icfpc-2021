use std::{
    fs,
    io,
    path::Path,
};

use geo::{
    algorithm::{
        rotate::{
            RotatePoint,
        },
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

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub struct Point(pub i64, pub i64);

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub struct Edge(pub usize, pub usize);

#[derive(Clone, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub struct Problem {
    pub hole: Vec<Point>,
    pub figure: Figure,
    pub epsilon: u64,
    pub bonuses: Option<Vec<ProblemBonus>>,
}

#[derive(Clone, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub struct Figure {
    pub edges: Vec<Edge>,
    pub vertices: Vec<Point>,
}

#[derive(Clone, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub struct Pose {
    pub vertices: Vec<Point>,
    pub bonuses: Option<Vec<PoseBonus>>,
}

#[derive(Clone, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub struct ProblemBonus {
    pub position: Point,
    pub bonus: ProblemBonusType,
    pub problem: ProblemId,
}

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub enum ProblemBonusType {
    #[serde(rename = "BREAK_A_LEG")]
    BreakALeg,
    #[serde(rename = "GLOBALIST")]
    Globalist,
    #[serde(rename = "WALLHACK")]
    Wallhack,
    #[serde(rename = "SUPERFLEX")]
    Superflex,
}

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug)]
pub struct ProblemId(pub usize);

#[derive(Clone, PartialEq, Eq, Serialize, Deserialize, Debug)]
#[serde(tag = "bonus")]
pub enum PoseBonus {
    #[serde(rename = "BREAK_A_LEG")]
    BreakALeg {
        problem: ProblemId,
        edge: Edge,
    },
    #[serde(rename = "GLOBALIST")]
    Globalist {
        problem: ProblemId,
    },
    #[serde(rename = "WALLHACK")]
    Wallhack {
        problem: ProblemId,
    },
    #[serde(rename = "SUPERFLEX")]
    Superflex {
        problem: ProblemId,
    },
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

#[derive(Debug, PartialEq, Eq)]
pub enum PoseValidationError {
    VerticeCountMismatch,
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
            bonuses: None,
        }
    }

    pub fn import_pose(&mut self, pose: Pose) -> Result<i64, PoseValidationError> {
        let score = self.score_pose(&pose);
        self.figure.vertices = pose.vertices;
        score
    }

    pub fn hole_polygon(&self) -> geo::Polygon<i64> {
        geo::Polygon::new(self.hole.clone().into(), vec![])
    }

    pub fn hole_polygon_f64(&self) -> geo::Polygon<f64> {
        geo::Polygon::new(self.hole.clone().into(), vec![])
    }

    pub fn score_vertices_check_count(&self, pose_vertices: &[Point]) -> Result<(), PoseValidationError> {
        // Check (a): connectivity. As our app does not change include edges in Pose,
        // we just check that the new Pose inclues the same number of vertices as the original
        if self.figure.vertices.len() != pose_vertices.len() {
            return Err(PoseValidationError::VerticeCountMismatch)
        }
        Ok(())
    }

    pub fn score_vertices_check_stretching(&self, pose_vertices: &[Point]) -> Result<(), PoseValidationError> {
        // Check stretching
        let mut broken_edges = Vec::new();
        for &Edge(from_idx, to_idx) in &self.figure.edges {
            let d_before = distance(&self.figure.vertices[from_idx], &self.figure.vertices[to_idx]);
            let d_after = distance(&pose_vertices[from_idx], &pose_vertices[to_idx]);
            if ((d_after as f64) / (d_before as f64) - 1_f64).abs() > self.epsilon as f64 / 1000000_f64 {
                // log::debug!("broken edge found. {:?}: d_before {}, d_after {}", edge, d_before, d_after);
                broken_edges.push(Edge(from_idx, to_idx));
            }
        }
        if !broken_edges.is_empty() {
            return Err(PoseValidationError::BrokenEdgesFound(broken_edges));
        }

        Ok(())
    }

    pub fn score_vertices_check_hole(&self, pose_vertices: &[Point]) -> Result<(), PoseValidationError> {
        let geo_hole = self.hole_polygon_f64();
        let mut edges_out_of_hole = Vec::new();
        for &Edge(from_idx, to_idx) in &self.figure.edges {
            let geo_edge = geo::Line {
                start: geo::Coordinate::from(pose_vertices[from_idx]),
                end: geo::Coordinate::from(pose_vertices[to_idx])
            };
            if geo_hole.contains(&geo_edge) || geo_hole.exterior().contains(&geo_edge) {
                // ok
            }
            else {
                edges_out_of_hole.push(Edge(from_idx, to_idx));
            }
        }
        if !edges_out_of_hole.is_empty() {
            return Err(PoseValidationError::EdgesNotFitHole(edges_out_of_hole));
        }

        Ok(())
    }


    pub fn score_vertices(&self, pose_vertices: &[Point]) -> Result<i64, PoseValidationError> {
        self.score_vertices_check_count(pose_vertices)?;
        self.score_vertices_check_stretching(pose_vertices)?;
        self.score_vertices_check_hole(pose_vertices)?;


        let dislikes = self.hole.iter().map(|hole_vert| {
            pose_vertices.iter().map(|pose_vert| distance(hole_vert, pose_vert)).min().unwrap()
        }).sum();

        Ok(dislikes)
    }


    pub fn score_pose(&self, pose: &Pose) -> Result<i64, PoseValidationError> {
        self.score_vertices(&pose.vertices)
    }

    pub fn possible_rotations(&self) -> Vec<f64> {
        let mut angles = vec![];
        let geo_figure = self.figure.export_to_geo().unwrap();
        for angle in 1..360 {
            // log::debug!("checking angle {}", angle);
            let mut new_geo_figure = geo_figure.clone();
            new_geo_figure.rotate_around_centroid_mut(angle as f64);
            let mut new_figure = self.figure.clone();
            new_figure.import_from_geo(new_geo_figure.points).unwrap();
            match self.score_vertices(&new_figure.vertices) {
                Err(PoseValidationError::BrokenEdgesFound(_)) |
                Err(PoseValidationError::VerticeCountMismatch) => continue,
                _ => { angles.push(angle as f64); }
            }
        }
        angles
    }

    pub fn possible_rotations_around_point(&self, point: &Point) -> Vec<f64> {
        let mut angles = vec![];
        let geo_point = geo::Point::from(point);
        let geo_figure = self.figure.export_to_geo().unwrap();
        for angle in 1..360 {
            // log::debug!("(for point) checking angle {}", angle);
            let mut new_geo_figure = geo_figure.clone();
            new_geo_figure.rotate_around_point_mut(angle as f64, geo_point);
            let mut new_figure = self.figure.clone();
            new_figure.import_from_geo(new_geo_figure.points).unwrap();
            match self.score_vertices(&new_figure.vertices) {
                Err(PoseValidationError::BrokenEdgesFound(_)) |
                Err(PoseValidationError::VerticeCountMismatch) => continue,
                _ => { angles.push(angle as f64); }
            }
        }
        angles
    }

    pub fn possible_rotations_for_vertices(&self, vertices: &Vec<Point>) -> Vec<f64> {
        let mut angles = vec![];
        let mut figure = self.figure.clone();
        figure.vertices = vertices.clone();
        let geo_figure = figure.export_to_geo().unwrap();
        for angle in 1..360 {
            // log::debug!("checking angle {}", angle);
            let mut new_geo_figure = geo_figure.clone();
            new_geo_figure.rotate_around_centroid_mut(angle as f64);
            let mut new_figure = self.figure.clone();
            new_figure.import_from_geo(new_geo_figure.points).unwrap();
            match self.score_vertices(&new_figure.vertices) {
                Err(PoseValidationError::BrokenEdgesFound(_)) |
                Err(PoseValidationError::VerticeCountMismatch) => continue,
                _ => { angles.push(angle as f64); }
            }
        }
        angles
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

#[derive(Clone, Debug)]
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
            vertex.0 = point.x().round() as i64;
            vertex.1 = point.y().round() as i64;
        }

        Ok(())
    }

}

impl GeoFigure {
    pub fn rotate_around_centroid_mut(&mut self, angle: f64) {
        let rotated_points: Vec<_> = self
            .points
            .iter()
            .map(|p| p.rotate_around_point(angle, self.centroid))
            .collect();
        self.points = rotated_points;
    }

    pub fn rotate_around_point_mut(&mut self, angle: f64, point: geo::Point<f64>) {
        let rotated_points: Vec<_> = self
            .points
            .iter()
            .map(|p| {
                if *p == point {
                    p.clone()
                } else {
                    p.rotate_around_point(angle, point)
                }
            })
            .collect();
        self.points = rotated_points;
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

impl From<Point> for geo::Point<f64> {
    fn from(point: Point) -> Self {
        geo::Point(geo::Coordinate::<f64> { x: point.0 as f64, y: point.1 as f64 })
    }
}

impl From<&Point> for geo::Point<f64> {
    fn from(point: &Point) -> Self {
        geo::Point(geo::Coordinate::<f64> { x: point.0 as f64, y: point.1 as f64 })
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
            bonuses: None,
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
            bonuses: None,
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

    #[test]
    fn deserialize_problem_bonus_break_a_leg() {
        let data = r#"{"bonus":"BREAK_A_LEG","problem":28,"position":[43,44]}"#;
        assert_eq!(
            serde_json::from_str::<ProblemBonus>(data).unwrap(),
            ProblemBonus {
                position: Point(43, 44),
                bonus: ProblemBonusType::BreakALeg,
                problem: ProblemId(28),
            },
        );
    }

    #[test]
    fn deserialize_problem_bonus_globalist() {
        let data = r#"{"bonus":"GLOBALIST","problem":70,"position":[106,85]}"#;
        assert_eq!(
            serde_json::from_str::<ProblemBonus>(data).unwrap(),
            ProblemBonus {
                position: Point(106, 85),
                bonus: ProblemBonusType::Globalist,
                problem: ProblemId(70),
            },
        );
    }

    #[test]
    fn deserialize_pose_bonus_globalist() {
        let data = r#"{"bonus":"GLOBALIST","problem":70}"#;
        assert_eq!(
            serde_json::from_str::<PoseBonus>(data).unwrap(),
            PoseBonus::Globalist {
                problem: ProblemId(70),
            },
        );
    }

    #[test]
    fn deserialize_pose_bonus_break_a_leg() {
        let data = r#"{"bonus":"BREAK_A_LEG","problem":70,"edge":[0, 2]}"#;
        assert_eq!(
            serde_json::from_str::<PoseBonus>(data).unwrap(),
            PoseBonus::BreakALeg {
                problem: ProblemId(70),
                edge: Edge(0, 2),
            },
        );
    }

    const PROBLEM_13_JSON: &str = r#"{"bonuses":[{"bonus":"GLOBALIST","problem":46,"position":[20,20]},{"bonus":"BREAK_A_LEG","problem":88,"position":[30,30]}],"hole":[[20,0],[40,20],[20,40],[0,20]],"epsilon":2494,"figure":{"edges":[[0,1],[0,2],[1,3],[2,3]],"vertices":[[15,21],[34,0],[0,45],[19,24]]}}"#;

    const POSE_13_SCORE_0_JSON: &str = r#"{"vertices":[[20,0],[40,20],[0,20],[20,40]]}"#;

    #[test]
    fn score_vertices_check_count() {
        let problem: Problem = serde_json::from_str(PROBLEM_13_JSON).unwrap();

        assert_eq!(problem.score_vertices_check_count(&vec![]), Err(PoseValidationError::VerticeCountMismatch));
        assert_eq!(problem.score_vertices_check_count(&vec![Point(15,21), Point(34,0), Point(0,45)]), Err(PoseValidationError::VerticeCountMismatch));
        assert_eq!(problem.score_vertices_check_count(&vec![Point(15,21), Point(34,0), Point(0,45), Point(19,24)]), Ok(()));
        assert_eq!(problem.score_vertices_check_count(&vec![Point(15,21), Point(34,0), Point(0,45), Point(0,45), Point(0,45)]), Err(PoseValidationError::VerticeCountMismatch));
    }

    #[test]
    fn score_vertices_check_stretching() {
        let problem: Problem = serde_json::from_str(PROBLEM_13_JSON).unwrap();
        assert_eq!(problem.score_vertices_check_stretching(&vec![Point(15,21), Point(34,0), Point(0,45), Point(19,24)]), Ok(()));
        assert!(problem.score_vertices_check_stretching(&vec![Point(15,0), Point(34,0), Point(0,45), Point(19,24)]).is_err());
        // TODO: add more tests
    }

    #[test]
    fn score_vertices_check_hole() {
        let problem: Problem = serde_json::from_str(PROBLEM_13_JSON).unwrap();
        assert!(problem.score_vertices_check_hole(&vec![Point(15,21), Point(34,0), Point(0,45), Point(19,24)]).is_err());
        assert_eq!(problem.score_vertices_check_hole(&vec![Point(20,0), Point(0,20), Point(20,40), Point(19,24)]), Ok(()));
        // TODO: add more tests
    }

    #[test]
    fn score_vertices() {
        let problem: Problem = serde_json::from_str(PROBLEM_13_JSON).unwrap();
        let pose: Pose = serde_json::from_str(POSE_13_SCORE_0_JSON).unwrap();

        assert!(problem.score_vertices(&problem.export_pose().vertices).is_err());
        assert_eq!(problem.score_vertices(&pose.vertices), Ok(0));
        // TODO: add more tests
    }
}
