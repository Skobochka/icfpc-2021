use std::{
    fs,
    io,
    path::Path,
    cmp,
    collections::HashSet,
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

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize, Debug)]
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

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug)]
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

#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize, Debug)]
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

#[derive(Debug, PartialEq)]
pub enum PoseValidationError {
    VerticeCountMismatch,
    BrokenEdgesFound { ratio_sum: f64, broken_edges_count: usize, },
    EdgesNotFitHole(usize),
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
        let score = self.score_pose(&self.hole_polygon_f64(), &pose);
        self.figure.vertices = pose.vertices;
        score
    }

    pub fn hole_polygon(&self) -> geo::Polygon<i64> {
        geo::Polygon::new(self.hole.clone().into(), vec![])
    }

    pub fn hole_polygon_f64(&self) -> geo::Polygon<f64> {
        geo::Polygon::new(self.hole.clone().into(), vec![])
    }

    pub fn score_vertices_check_count(&self,
                                      pose_vertices: &[Point],
                                      bonus: Option<PoseBonus>) -> Result<(), PoseValidationError> {
        // Check (a): connectivity. As our app does not change include edges in Pose,
        // we just check that the new Pose inclues the same number of vertices as the original
        if let Some(PoseBonus::BreakALeg { .. }) = bonus {
            unimplemented!("BREAK_A_LEG is not supported yet");
        }

        if self.figure.vertices.len() != pose_vertices.len() {
            return Err(PoseValidationError::VerticeCountMismatch)
        }
        Ok(())
    }

    pub fn score_vertices_check_stretching(&self,
                                           pose_vertices: &[Point],
                                           bonus: Option<PoseBonus>) -> Result<f64, PoseValidationError> {
        match bonus {
            Some(PoseBonus::Globalist { .. }) => {
                // Check stretching
                let mut ratio_sum = 0.0;
                for &Edge(from_idx, to_idx) in &self.figure.edges {
                    let d_before = distance(&self.figure.vertices[from_idx], &self.figure.vertices[to_idx]);
                    let d_after = distance(&pose_vertices[from_idx], &pose_vertices[to_idx]);

                    let ratio = ((d_after as f64) / (d_before as f64) - 1_f64).abs();
                    ratio_sum += ratio;
                }

                if ratio_sum > (self.figure.edges.len() as f64 * self.epsilon as f64) / 1000000_f64 {
                    return Err(PoseValidationError::BrokenEdgesFound { ratio_sum, broken_edges_count: 0, });
                }

                Ok(ratio_sum)
            }
            Some(PoseBonus::BreakALeg { .. }) => {
                unimplemented!("BREAK_A_LEG is not supported yet");
            }
            _ => {
                // Check stretching
                let mut broken_edges_count = 0;
                let mut allow_broken = match bonus {
                    Some(PoseBonus::Superflex { .. }) => 1,
                    _ => 0,
                };
                let mut ratio_sum = 0.0;
                for &Edge(from_idx, to_idx) in &self.figure.edges {
                    let d_before = distance(&self.figure.vertices[from_idx], &self.figure.vertices[to_idx]);
                    let d_after = distance(&pose_vertices[from_idx], &pose_vertices[to_idx]);

                    let ratio = ((d_after as f64) / (d_before as f64) - 1_f64).abs();
                    ratio_sum += ratio;
                    if ratio > self.epsilon as f64 / 1000000_f64 {
                        if allow_broken > 0 {
                            allow_broken -= 1;
                            continue;
                        }
                        // log::debug!("broken edge found. {:?}: d_before {}, d_after {}", edge, d_before, d_after);
                        broken_edges_count += 1;
                    }
                }
                if broken_edges_count > 0 {
                    return Err(PoseValidationError::BrokenEdgesFound { ratio_sum, broken_edges_count, });
                }

                Ok(ratio_sum)
            }
        }
    }

    pub fn score_vertices_check_hole<E>(
        &self,
        geo_hole: &E,
        pose_vertices: &[Point],
        bonus: Option<PoseBonus>,
    )
        -> Result<(), PoseValidationError>
    where E: InvalidEdge,
    {
        let mut edges_out_of_hole_count = 0;
        let mut outer_vertex: Option<usize> = None;
        for &Edge(from_idx, to_idx) in &self.figure.edges {
            let v_start = pose_vertices[from_idx];
            let v_end = pose_vertices[to_idx];
            if geo_hole.is_edge_invalid(v_start, v_end) {
                if let Some(PoseBonus::Wallhack { .. }) = bonus {
                    /* probably we can allow that for one vertice */
                    match outer_vertex {
                        None => {
                            let contains_start = !geo_hole.is_edge_invalid(v_start, v_start);
                            let contains_end = !geo_hole.is_edge_invalid(v_end, v_end);
                            if !contains_start && contains_end {
                                outer_vertex = Some(from_idx);
                                continue; // Ok, that's edge belongs to outer-point
                            }
                            else if contains_start && !contains_end {
                                outer_vertex = Some(to_idx);
                                continue; // Ok, that's edge belongs to outer-point
                            }
                        },
                        Some(idx) => {
                            if (idx == from_idx) || (idx == to_idx) {
                                continue; // Ok, that's edge belongs to outer-point
                            }
                        },
                    };
                }

                edges_out_of_hole_count += 1;
            }
        }

        if edges_out_of_hole_count > 0 {
            return Err(PoseValidationError::EdgesNotFitHole(edges_out_of_hole_count));
        }

        Ok(())
    }


    pub fn score_vertices<E>(
        &self,
        geo_hole: &E,
        pose_vertices: &[Point],
        bonus: Option<PoseBonus>,
    )
        -> Result<i64, PoseValidationError>
    where E: InvalidEdge,
    {
        self.score_vertices_check_count(pose_vertices, bonus)?;
        self.score_vertices_check_hole(geo_hole, pose_vertices, bonus)?;
        self.score_vertices_check_stretching(pose_vertices, bonus)?;

        let dislikes = self.hole.iter().map(|hole_vert| {
            pose_vertices.iter().map(|pose_vert| distance(hole_vert, pose_vert)).min().unwrap()
        }).sum();

        Ok(dislikes)
    }


    pub fn score_pose<E>(&self, geo_hole: &E, pose: &Pose) -> Result<i64, PoseValidationError> where E: InvalidEdge {
        self.score_vertices(geo_hole, &pose.vertices, pose.bonuses.as_ref().and_then(|bonuses| bonuses.first().cloned()))
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
            match self.score_vertices(&self.hole_polygon_f64(), &new_figure.vertices, None) {
                Err(PoseValidationError::BrokenEdgesFound { .. }) |
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
            match self.score_vertices(&self.hole_polygon_f64(), &new_figure.vertices, None) {
                Err(PoseValidationError::BrokenEdgesFound { .. }) |
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
            match self.score_vertices(&self.hole_polygon_f64(), &new_figure.vertices, None) {
                Err(PoseValidationError::BrokenEdgesFound { .. }) |
                Err(PoseValidationError::VerticeCountMismatch) => continue,
                _ => { angles.push(angle as f64); }
            }
        }
        angles
    }

    pub fn distance_cache(&self) -> Vec<i64> {
        let mut distances = vec![-1; self.figure.vertices.len() * self.figure.vertices.len()];

        for &Edge(from_idx, to_idx) in self.figure.edges.iter() {
            let distance = distance(&self.figure.vertices[from_idx], &self.figure.vertices[to_idx]);
            distances[from_idx * self.figure.vertices.len() + to_idx] = distance;
            distances[to_idx * self.figure.vertices.len() + from_idx] = distance;
        }

        distances
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

    pub fn bonus(&self) -> Option<PoseBonus> {
        self.bonuses.as_ref()
            .map_or(None, |bonus_vec| Some(bonus_vec[0]))
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

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct BoundingBox(pub Point, pub Point);

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct SquareRing(pub BoundingBox, pub BoundingBox);

impl SquareRing {
    pub fn point_set(&self) -> HashSet<Point> {
        let outer_box = &self.0;
        let inner_box = &self.1;

        let capacity = ((outer_box.0.0 - outer_box.1.0).abs() * (outer_box.0.1 - outer_box.1.1).abs()) as usize;
        let mut set = HashSet::with_capacity(capacity);

        for x in cmp::max(0, cmp::min(outer_box.0.0, outer_box.1.0))..=cmp::max(outer_box.0.0, outer_box.1.0) {
            for y in cmp::max(0, cmp::min(outer_box.0.1, outer_box.1.1))..=cmp::max(outer_box.0.1, outer_box.1.1) {
                if x > cmp::min(inner_box.0.0, inner_box.1.0)
                    && x < cmp::max(inner_box.0.0, inner_box.1.0)
                    && y > cmp::min(inner_box.0.1, inner_box.1.1)
                    && y < cmp::max(inner_box.0.1, inner_box.1.1) {
                    continue;
                }
                set.insert(Point(x, y));
            }
        }

        set
    }

    pub fn point_set_within_hole(&self, hole: &Vec<Point>) -> HashSet<Point> {
        let geo_hole = geo::Polygon::new(hole.clone().into(), vec![]);

        let outer_box = &self.0;
        let inner_box = &self.1;

        let capacity = ((outer_box.0.0 - outer_box.1.0).abs() * (outer_box.0.1 - outer_box.1.1).abs()) as usize;
        let mut set = HashSet::with_capacity(capacity);

        for x in cmp::max(0, cmp::min(outer_box.0.0, outer_box.1.0))..=cmp::max(outer_box.0.0, outer_box.1.0) {
            for y in cmp::max(0, cmp::min(outer_box.0.1, outer_box.1.1))..=cmp::max(outer_box.0.1, outer_box.1.1) {
                if x > cmp::min(inner_box.0.0, inner_box.1.0)
                    && x < cmp::max(inner_box.0.0, inner_box.1.0)
                    && y > cmp::min(inner_box.0.1, inner_box.1.1)
                    && y < cmp::max(inner_box.0.1, inner_box.1.1) {
                        continue;
                    }

                let point = Point(x, y);

                if !geo_hole.contains(&point) {
                    continue;
                }

                set.insert(point);
            }
        }

        set
    }
}

pub trait InvalidEdge {
    fn is_edge_invalid(&self, edge_from: Point, edge_to: Point) -> bool;
}

impl InvalidEdge for geo::Polygon<f64> {
    fn is_edge_invalid(&self, edge_from: Point, edge_to: Point) -> bool {
        let geo_start = geo::Coordinate::from(edge_from);
        let geo_end = geo::Coordinate::from(edge_to);
        let geo_edge = geo::Line { start: geo_start, end: geo_end, };

        if self.contains(&geo_edge) || self.exterior().contains(&geo_edge) {
            false
        } else {
            true
        }
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

        assert_eq!(problem.score_vertices_check_count(&vec![], None),
                   Err(PoseValidationError::VerticeCountMismatch));
        assert_eq!(problem.score_vertices_check_count(&vec![Point(15,21), Point(34,0), Point(0,45)], None),
                   Err(PoseValidationError::VerticeCountMismatch));
        assert_eq!(problem.score_vertices_check_count(&vec![Point(15,21), Point(34,0), Point(0,45), Point(19,24)],  None),
                   Ok(()));
        assert_eq!(problem.score_vertices_check_count(&vec![Point(15,21), Point(34,0), Point(0,45), Point(0,45), Point(0,45)], None),
                   Err(PoseValidationError::VerticeCountMismatch));
    }

    #[test]
    fn score_vertices_check_stretching() {
        let problem: Problem = serde_json::from_str(PROBLEM_13_JSON).unwrap();
        assert!(problem.score_vertices_check_stretching(&vec![Point(15,21), Point(34,0), Point(0,45), Point(19,24)], None).is_ok());
        assert!(problem.score_vertices_check_stretching(&vec![Point(15,0), Point(34,0), Point(0,45), Point(19,24)], None).is_err());
        // TODO: add more tests
    }

    #[test]
    fn score_vertices_check_hole() {
        let problem: Problem = serde_json::from_str(PROBLEM_13_JSON).unwrap();
        assert!(
            problem.score_vertices_check_hole(
                &problem.hole_polygon_f64(),
                &vec![Point(15,21), Point(34,0), Point(0,45), Point(19,24)], None
            ).is_err(),
        );
        assert!(
            problem.score_vertices_check_hole(
                &problem.hole_polygon_f64(),
                &vec![Point(20,0), Point(0,20), Point(20,40), Point(19,24)], None
            ).is_ok(),
        );
    }

    #[test]
    fn score_vertices() {
        let problem: Problem = serde_json::from_str(PROBLEM_13_JSON).unwrap();
        let pose: Pose = serde_json::from_str(POSE_13_SCORE_0_JSON).unwrap();

        assert!(
            problem.score_vertices(
                &problem.hole_polygon_f64(),
                &problem.export_pose().vertices,
                None
            ).is_err());
        assert_eq!(
            problem.score_vertices(
                &problem.hole_polygon_f64(),
                &pose.vertices,
                None,
            ),
            Ok(0),
        );
    }

    #[test]
    fn score_vertices_check_stretching_broken_pose_task_50() {
        let problem: Problem = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"GLOBALIST","problem":9,"position":[0,8]},{"bonus":"GLOBALIST","problem":70,"position":[33,48]},{"bonus":"WALLHACK","problem":91,"position":[42,33]}],"hole":[[21,35],[0,22],[26,14],[0,8],[15,0],[45,0],[52,12],[52,48],[51,60],[49,86],[38,73],[17,85],[31,92],[24,103],[2,103],[0,85],[0,52],[8,41]],"epsilon":17065,"figure":{"edges":[[0,3],[0,4],[1,2],[1,5],[2,3],[2,5],[3,6],[3,7],[4,6],[5,8],[6,7],[7,8]],"vertices":[[0,75],[7,0],[10,26],[20,55],[20,95],[29,13],[40,75],[46,52],[48,28]]}}"#,
        ).unwrap();
        let pose: Pose = serde_json::from_str(
            r#"{"vertices":[[18,81],[40,46],[28,23],[22,53],[38,61],[51,23],[42,33],[48,56],[32,38]],"bonuses":null}"#,
        ).unwrap();

        assert!(matches!(
            problem.score_vertices_check_stretching(&pose.vertices, None),
            Err(PoseValidationError::BrokenEdgesFound { broken_edges_count: 1, .. }),
        ));
        assert!(matches!(
            problem.score_vertices(&problem.hole_polygon_f64(), &pose.vertices, None),
            Err(PoseValidationError::BrokenEdgesFound { broken_edges_count: 1, .. }),
        ));
    }

    #[test]
    fn score_vertices_check_hole_with_wallhack() {
        let problem: Problem = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"GLOBALIST","problem":9,"position":[0,8]},{"bonus":"GLOBALIST","problem":70,"position":[33,48]},{"bonus":"WALLHACK","problem":91,"position":[42,33]}],"hole":[[21,35],[0,22],[26,14],[0,8],[15,0],[45,0],[52,12],[52,48],[51,60],[49,86],[38,73],[17,85],[31,92],[24,103],[2,103],[0,85],[0,52],[8,41]],"epsilon":17065,"figure":{"edges":[[0,1],[0,2],[1,2]],"vertices":[[0,75],[7,0],[10,26]]}}"#,
        ).unwrap();
        let pose_vertices = vec![
            Point(32, 47), Point(34, 47), Point(33, 48),
        ];
        assert_eq!(problem.score_vertices_check_hole(&problem.hole_polygon_f64(), &pose_vertices, None), Ok(()));
        assert_eq!(problem.score_vertices_check_hole(
            &problem.hole_polygon_f64(), &pose_vertices, Some(PoseBonus::Wallhack { problem: ProblemId(0), })), Ok(())
        );

        let pose_vertices = vec![
            Point(32, 47), Point(0, 48), Point(33, 48),
        ];
        assert!(problem.score_vertices_check_hole(&problem.hole_polygon_f64(), &pose_vertices, None).is_err());
        assert_eq!(problem.score_vertices_check_hole(
            &problem.hole_polygon_f64(), &pose_vertices, Some(PoseBonus::Wallhack { problem: ProblemId(0), })), Ok(())
        );

        let pose_vertices = vec![
            Point(0, 48), Point(34, 47), Point(1000, 48),
        ];
        assert!(problem.score_vertices_check_hole(&problem.hole_polygon_f64(), &pose_vertices, None).is_err());
        assert!(problem.score_vertices_check_hole(
            &problem.hole_polygon_f64(), &pose_vertices, Some(PoseBonus::Wallhack { problem: ProblemId(0), })
        ).is_err());
    }

    #[test]
    fn bounding_ring_box() {
        let outer = BoundingBox(Point(0,0), Point(4,4));
        let inner = BoundingBox(Point(1,1), Point(3,3));

        let ring = SquareRing(outer, inner);

        let right = [
            Point(0,0), Point(1,0), Point(2,0), Point(3,0), Point(4,0),
            Point(0,1), Point(1,1), Point(2,1), Point(3,1), Point(4,1),
            Point(0,2), Point(1,2),             Point(3,2), Point(4,2),
            Point(0,3), Point(1,3), Point(2,3), Point(3,3), Point(4,3),
            Point(0,4), Point(1,4), Point(2,4), Point(3,4), Point(4,4),
            ];
        assert_eq!(ring.point_set(), right.iter().cloned().collect());
    }

    #[test]
    fn score_vertice_broken_wallhack_tasks() {
        let problem_47: Problem = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"BREAK_A_LEG","problem":43,"position":[59,26]},{"bonus":"SUPERFLEX","problem":58,"position":[54,9]},{"bonus":"SUPERFLEX","problem":27,"position":[21,18]}],"hole":[[6,14],[36,19],[40,17],[69,0],[79,21],[41,36],[36,33],[16,44],[7,34],[0,28]],"epsilon":41323,"figure":{"edges":[[0,3],[3,7],[7,4],[4,0],[7,10],[10,9],[9,8],[8,7],[4,5],[5,8],[8,6],[6,1],[1,5],[5,2],[2,4]],"vertices":[[0,11],[1,85],[8,56],[11,0],[14,45],[14,59],[14,88],[30,37],[30,56],[56,85],[67,64]]}}"#,
        ).unwrap();
        let pose_47: Pose = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"WALLHACK","problem":55}],"vertices":[[22,27],[33,24],[53,15],[12,15],[59,26],[59,12],[21,18],[54,9],[56,28],[19,17],[21,41]]}"#,
        ).unwrap();

        assert!(problem_47.score_pose(&problem_47.hole_polygon_f64(), &pose_47).is_err());

        let problem_92: Problem = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"BREAK_A_LEG","problem":99,"position":[96,12]},{"bonus":"SUPERFLEX","problem":30,"position":[30,24]},{"bonus":"GLOBALIST","problem":7,"position":[84,72]}],"hole":[[5,29],[4,18],[0,10],[8,5],[16,13],[21,0],[33,4],[33,44],[47,16],[59,4],[68,3],[78,3],[115,8],[72,18],[83,29],[92,41],[109,42],[101,52],[97,65],[115,59],[115,69],[108,83],[110,100],[102,115],[100,93],[99,81],[85,90],[92,102],[74,98],[70,115],[61,91],[61,80],[77,85],[65,68],[73,58],[91,50],[71,44],[59,55],[55,63],[51,72],[47,83],[41,95],[25,100],[44,108],[26,111],[15,113],[5,115],[2,106],[14,103],[12,92],[24,87],[36,73],[56,34],[24,66],[22,54],[0,82],[4,59],[2,49],[1,40],[19,37]],"epsilon":160000,"figure":{"edges":[[2,5],[5,4],[4,1],[1,0],[0,8],[8,3],[3,7],[7,11],[11,13],[13,12],[12,18],[18,19],[19,14],[14,15],[15,17],[17,16],[16,10],[10,6],[6,2],[8,12],[7,9],[9,3],[8,9],[9,12],[13,9],[9,11],[4,8],[12,14],[5,10],[10,15]],"vertices":[[30,40],[30,50],[40,105],[50,25],[50,45],[50,75],[50,105],[55,15],[55,35],[60,25],[60,80],[65,15],[65,35],[70,25],[70,45],[70,75],[70,105],[80,105],[90,40],[90,50]]}}"#,
        ).unwrap();
        let pose_92: Pose = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"WALLHACK","problem":119}],"vertices":[[81,43],[74,35],[15,31],[50,26],[52,33],[35,55],[17,22],[61,27],[61,28],[53,35],[25,49],[52,24],[51,24],[43,32],[50,13],[31,39],[8,70],[10,60],[77,23],[67,22]]}"#,
        ).unwrap();

        assert!(problem_92.score_pose(&problem_92.hole_polygon_f64(), &pose_92).is_err());
    }

    #[test]
    fn score_vertice_wallhack_synthetic() {
        let problem_2_outer: Problem = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"GLOBALIST","problem":46,"position":[20,20]},{"bonus":"BREAK_A_LEG","problem":88,"position":[30,30]},{"bonus":"GLOBALIST","problem":20,"position":[30,12]}],"hole":[[20,0],[40,20],[20,40],[0,20]],"epsilon":2494,"figure":{"edges":[[0,1],[0,2],[1,3],[2,3]],"vertices":[[15,21],[34,0],[0,45],[19,24]]}}"#,
        ).unwrap();
        let pose_2_outer: Pose = serde_json::from_str(
            r#"{"vertices":[[15,21],[34,0],[0,45],[19,24]],"bonuses":[{"bonus":"WALLHACK","problem":999}]}"#,
        ).unwrap();

        assert!(problem_2_outer.score_pose(&problem_2_outer.hole_polygon_f64(), &pose_2_outer).is_err());

        let problem_1_outer: Problem = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"GLOBALIST","problem":46,"position":[20,20]},{"bonus":"BREAK_A_LEG","problem":88,"position":[30,30]},{"bonus":"GLOBALIST","problem":20,"position":[30,12]}],"hole":[[20,0],[40,20],[20,40],[0,20]],"epsilon":2494,"figure":{"edges":[[0,1],[0,2],[1,3],[2,3]],"vertices":[[15,21],[34,0],[15,15],[19,24]]}}"#,
        ).unwrap();

        let pose_1_outer_no_bonus: Pose = serde_json::from_str(
            r#"{"vertices":[[15,21],[34,0],[15,15],[19,24]],"bonuses":null}"#,
        ).unwrap();
        assert!(problem_1_outer.score_pose(&problem_1_outer.hole_polygon_f64(), &pose_1_outer_no_bonus).is_err());

        let pose_1_outer_wallhack: Pose = serde_json::from_str(
            r#"{"vertices":[[15,21],[34,0],[15,15],[19,24]],"bonuses":[{"bonus":"WALLHACK","problem":999}]}"#,
        ).unwrap();
        assert!(problem_1_outer.score_pose(&problem_1_outer.hole_polygon_f64(), &pose_1_outer_wallhack).is_ok());

        let problem_2_outer_same: Problem = serde_json::from_str(
            r#"{"bonuses":[{"bonus":"GLOBALIST","problem":46,"position":[20,20]},{"bonus":"BREAK_A_LEG","problem":88,"position":[30,30]},{"bonus":"GLOBALIST","problem":20,"position":[30,12]}],"hole":[[20,0],[40,20],[20,40],[0,20]],"epsilon":2494,"figure":{"edges":[[0,1],[0,2],[1,3],[2,3]],"vertices":[[15,21],[34,0],[34,0],[19,24]]}}"#,
        ).unwrap();
        let pose_2_outer_same_nobonus: Pose = serde_json::from_str(
            r#"{"vertices":[[15,21],[34,0],[34,0],[19,24]],"bonuses":null}"#,
        ).unwrap();
        assert!(problem_2_outer_same.score_pose(&problem_2_outer_same.hole_polygon_f64(), &pose_2_outer_same_nobonus).is_err());
        let pose_2_outer_same_wallhack: Pose = serde_json::from_str(
            r#"{"vertices":[[15,21],[34,0],[34,0],[19,24]],"bonuses":[{"bonus":"WALLHACK","problem":999}]}"#,
        ).unwrap();
        assert!(problem_2_outer_same.score_pose(&problem_2_outer_same.hole_polygon_f64(), &pose_2_outer_same_wallhack ).is_err());
    }
}
