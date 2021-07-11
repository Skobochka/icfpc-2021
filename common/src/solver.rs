use geo::algorithm::contains::Contains;

use crate::{
    problem,
};

pub mod simulated_annealing;
pub mod bruteforce;
pub mod bruteforce_hole;

#[allow(dead_code)]
pub struct Solver {
    hole_mask: bit_vec::BitVec,
    field_min: problem::Point,
    field_max: problem::Point,
    field_area: usize,
    field_width: i64,
    field_height: i64,
    problem: problem::Problem,
    pose: problem::Pose,
    pose_score: i64,
}

#[derive(Debug)]
pub enum CreateError {
    NoPointsInHole,
    NoPointsInFigure,
}

impl Solver {
    pub fn new(problem: &problem::Problem, pose: Option<problem::Pose>) -> Result<Solver, CreateError> {
        if problem.hole.is_empty() {
            return Err(CreateError::NoPointsInHole);
        }
        if problem.figure.vertices.is_empty() {
            return Err(CreateError::NoPointsInFigure);
        }

        let field_min = problem::Point(
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.0)
                .min()
                .unwrap(),
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.1)
                .min()
                .unwrap(),
        );
        let field_max = problem::Point(
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.0)
                .max()
                .unwrap(),
            problem.hole.iter()
                .chain(problem.figure.vertices.iter())
                .map(|p| p.1)
                .max()
                .unwrap(),
        );
        let field_width = field_max.0 - field_min.0 + 1;
        let field_height = field_max.1 - field_min.1 + 1;
        let field_area = (field_width * field_height) as usize;
        let mut hole_mask = bit_vec::BitVec::from_elem(field_area, false);

        let hole_poly = problem.hole_polygon();
        for y in field_min.1 ..= field_max.1 {
            for x in field_min.0 ..= field_max.0 {
                if hole_poly.contains(&problem::Point(x, y)) {
                    let mask_index = (y - field_min.1) * field_width + (x - field_min.0);
                    hole_mask.set(mask_index as usize, true);
                }
            }
        }

        let pose = match pose {
            None => problem::Pose {
                vertices: problem.figure.vertices.clone(),
                bonuses: None,
            },
            Some(pose) => pose,
        };
        let pose_score = match problem.score_pose(&pose) {
            Ok(score) => score,
            _ => i64::MAX,
        };

        Ok(Solver {
            hole_mask,
            field_min,
            field_max,
            field_area,
            field_width,
            field_height,
            problem: problem.clone(),
            pose,
            pose_score,
        })
    }

    pub fn is_hole(&self, point: &problem::Point) -> bool {
        if point.0 < self.field_min.0 || point.0 > self.field_max.0 || point.1 < self.field_min.1 || point.1 > self.field_max.1 {
            return false;
        }
        let mask_index = (point.1 - self.field_min.1) * self.field_width + (point.0 - self.field_min.0);
        self.hole_mask.get(mask_index as usize)
            .unwrap_or(false)
    }
}

pub fn is_edge_ratio_valid(edge: &problem::Edge, vertices: &[problem::Point], problem: &problem::Problem) -> (bool, f64) {
    let sample_vertex_a = problem.figure.vertices[edge.0];
    let sample_vertex_b = problem.figure.vertices[edge.1];

    let try_vertex_a = vertices[edge.0];
    let try_vertex_b = vertices[edge.1];

    let sample_sq_dist = (sample_vertex_a.0 - sample_vertex_b.0) * (sample_vertex_a.0 - sample_vertex_b.0)
        + (sample_vertex_a.1 - sample_vertex_b.1) * (sample_vertex_a.1 - sample_vertex_b.1);
    let try_sq_dist = (try_vertex_a.0 - try_vertex_b.0) * (try_vertex_a.0 - try_vertex_b.0)
        + (try_vertex_a.1 - try_vertex_b.1) * (try_vertex_a.1 - try_vertex_b.1);

    let ratio = ((try_sq_dist as f64 / sample_sq_dist as f64) - 1.0).abs();
    (ratio <= problem.epsilon as f64 / 1000000.0, ratio)
}

#[cfg(test)]
mod tests {
    use geo::algorithm::contains::Contains;

    use crate::{
        problem,
        solver::{
            Solver,
        },
    };

    #[test]
    fn is_hole() {
        let problem_data = r#"{"bonuses":[{"bonus":"GLOBALIST","problem":72,"position":[17,10]}],"hole":[[34,0],[17,30],[10,62],[13,30],[0,0]],"epsilon":6731,"figure":{"edges":[[0,1],[0,3],[1,2],[1,3],[2,4],[3,4]],"vertices":[[0,0],[0,34],[17,62],[30,17],[45,46]]}}"#;
        let problem: problem::Problem = serde_json::from_str(problem_data).unwrap();
        let solver = Solver::new(&problem, None).unwrap();
        let hole_poly = problem.hole_polygon();
        for y in solver.field_min.1 ..= solver.field_max.1 {
            for x in solver.field_min.0 ..= solver.field_max.0 {
                let required = hole_poly.contains(&problem::Point(x, y));
                let provided = solver.is_hole(&problem::Point(x, y));
                assert_eq!(required, provided);
            }
        }
    }

    #[test]
    fn is_hole_task13() {
        let problem_data = r#"{"bonuses":[{"bonus":"GLOBALIST","problem":46,"position":[20,20]}],"hole":[[20,0],[40,20],[20,40],[0,20]],"epsilon":2494,"figure":{"edges":[[0,1],[0,2],[1,3],[2,3]],"vertices":[[15,21],[34,0],[0,45],[19,24]]}}"#;
        let problem: problem::Problem = serde_json::from_str(problem_data).unwrap();
        let solver = Solver::new(&problem, None).unwrap();
        assert_eq!(solver.is_hole(&problem::Point(40, 20)), true);
        assert_eq!(solver.is_hole(&problem::Point(20, 0)), true);
        assert_eq!(solver.is_hole(&problem::Point(0, 20)), true);
        assert_eq!(solver.is_hole(&problem::Point(20, 40)), true);
        let hole_poly = problem.hole_polygon();
        assert_eq!(hole_poly.contains(&problem::Point(40, 20)), true);
        assert_eq!(hole_poly.contains(&problem::Point(20, 0)), true);
        assert_eq!(hole_poly.contains(&problem::Point(0, 20)), true);
        assert_eq!(hole_poly.contains(&problem::Point(20, 40)), true);
    }
}
