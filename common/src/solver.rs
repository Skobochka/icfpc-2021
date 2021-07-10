use geo::algorithm::contains::Contains;

use crate::{
    problem,
};

pub mod simulated_annealing;

#[allow(dead_code)]
pub struct Solver {
    hole_mask: bit_vec::BitVec,
    field_min: problem::Point,
    field_max: problem::Point,
    field_area: usize,
    field_width: i64,
    field_height: i64,
    problem: problem::Problem,
}

#[derive(Debug)]
pub enum CreateError {
    NoPointsInHole,
    NoPointsInFigure,
}

impl Solver {
    pub fn new(problem: &problem::Problem) -> Result<Solver, CreateError> {
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
        let field_width = field_max.0 - field_min.0;
        let field_height = field_max.1 - field_min.1;
        let field_area = (field_width * field_height) as usize;
        let mut hole_mask = bit_vec::BitVec::from_elem(field_area, false);

        let hole_poly = problem.hole_polygon();
        for y in field_min.1 .. field_max.1 {
            for x in field_min.0 .. field_max.0 {
                if hole_poly.contains(&problem::Point(x, y)) {
                    let mask_index = (y - field_min.1) * field_width + (x - field_min.0);
                    hole_mask.set(mask_index as usize, true);
                }
            }
        }

        Ok(Solver {
            hole_mask,
            field_min,
            field_max,
            field_area,
            field_width,
            field_height,
            problem: problem.clone(),
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
