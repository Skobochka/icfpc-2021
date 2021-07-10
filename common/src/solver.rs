use geo::algorithm::contains::Contains;

use crate::{
    problem,
};

pub struct Solver {
    hole_mask: bit_vec::BitVec,
    field_min: problem::Point,
    field_max: problem::Point,
    field_area: usize,
    field_width: i64,
    field_height: i64,
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
                    let mask_index = y * field_width + x;
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
        })
    }
}
