use rand::Rng;

use crate::{
    solver,
    problem,
};

pub struct SimulatedAnnealingSolver {
    solver: solver::Solver,
    vertices: Vec<problem::Point>,
}

impl SimulatedAnnealingSolver {
    pub fn new(solver: solver::Solver) -> SimulatedAnnealingSolver {
        let vertices = generate_init_vertices(&solver);
        SimulatedAnnealingSolver {
            solver,
            vertices,
        }
    }

    pub fn reset(&mut self) {
        self.vertices = generate_init_vertices(&self.solver);
    }

    pub fn vertices(&self) -> &[problem::Point] {
        &self.vertices
    }
}

fn generate_init_vertices(solver: &solver::Solver) -> Vec<problem::Point> {
    let figure_vertices_iter = solver
        .problem
        .figure
        .vertices
        .iter();
    let mut rng = rand::thread_rng();
    figure_vertices_iter
        .map(|_vertex| {
            loop {
                let x = rng.gen_range(solver.field_min.0 ..= solver.field_max.1);
                let y = rng.gen_range(solver.field_min.1 ..= solver.field_max.1);
                let point = problem::Point(x, y);
                if solver.is_hole(&point) {
                    break point;
                }
            }
        })
        .collect()
}
