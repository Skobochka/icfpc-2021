use rand::Rng;

use crate::{
    solver,
    problem,
};

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub max_temp: f64,
    pub cooling_step_temp: f64,
}

pub struct SimulatedAnnealingSolver {
    solver: solver::Solver,
    params: Params,
    vertices: Vec<problem::Point>,
    temp: f64,
}

impl SimulatedAnnealingSolver {
    pub fn new(solver: solver::Solver, params: Params) -> SimulatedAnnealingSolver {
        let vertices = generate_init_vertices(&solver);
        let temp = params.max_temp;
        SimulatedAnnealingSolver {
            solver,
            params,
            vertices,
            temp,
        }
    }

    pub fn reset(&mut self) {
        self.vertices = generate_init_vertices(&self.solver);
        self.temp = self.params.max_temp;
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
