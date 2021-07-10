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
    vertices_cur: Vec<problem::Point>,
    vertices_tmp: Vec<problem::Point>,
    temp: f64,
}

impl SimulatedAnnealingSolver {
    pub fn new(solver: solver::Solver, params: Params) -> SimulatedAnnealingSolver {
        let mut vertices_cur = Vec::new();
        generate_vertices(&solver, &mut vertices_cur);
        let temp = params.max_temp;
        SimulatedAnnealingSolver {
            solver,
            params,
            vertices_cur,
            vertices_tmp: Vec::new(),
            temp,
        }
    }

    pub fn reset(&mut self) {
        generate_vertices(&self.solver, &mut self.vertices_cur);
        self.temp = self.params.max_temp;
    }

    pub fn vertices(&self) -> &[problem::Point] {
        &self.vertices_cur
    }
}

fn generate_vertices(solver: &solver::Solver, vertices: &mut Vec<problem::Point>) {
    let figure_vertices_iter = solver
        .problem
        .figure
        .vertices
        .iter();
    let mut rng = rand::thread_rng();
    vertices.clear();
    vertices.extend(
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
    );
}
