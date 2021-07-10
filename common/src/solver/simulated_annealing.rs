use rand::Rng;

use crate::{
    solver,
    problem,
};

#[allow(dead_code)]
pub struct SimulatedAnnealingSolver {
    solver: solver::Solver,
}

impl SimulatedAnnealingSolver {
    pub fn new(solver: solver::Solver) -> SimulatedAnnealingSolver {
        SimulatedAnnealingSolver {
            solver,
        }
    }

    pub fn start(self) -> SimulatedAnnealingSolverStart {
        let figure_vertices_iter = self.solver
            .problem
            .figure
            .vertices
            .iter();
        let mut rng = rand::thread_rng();
        let vertices = figure_vertices_iter
            .map(|_vertex| {
                loop {
                    let x = rng.gen_range(self.solver.field_min.0 ..= self.solver.field_max.1);
                    let y = rng.gen_range(self.solver.field_min.1 ..= self.solver.field_max.1);
                    let point = problem::Point(x, y);
                    if self.solver.is_hole(&point) {
                        break point;
                    }
                }
            })
            .collect();

        SimulatedAnnealingSolverStart {
            solver: self.solver,
            vertices,
        }
    }
}

#[allow(dead_code)]
pub struct SimulatedAnnealingSolverStart {
    solver: solver::Solver,
    vertices: Vec<problem::Point>,
}
