use crate::{
    solver,
    problem,
};

#[allow(dead_code)]
pub struct BruteforceSolver {
    solver: solver::Solver,
}

impl BruteforceSolver {
    pub fn new(solver: solver::Solver) -> BruteforceSolver {
        BruteforceSolver {
            solver,
        }
    }

    pub fn solve(self) -> Option<problem::Pose> {
        None
    }
}
