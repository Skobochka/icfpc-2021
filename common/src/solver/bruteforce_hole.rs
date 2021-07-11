// use std::{
//     path::PathBuf,
// };

use crate::{
    solver,
    problem,
};

#[allow(dead_code)]
pub struct BruteforceHoleSolver {
    solver: solver::Solver,
    step: i64,
    total_steps: i64,
}

impl BruteforceHoleSolver {
    pub fn new(solver: solver::Solver) -> BruteforceHoleSolver {
        let hole_size = solver.problem.hole.len() as i64;
        let figure_size = solver.problem.figure.vertices.len() as u32;
        let total_steps = hole_size.pow(figure_size);
        BruteforceHoleSolver {
            solver,
            total_steps,
            step: 0,
        }
    }

    pub fn solve(&mut self) -> Option<problem::Pose> {
        let mut vertices = self.solver.problem.figure.vertices.clone();
        let hole = self.solver.problem.hole.clone();

        println!("Bruteforce of hole size {} for figure size {}. It will take ~{} iterations",
                 hole.len(), vertices.len(), self.total_steps);
        let (score, pose) = self.run(0, i64::MAX, &mut vertices, &hole);
        println!("Found solution with score {:?}: {:?}", score, pose);
        pose
    }

    fn run(&mut self, vert_idx: usize,
           last_best_score: i64,
           vertices: &mut Vec<problem::Point>,
           hole: &Vec<problem::Point>) -> (i64, Option<problem::Pose>) {

        let mut best_pose_score = last_best_score;
        let mut best_pose = None;

        for hole_vertice in hole {
            vertices[vert_idx] = *hole_vertice;
            let (new_score, new_pose) = if vert_idx == vertices.len() - 1 {
                self.track_progress();
                // self.track_progress(vert_idx, hole_idx);
                match self.solver.problem.score_vertices(vertices) {
                    Ok(score) => (score, Some(problem::Pose {
                        vertices: vertices.clone(),
                        bonuses: None, // fixme
                    })),
                    _ => (i64::MAX, None),
                }
            }
            else {
                self.run(vert_idx + 1, best_pose_score, vertices, hole)
            };

            if new_score == 0 {
                // perfect match
                return (0, Some(problem::Pose {
                    vertices: vertices.clone(),
                    bonuses: None, // fixme
                }))
            }
            else if new_score < self.solver.pose_score {
                // improvement match
                best_pose_score = new_score;
                best_pose = new_pose;
            }
        }
        (best_pose_score, best_pose)
    }

    fn track_progress(&mut self) {
        let perc_before = (self.step / self.total_steps * 1000) as i64;
        self.step += 1;
        let perc_after = (self.step / self.total_steps * 1000) as i64;

        if perc_after > perc_before {
            println!("[BruteforceHoleSolver] PROGRESS: {} ({} of {})",
                     perc_after as f64 / 10.0, self.step, self.total_steps);
        }
    }
}
