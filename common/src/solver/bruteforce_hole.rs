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
    step: u128,
    total_steps: u128,
}

impl BruteforceHoleSolver {
    pub fn new(solver: solver::Solver) -> BruteforceHoleSolver {
        let hole_size = solver.problem.hole.len() as u128;
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
        let mut distances = vec![-1; vertices.len() * vertices.len()];

        for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
            let distance = problem::distance(&vertices[from_idx], &vertices[to_idx]);
            distances[from_idx*vertices.len() + to_idx] = distance;
            distances[to_idx*vertices.len() + from_idx] = distance;
        }

        println!("Bruteforce of hole size {} for figure size {}. It will take ~{} iterations",
                 hole.len(), vertices.len(), self.total_steps);
        let (score, pose) = self.run(0, i64::MAX, &mut vertices, &hole, &distances);
        match pose {
            None => println!("Found solution not found..."),
            Some(ref pose) => println!("Found solution with score {:?}: {:?}", score, pose),
        };
        pose
    }

    fn run(&mut self, vert_idx: usize,
           last_best_score: i64,
           vertices: &mut Vec<problem::Point>,
           hole: &Vec<problem::Point>,
           distances: &[i64]) -> (i64, Option<problem::Pose>) {

        let mut best_pose_score = last_best_score;
        let mut best_pose = None;

        'next_hole_vertice: for hole_vertice in hole {
            for idx in 0..vert_idx {
                let edge_distance = distances[vert_idx*vertices.len()+idx];
                if edge_distance == -1 {
                    continue;
                }
                let distance = problem::distance(&hole_vertice, &vertices[idx]);

                if ((distance as f64 / edge_distance as f64) - 1_f64).abs() > self.solver.problem.epsilon as f64 / 1000000_f64 {
                    self.track_progress((hole.len() as u128).pow((vertices.len() - vert_idx) as u32));
                    continue 'next_hole_vertice;
                }
            }

            vertices[vert_idx] = *hole_vertice;
            let (new_score, new_pose) = if vert_idx == vertices.len() - 1 {
                self.track_progress(1);
                // self.track_progress(vert_idx, hole_idx);
                match self.solver.problem.score_vertices(vertices, None) {
                    Ok(score) => (score, Some(problem::Pose {
                        vertices: vertices.clone(),
                        bonuses: None, // fixme
                    })),
                    _ => (i64::MAX, None),
                }
            }
            else {
                self.run(vert_idx + 1, best_pose_score, vertices, hole, distances)
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

    fn track_progress(&mut self, step: u128) {
        let perc_before = ((self.step * 1000) / self.total_steps) as u128;
        self.step += step;
        let perc_after = ((self.step * 1000) / self.total_steps) as u128;

        if perc_after > perc_before {
            println!("PROGRESS: {} ({} of {})",
                     perc_after as f64 / 10.0, self.step, self.total_steps);
        }
    }
}
