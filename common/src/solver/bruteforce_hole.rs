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
}

impl BruteforceHoleSolver {
    pub fn new(solver: solver::Solver) -> BruteforceHoleSolver {
        BruteforceHoleSolver {
            solver,
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

        println!("Bruteforce of hole size {} for figure size {}...", hole.len(), vertices.len());
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
        let mut progress = 0;
        'next_hole_vertice: for hole_vertice in hole {
            if vert_idx == 0 {
                println!("Starting {} of {}...", progress, hole.len());
            }
            for idx in 0..vert_idx {
                let edge_distance = distances[vert_idx*vertices.len()+idx];
                if edge_distance == -1 {
                    continue;
                }
                let distance = problem::distance(&hole_vertice, &vertices[idx]);

                if ((distance as f64 / edge_distance as f64) - 1_f64).abs() > self.solver.problem.epsilon as f64 / 1000000_f64 {
                    // self.track_progress((hole.len() as u128).pow((vertices.len() - vert_idx) as u32));
                    if vert_idx == 0 {
                        println!("skipped {}...", progress);
                        progress += 1;
                    }
                    continue 'next_hole_vertice;
                }
            }

            vertices[vert_idx] = *hole_vertice;
            let (new_score, new_pose) = if vert_idx == vertices.len() - 1 {
                // self.track_progress(1);
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
            if vert_idx == 0 {
                println!("DONE {} of {}...", progress, hole.len());
                progress += 1;
            }
        }
        (best_pose_score, best_pose)
    }
}
