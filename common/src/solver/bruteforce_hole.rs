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

        println!("Bruteforce of hole size {} for figure size {} with bonus {:?}...", hole.len(), vertices.len(), self.solver.pose.bonus());
        let (score, pose) = self.run(0, i64::MAX, &mut vertices, &hole, &distances, self.solver.pose.bonus());
        match pose {
            None => println!("Solution not found..."),
            Some(ref pose) => println!("Found solution with score {:?}: {:?}", score, pose),
        };
        pose
    }

    fn run(&mut self, vert_idx: usize,
           last_best_score: i64,
           vertices: &mut Vec<problem::Point>,
           hole: &Vec<problem::Point>,
           distances: &[i64],
           bonus: Option<problem::PoseBonus>) -> (i64, Option<problem::Pose>) {

        let mut best_pose_score = last_best_score;
        let mut best_pose = None;
        let mut progress = 1;
        'next_hole_vertice: for hole_vertice in hole {
            if vert_idx == 0 {
                println!("Starting {} of {}...", progress, hole.len());
            }
            if vert_idx == 1 {
                println!(" + Starting {} of {}...", progress, hole.len());
            }
            if vert_idx == 2 {
                println!("  ++  Starting {} of {}...", progress, hole.len());
            }

            match bonus {
                Some(problem::PoseBonus::Globalist {..}) => {
                    let mut eps = 0_f64;
                    for idx in 0..vert_idx {
                        for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
                            if from_idx != vert_idx {
                                continue;
                            }
                            if to_idx > vert_idx {
                                continue;
                            }

                            let d_before = problem::distance(&self.solver.problem.figure.vertices[from_idx], &self.solver.problem.figure.vertices[to_idx]);
                            let d_after = problem::distance(&hole_vertice, &vertices[idx]);
                            eps += ((d_after as f64 / d_before as f64) - 1_f64).abs();
                        }

                        // //let d_before = distances[vert_idx*vertices.len()+idx];
                        // if d_before == -1 {
                        //     continue;
                        // }
                    }
                    let max_eps = self.solver.problem.figure.edges.len() as f64 * self.solver.problem.epsilon as f64 / 1000000_f64 ;
                    if eps > max_eps{
                        // self.track_progress((hole.len() as u128).pow((vertices.len() - vert_idx) as u32));
                        if vert_idx == 0 {
                            println!("skipped {}..., eps: {}, max_eps: {}, orig_eps: {}", progress, eps, max_eps, self.solver.problem.epsilon as f64 / 1000000_f64);
                            progress += 1;
                        }
                        if vert_idx == 1 {
                            println!(" + skipped {}..., eps: {}, max_eps: {}, orig_eps: {}", progress, eps, max_eps, self.solver.problem.epsilon as f64 / 1000000_f64);
                            progress += 1;
                        }
                        if vert_idx == 2 {
                            println!("  ++  skipped {}..., eps: {}, max_eps: {}", progress, eps, max_eps);
                            progress += 1;
                        }
                        continue 'next_hole_vertice;
                    }
                },
                _ => {
                    let mut superstretch_allow = match bonus {
                        Some(problem::PoseBonus::Superflex {..}) => 1,
                        _ => 0,
                    };
                    for idx in 0..vert_idx {
                        for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
                            if from_idx != vert_idx {
                                continue;
                            }
                            if to_idx > vert_idx {
                                continue;
                            }

                            let d_before = problem::distance(&self.solver.problem.figure.vertices[from_idx], &self.solver.problem.figure.vertices[to_idx]);
                            let d_after = problem::distance(&hole_vertice, &vertices[idx]);

                            if ((d_after as f64 / d_before as f64) - 1_f64).abs() > self.solver.problem.epsilon as f64 / 1000000_f64 {
                                if superstretch_allow > 0 {
                                    superstretch_allow = 0;
                                    continue;
                                }

                                // self.track_progress((hole.len() as u128).pow((vertices.len() - vert_idx) as u32));
                                if vert_idx == 0 {
                                    println!("skipped {}...", progress);
                                    progress += 1;
                                }
                                if vert_idx == 1 {
                                    println!(" + skipped {}...", progress);
                                    progress += 1;
                                }
                                if vert_idx == 3 {
                                    println!("  ++  skipped {}...", progress);
                                    progress += 1;
                                }
                                continue 'next_hole_vertice;
                            }
                        }
                    }
                }
            }

            vertices[vert_idx] = *hole_vertice;
            let (new_score, new_pose) = if vert_idx == vertices.len() - 1 {
                // println!("NOT SKIPPED");
                match self.solver.problem.score_vertices(vertices, bonus) {
                    Ok(score) => (score, Some(problem::Pose {
                        vertices: vertices.clone(),
                        bonuses: bonus.map(|b| vec![b]),
                    })),
                    _ => (i64::MAX, None),
                }
            }
            else {
                self.run(vert_idx + 1, best_pose_score, vertices, hole, distances, bonus)
            };

            if new_score == 0 {
                // perfect match
                return (0, Some(problem::Pose {
                    vertices: vertices.clone(),
                    bonuses: bonus.map(|b| vec![b]),
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
            if vert_idx == 1 {
                println!(" + DONE {} of {}...", progress, hole.len());
                progress += 1;
            }
            if vert_idx == 2 {
                println!("  ++  DONE {} of {}...", progress, hole.len());
                progress += 1;
            }
        }
        (best_pose_score, best_pose)
    }
}
