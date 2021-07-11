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


    fn run(&self,
           start: problem::Point, vert_idx: usize, last_best: i64,
           vertices: &mut Vec<problem::Point>,
           distances: &[i64]) -> (i64, Option<problem::Pose>) {

        // the last vertex left. brute-forcing...
        let mut new_pose = None;
        let mut best_score = last_best;
        let mut next_y = start.1;
        let mut next_x = start.0;
        while next_y <= self.solver.field_max.1 {
            if vert_idx < 2 {
                // log::debug!("Starting Y-step {} for idx: {}...", next_y, vert_idx);
            }
            'loop_x: while next_x <= self.solver.field_max.0 {

                if vert_idx < 2 {
                    // log::debug!("Starting X-step {} for idx: {}...", next_x, vert_idx);
                }
                if next_y > self.solver.field_max.1 {
                    break;
                }

                let vertice = problem::Point(next_x, next_y);
                // log::debug!("checking {} vertice: {:?}, vertices: {:?}, is_hole: {}", vert_idx, vertice, vertices, self.solver.is_hole(&vertice));

                next_x += 1;
                if next_x > self.solver.field_max.0 {
                    next_x = 0;
                    next_y += 1;
                }

                if !self.solver.is_hole(&vertice) {
                    continue;
                }

                for idx in 0..vert_idx {
                    let edge_distance = distances[vert_idx*vertices.len()+idx];
                    if edge_distance == -1 {
                        continue;
                    }
                    let distance = problem::distance(&vertice, &vertices[idx]);

                    if ((distance as f64 / edge_distance as f64) - 1_f64).abs() > self.solver.problem.epsilon as f64 / 1000000_f64 {
                        continue 'loop_x;
                    }
                }

                vertices[vert_idx] = vertice;

                if vert_idx == vertices.len() - 1 {
                    // log::debug!("scoring candidate... {:?}", vertices);

                    match self.solver.problem.score_vertices(vertices, &None) {
                        Ok(score) => {
                            // log::debug!("Found solution with score {:?}: {:?}", score, vertices);
                            if score == 0 { // perfect solution found
                                return (0, Some(problem::Pose {
                                    vertices: vertices.clone(),
                                    bonuses: None, // fixme
                                }))
                            }
                            if score < best_score {
                                best_score = score;
                                new_pose = Some(problem::Pose {
                                    vertices: vertices.clone(),
                                    bonuses: None, // fixme
                                })
                            }
                        },
                        _ => continue,
                    }
                }
                else {
                    let (rec_best_score, rec_new_pose) = self.run(self.solver.field_min, vert_idx + 1, best_score, vertices,
                                                                  distances);
                    if rec_best_score == 0 {
                        return (0, rec_new_pose);
                    }
                    if rec_best_score < best_score {
                        best_score = rec_best_score;
                        new_pose = rec_new_pose;
                    }
                }
            }
            if vert_idx < 2 {
                // log::debug!("Passed Y-step {} for idx: {}", next_y, vert_idx);
            }

        }

        (best_score, new_pose)
    }

    pub fn solve(&self) -> Option<problem::Pose> {
        let mut vertices = self.solver.problem.figure.vertices.clone();
        let mut distances = vec![-1; vertices.len() * vertices.len()];

        for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
            let distance = problem::distance(&vertices[from_idx], &vertices[to_idx]);
            distances[from_idx*vertices.len() + to_idx] = distance;
            distances[to_idx*vertices.len() + from_idx] = distance;
        }

        // log::debug!("distance matrix: {:?}", distances);
        let (score, pose) = self.run(self.solver.field_min, 0, i64::MAX, &mut vertices, &distances);
        println!("Found solution with score {:?}: {:?}", score, pose);
        pose
    }
}
