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
           distances: &[(i64, i64)]) -> (i64, Option<problem::Pose>) {

        // the last vertex left. brute-forcing...
        let mut new_pose = None;
        let mut best_score = last_best;
        for next_y in start.1..self.solver.field_max.1 {
            if vert_idx < 2 {
                log::debug!("Starting Y-step {} for idx: {}...", next_y, vert_idx);
            }
            'loop_x: for next_x in start.0..self.solver.field_max.0 {
                let vertice = problem::Point(next_x, next_y);
                if !self.solver.is_hole(&vertice) {
                    continue;
                }

                for idx in 0..vert_idx {
                    let (dmin, dmax) = distances[vert_idx*vertices.len()+idx];
                    // log::debug!("vert_idx: {}, idx: {}, dmin: {}, dmax: {}", vert_idx, idx, dmin, dmax);
                    if dmin == -1 {
                        continue;
                    }
                    let distance = problem::distance(&vertice, &vertices[idx]);
                    // log::debug!("found edge, distance: {}", distance);
                    if distance < dmin || distance > dmax {
                        continue 'loop_x;
                    }
                }

                vertices[vert_idx] = vertice;

                if vert_idx == vertices.len() - 1 {
                    match self.solver.problem.score_vertices(vertices) {
                        Ok(score) => {
                            log::debug!("Found solution with score {:?}: {:?}", score, vertices);
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
                    let (rec_best_score, rec_new_pose) = self.run(vertice, vert_idx + 1, best_score, vertices,
                                                                  distances);
                    if rec_best_score < best_score {
                        best_score = rec_best_score;
                        new_pose = rec_new_pose;
                    }
                }
            }
            if vert_idx < 2 {
                log::debug!("Passed Y-step {} for idx: {}", next_y, vert_idx);
            }

        }

        (best_score, new_pose)
    }

    pub fn solve(&self) -> Option<problem::Pose> {
        let start = self.solver.field_min;
        let mut vertices = self.solver.problem.figure.vertices.clone();
        let mut distances = vec![(-1, -1); vertices.len() * vertices.len()];

        for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
            let distance = problem::distance(&vertices[from_idx], &vertices[to_idx]) as f64;
            let dmin = (distance * (0.5_f64 - self.solver.problem.epsilon as f64 / 1000000.0)).round() as i64;
            let dmax = (distance * (1.5_f64 + self.solver.problem.epsilon as f64 / 1000000.0)).round() as i64;
            distances[from_idx*vertices.len() + to_idx] = (dmin, dmax);
            distances[to_idx*vertices.len() + from_idx] = (dmin, dmax);
        }

        log::debug!("distance matrix: {:?}", distances);
        let (score, pose) = self.run(start, 0, i64::MAX, &mut vertices, &distances);
        println!("Found solution with score {:?}: {:?}", score, pose);
        pose
    }
}
