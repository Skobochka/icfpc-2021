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
           vertices: &mut Vec<problem::Point>) -> (i64, Option<problem::Pose>) {

        // the last vertex left. brute-forcing...
        let mut new_pose = None;
        let mut best_score = last_best;
        for next_y in start.1..self.solver.field_max.1 {
            for next_x in start.0..self.solver.field_max.0 {
                let vertice = problem::Point(next_x, next_y);
                if !self.solver.is_hole(&vertice) {
                    continue;
                }

                vertices[vert_idx] = vertice;

                if vert_idx == vertices.len() - 1 {
                    match self.solver.problem.score_vertices(vertices) {
                        Ok(score) => {
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
                    let (rec_best_score, rec_new_pose) = self.run(vertice, vert_idx + 1, best_score, vertices);
                    if rec_best_score < best_score {
                        best_score = rec_best_score;
                        new_pose = rec_new_pose;
                    }
                }
            }
        }

        (best_score, new_pose)
    }

    pub fn solve(&self) -> Option<problem::Pose> {
        let start = self.solver.field_min;
        let mut vertices = self.solver.problem.figure.vertices.clone();
        let (score, pose) = self.run(start, 0, i64::MAX, &mut vertices);
        println!("Found solution with score {:?}: {:?}", score, pose);
        pose
    }
}
