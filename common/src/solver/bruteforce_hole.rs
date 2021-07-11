// use std::{
//     path::PathBuf,
// };

use std::{
    cmp,
    collections::HashSet,
    iter::FromIterator,
};

use crate::{
    solver,
    problem,
};

#[allow(dead_code)]
pub struct BruteforceHoleSolver {
    solver: solver::Solver,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct BoundingBox(problem::Point, problem::Point);

impl BoundingBox {
    pub fn point_set(&self) -> HashSet<problem::Point> {
        let capacity = ((self.0.0 - self.1.0).abs() * (self.0.1 - self.1.1).abs()) as usize;
        let mut set = HashSet::with_capacity(capacity);

        for x in cmp::max(0, cmp::min(self.0.0, self.1.0))..cmp::max(self.0.0, self.1.0) {
            for y in cmp::max(0, cmp::min(self.0.1, self.1.1))..cmp::max(self.0.1, self.1.1) {
                set.insert(problem::Point(x, y));
            }
        }

        set
    }
}

impl BruteforceHoleSolver {
    pub fn new(solver: solver::Solver) -> BruteforceHoleSolver {
        BruteforceHoleSolver {
            solver,
        }
    }

    pub fn solve(&self) -> Option<problem::Pose> {
        let mut vertices = self.solver.problem.figure.vertices.clone();
        let hole = HashSet::from_iter(self.solver.problem.hole.iter().cloned());
        let mut distances = vec![-1; vertices.len() * vertices.len()];

        for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
            let distance = problem::distance(&vertices[from_idx], &vertices[to_idx]);
            distances[from_idx*vertices.len() + to_idx] = distance;
            distances[to_idx*vertices.len() + from_idx] = distance;
        }

        println!("Bruteforce of hole size {} for figure size {} with bonus {:?}...", hole.len(), vertices.len(), self.solver.pose.bonus());
        match self.solver.pose.bonus() {
            Some(problem::PoseBonus::Globalist {..}) => {
                println!("GLOBALIST bonus detected. Max ratio is: {}", (self.solver.problem.figure.edges.len() as f64 * self.solver.problem.epsilon as f64) / 1000000_f64);
            }
            _ => {},
        };

        let (score, pose) = self.run(0, i64::MAX, &mut vertices, hole, &distances, self.solver.pose.bonus());
        match pose {
            None => println!("Solution not found..."),
            Some(ref pose) => println!("Found solution with score {:?}: {:?}", score, pose),
        };
        pose
    }

    fn run(&self, vert_idx: usize,
           last_best_score: i64,
           vertices: &mut Vec<problem::Point>,
           hole: HashSet<problem::Point>,
           distances: &[i64],
           bonus: Option<problem::PoseBonus>) -> (i64, Option<problem::Pose>) {
        let mut best_pose_score = last_best_score;
        let mut best_pose = None;
        let mut progress = 1;
        'next_hole_vertice: for hole_vertice in &hole {
            if vert_idx == 0 {
                println!("Starting {} of {}...", progress, hole.len());
            }
            if vert_idx == 1 {
                println!(" + Starting {} of {}...", progress, hole.len());
            }
            if vert_idx == 2 {
                println!("  ++  Starting {} of {}...", progress, hole.len());
            }

            /* We have points left, applying */
            match bonus {
                Some(problem::PoseBonus::Globalist {..}) => {
                    let mut eps = 0_f64;
                    for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
                        if from_idx != vert_idx {
                            continue;
                        }
                        if to_idx >= vert_idx {
                            continue;
                        }

                        let d_before = problem::distance(&self.solver.problem.figure.vertices[from_idx], &self.solver.problem.figure.vertices[to_idx]);
                        let d_after = problem::distance(&hole_vertice, &vertices[to_idx]);
                        eps += ((d_after as f64 / d_before as f64) - 1_f64).abs();
                    }

                    let max_eps = self.solver.problem.figure.edges.len() as f64 * self.solver.problem.epsilon as f64 / 1000000_f64 ;
                    if eps > max_eps{
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
                    for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
                        if from_idx != vert_idx {
                            continue;
                        }
                        if to_idx >= vert_idx {
                            continue;
                        }

                        let d_before = problem::distance(&self.solver.problem.figure.vertices[from_idx], &self.solver.problem.figure.vertices[to_idx]);
                        let d_after = problem::distance(&hole_vertice, &vertices[to_idx]);

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

            vertices[vert_idx] = *hole_vertice;
            let (new_score, new_pose) = if vert_idx == vertices.len() - 1 {
                match self.solver.problem.score_vertices(vertices, bonus) {
                    Ok(score) => (score, Some(problem::Pose {
                        vertices: vertices.clone(),
                        bonuses: bonus.map(|b| vec![b]),
                    })),
                    Err(_) => (i64::MAX, None),
                    // Err(e) => {println!("Got error {:?}", e); (i64::MAX, None)},
                }
            }
            else {
                let mut new_hole = hole.clone();
                new_hole.remove(hole_vertice);
                self.run(vert_idx + 1, best_pose_score, vertices, new_hole, distances, bonus)
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

        if hole.is_empty() {
            // // println!("Hole bruteforce reached its end. We have {} vertices left: {:?}",
            // //          vertices.len() - vert_idx, &vertices[vert_idx..]);

            // // println!("Running plain bruteforce to complete the task");
            // return self.run_plain_bruteforce(self.solver.field_min, vert_idx, best_pose_score,
            //                                  vertices, distances, bonus);
            return self.run_bounding_box(vert_idx,
                                         best_pose_score,
                                         vertices, distances, bonus);
        }

        (best_pose_score, best_pose)
    }

    fn run_bounding_box(&self,
                        vert_idx: usize,
                        last_best_score: i64,
                        vertices: &mut Vec<problem::Point>,
                        distances: &[i64],
                        bonus: Option<problem::PoseBonus>) -> (i64, Option<problem::Pose>) {
        let mut best_pose_score = last_best_score;
        let mut best_pose = None;

        for point in self.point_set_for_vertice(vert_idx, vertices, distances, bonus) {
            vertices[vert_idx] = point;
            let (new_score, new_pose) = if vert_idx == vertices.len() - 1 {
                match self.solver.problem.score_vertices(vertices, bonus) {
                    Ok(score) => (score, Some(problem::Pose {
                        vertices: vertices.clone(),
                        bonuses: bonus.map(|b| vec![b]),
                    })),
                    Err(_) => (i64::MAX, None),
                    // Err(e) => {println!("Got error {:?}", e); (i64::MAX, None)},
                }
            }
            else {
                self.run_bounding_box(vert_idx + 1, best_pose_score, vertices, distances, bonus)
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
        }

        (best_pose_score, best_pose)
    }

    fn point_set_for_vertice(&self,
                             vert_idx: usize,
                             vertices: &mut Vec<problem::Point>,
                             distances: &[i64],
                             _bonus: Option<problem::PoseBonus>) -> HashSet<problem::Point>{
        let mut pointset: HashSet<problem::Point> = HashSet::new();
        let mut pointset_ready = false;

        // ...find all edges...
        for &problem::Edge(from_idx, to_idx) in &self.solver.problem.figure.edges {
            // println!("processing Edge: {} => {} for vert_idx: {}", from_idx, to_idx, vert_idx);
            let idx: usize;
            if from_idx == vert_idx {
                idx = to_idx;
            }
            else if to_idx == vert_idx {
                idx = from_idx;
            }
            else { continue; }

            // ...starting from it.
            if idx > vert_idx {
                /* This point not yet placed, ignore it */
                continue;
            }

            // TODO: here we need support for globalist, superflex, and other shit like that
            let edge_budget = distances[vert_idx * vertices.len() + idx] * 2;

            if !pointset_ready {
                pointset = self.points_within_distance(vertices[idx], edge_budget);
                pointset_ready = true;
            }
            else {
                pointset = pointset
                    .intersection(&self.points_within_distance(vertices[idx], edge_budget))
                    .cloned()
                    .collect();
            }
        }

        // println!("point_set_for_verticle: {:?}", pointset);
        pointset
    }

    fn points_within_distance(&self, point: problem::Point, distance: i64) -> HashSet<problem::Point>{
        // IMPORTANT: `distance` is SQUARE distance
        let length = (distance as f64).sqrt() as i64 + 1; // +1 just to be sure :)

        let pointset = BoundingBox(problem::Point(point.0 - length, point.1 - length),
                                   problem::Point(point.0 + length, point.1 + length))
            .point_set();

        // println!("points_within_distance({:?}, {}): {:?}", point, distance, pointset);

        pointset
    }

    #[allow(dead_code)]
    fn run_plain_bruteforce(&self,
                            start: problem::Point, vert_idx: usize, last_best: i64,
                            vertices: &mut Vec<problem::Point>,
                            distances: &[i64],
                            bonus: Option<problem::PoseBonus>) -> (i64, Option<problem::Pose>) {

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

                // match bonus {
                //     Some(problem::PoseBonus::Wallhack {..}) => 1,
                //     _ => 0,
                // }
                if !self.solver.is_hole(&vertice) {
                    continue;
                }

                match bonus {
                    Some(problem::PoseBonus::Globalist {..}) => {
                        let mut eps = 0_f64;
                        for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
                            if from_idx != vert_idx {
                                continue;
                            }
                            if to_idx >= vert_idx {
                                continue;
                            }

                            let d_before = problem::distance(&self.solver.problem.figure.vertices[from_idx], &self.solver.problem.figure.vertices[to_idx]);
                            let d_after = problem::distance(&vertice, &vertices[to_idx]);
                            eps += ((d_after as f64 / d_before as f64) - 1_f64).abs();
                        }

                        let max_eps = self.solver.problem.figure.edges.len() as f64 * self.solver.problem.epsilon as f64 / 1000000_f64 ;
                        if eps > max_eps{
                            // if vert_idx == 0 {
                            //     println!("skipped {}..., eps: {}, max_eps: {}, orig_eps: {}", progress, eps, max_eps, self.solver.problem.epsilon as f64 / 1000000_f64);
                            //     progress += 1;
                            // }
                            // if vert_idx == 1 {
                            //     println!(" + skipped {}..., eps: {}, max_eps: {}, orig_eps: {}", progress, eps, max_eps, self.solver.problem.epsilon as f64 / 1000000_f64);
                            //     progress += 1;
                            // }
                            // if vert_idx == 2 {
                            //     println!("  ++  skipped {}..., eps: {}, max_eps: {}", progress, eps, max_eps);
                            //     progress += 1;
                            // }
                            continue 'loop_x;
                        }
                    },
                    _ => {
                        let mut superstretch_allow = match bonus {
                            Some(problem::PoseBonus::Superflex {..}) => 1,
                            _ => 0,
                        };
                        for &problem::Edge(from_idx, to_idx) in self.solver.problem.figure.edges.iter() {
                            if from_idx != vert_idx {
                                continue;
                            }
                            if to_idx >= vert_idx {
                                continue;
                            }

                            let d_before = problem::distance(&self.solver.problem.figure.vertices[from_idx], &self.solver.problem.figure.vertices[to_idx]);
                            let d_after = problem::distance(&vertice, &vertices[to_idx]);

                            if ((d_after as f64 / d_before as f64) - 1_f64).abs() > self.solver.problem.epsilon as f64 / 1000000_f64 {
                                if superstretch_allow > 0 {
                                    superstretch_allow = 0;
                                    continue;
                                }

                                // self.track_progress((hole.len() as u128).pow((vertices.len() - vert_idx) as u32));
                                // if vert_idx == 0 {
                                //     println!("skipped {}...", progress);
                                //     progress += 1;
                                // }
                                // if vert_idx == 1 {
                                //     println!(" + skipped {}...", progress);
                                //     progress += 1;
                                // }
                                // if vert_idx == 3 {
                                //     println!("  ++  skipped {}...", progress);
                                //     progress += 1;
                                // }
                                continue 'loop_x;
                            }
                        }
                    }
                }

                vertices[vert_idx] = vertice;

                if vert_idx == vertices.len() - 1 {
                    // log::debug!("scoring candidate... {:?}", vertices);

                    match self.solver.problem.score_vertices(vertices, bonus) {
                        Ok(score) => {
                            // log::debug!("Found solution with score {:?}: {:?}", score, vertices);
                            if score == 0 { // perfect solution found
                                return (0, Some(problem::Pose {
                                    vertices: vertices.clone(),
                                    bonuses: bonus.map(|b| vec![b]),
                                }))
                            }
                            if score < best_score {
                                best_score = score;
                                new_pose = Some(problem::Pose {
                                    vertices: vertices.clone(),
                                    bonuses: bonus.map(|b| vec![b]),
                                })
                            }
                        },
                        _ => continue,
                    }
                }
                else {
                    let (rec_best_score, rec_new_pose) = self.run_plain_bruteforce(self.solver.field_min, vert_idx + 1, best_score, vertices,
                                                                                   distances, bonus);
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
}
