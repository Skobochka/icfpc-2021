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
    distances: Vec<i64>,
    bonus: Option<problem::PoseBonus>,
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub struct BoundingBox(problem::Point, problem::Point);

impl BoundingBox {
    pub fn point_set(&self) -> HashSet<problem::Point> {
        let capacity = ((self.0.0 - self.1.0).abs() * (self.0.1 - self.1.1).abs()) as usize;
        let mut set = HashSet::with_capacity(capacity);

        for x in cmp::max(0, cmp::min(self.0.0, self.1.0))..=cmp::max(self.0.0, self.1.0) {
            for y in cmp::max(0, cmp::min(self.0.1, self.1.1))..=cmp::max(self.0.1, self.1.1) {
                set.insert(problem::Point(x, y));
            }
        }

        set
    }
}

impl BruteforceHoleSolver {
    pub fn new(solver: solver::Solver) -> BruteforceHoleSolver {
        BruteforceHoleSolver {
            distances: solver.problem.distance_cache(),
            bonus: solver.pose.bonus(),
            solver,
        }
    }

    // pub fn solve(&self) -> Option<problem::Pose> {
    //     let mut vertices = self.solver.problem.figure.vertices.clone();
    //     let hole = HashSet::from_iter(self.solver.problem.hole.iter().cloned());
    //     let distances = self.solver.problem.distance_cache();

    //     println!("Bruteforce of hole size {} for figure size {} with bonus {:?}...", hole.len(), vertices.len(), self.solver.pose.bonus());
    //     match self.solver.pose.bonus() {
    //         Some(problem::PoseBonus::Globalist {..}) => {
    //             println!("GLOBALIST bonus detected. Max ratio is: {}", (self.solver.problem.figure.edges.len() as f64 * self.solver.problem.epsilon as f64) / 1000000_f64);
    //         }
    //         _ => {},
    //     };

    //     let (score, pose) = self.run(0,  i64::MAX, &mut vertices, hole, &distances, self.solver.pose.bonus());
    //     match pose {
    //         None => println!("Solution not found..."),
    //         Some(ref pose) => println!("Found solution with score {:?}: {:?}", score, pose),
    //     };
    //     pose
    // }

    pub fn solve(&self) -> Option<problem::Pose> {
        let (score, pose) = self.solve_dancing();
        match pose {
            None => println!("Solution not found..."),
            Some(ref pose) => println!("Found solution with score {:?}: {:?}", score, pose),
        };
        pose
    }

    fn solve_dancing(&self) -> (i64, Option<problem::Pose>) {
        let mut best_pose_score = i64::MAX;
        let mut best_pose = None;

        for start_idx in 0..self.solver.problem.figure.vertices.len() {
            let mut vertices = self.solver.problem.figure.vertices.clone();
            let mut vertices_placed = bit_vec::BitVec::from_elem(vertices.len(), false);
            /* The first point is picked from the hole */
            println!("looking from vertex {}", start_idx);
            let (new_score, new_pose) = self.place_vertex(start_idx, &mut vertices, &mut vertices_placed,
                                                          /* we always start from hole point */
                                                          HashSet::from_iter(self.solver.problem.hole.iter().cloned()));
            if new_score == 0 {
                /* found ideal solution, no need to continue */
                return (new_score, new_pose);
            }
            else if new_score < best_pose_score {
                best_pose_score = new_score;
                best_pose = new_pose;
            }
        }

        (best_pose_score, best_pose)
    }

    fn place_vertex(&self,
                    idx: usize,
                    vertices: &mut Vec<problem::Point>,
                    vertices_placed: &mut bit_vec::BitVec,
                    candidates: HashSet<problem::Point>,
                    )  -> (i64, Option<problem::Pose>) {
        // println!("place_vertex({}, {:?}, {:?}, {:?}) called", idx, vertices, vertices_placed, candidates.len());
        let mut best_pose_score = i64::MAX;
        let mut best_pose = None;

        let candidates_count = candidates.len();
        'next_point: for point in candidates {
            let mut next_vertice_idxs = Vec::with_capacity(vertices.len());

            for &problem::Edge(from_idx, to_idx) in &self.solver.problem.figure.edges {
                if from_idx != idx {
                    continue;
                }

                let d_before = self.distances[idx * vertices.len() + to_idx];
                if vertices_placed[to_idx] {
                    /* This point is already placed, checking distances */
                    let d_after = problem::distance(&point, &vertices[to_idx]);

                    match self.bonus {
                        Some(problem::PoseBonus::Globalist {..}) => { unimplemented!("No globalist for you!"); },
                        _ => {
                            if ((d_after as f64 / d_before as f64) - 1_f64).abs() > self.solver.problem.epsilon as f64 / 1000000_f64 {
                                continue 'next_point; /* edge would be too long */
                            }
                        }
                    }
                }
                else {
                    /* save idx to continue later */
                    next_vertice_idxs.push(to_idx);
                }
            }

            /* now actually place the vertex */
            vertices[idx] = point;
            vertices_placed.set(idx, true);

            // if idx == 0 {
            //     println!("next_vertice_idxs: {:?}", next_vertice_idxs);
            // }
            // if idx == 2 {
            //     println!("for vertex 2 processing candidates {}, placed: {:?}", candidates_count, vertices_placed);
            // }
            // if next_vertice_idxs.is_empty() {
            //     println!("next_vertice_idxs is empty, idx: {} placed: {:?}", idx, vertices_placed);
            // }
            /* place adjustment vertices */
            for next_idx in next_vertice_idxs {
                /* building candidate set. possible locations for the rib... */
                let edge_distance = self.distances[idx * vertices.len() + next_idx];
                let new_candidates = match self.bonus {
                    Some(problem::PoseBonus::Globalist {..}) => { unimplemented!("No globalist for you!"); },
                    _ => {
                        let min = (edge_distance as f64 - (edge_distance as f64 * self.solver.problem.epsilon as f64) / 1000000_f64).floor() as i64;
                        let max = (edge_distance as f64 + (edge_distance as f64 * self.solver.problem.epsilon as f64) / 1000000_f64).ceil() as i64;
                        self.points_within_distance(point, min, max)
                    }
                }.union(&HashSet::from_iter(self.solver.problem.hole.iter().cloned())).cloned().collect();

                let (new_score, new_pose) = self.place_vertex(next_idx, vertices, vertices_placed, new_candidates);
                if new_score == 0 {
                    /* found ideal solution, no need to continue */
                    return (new_score, new_pose);
                }
                else if new_score < best_pose_score {
                    best_pose_score = new_score;
                    best_pose = new_pose;
                }
            }
            // println!("huh: {:?}", vertices_placed);

            if vertices_placed.all() {
                let (new_score, new_pose) = match self.solver.problem.score_vertices(vertices, self.bonus) {
                    Ok(score) => (score, Some(problem::Pose {
                        vertices: vertices.clone(),
                        bonuses: self.bonus.map(|b| vec![b]),
                    })),
                    Err(_) => (i64::MAX, None),
                };

                if new_score == 0 {
                    /* found ideal solution, no need to continue */
                    return (new_score, new_pose);
                }
                else if new_score < best_pose_score {
                    best_pose_score = new_score;
                    best_pose = new_pose;
                }
            }

            /* 'unplace' the vertex */
            vertices_placed.set(idx, false);

        }
        // println!("place_vertex({}, {:?}, {:?}) exit", idx, vertices, vertices_placed);
        (best_pose_score, best_pose)
    }


    fn run(&self,
           vert_idx: usize, last_best_score: i64,
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
                        let idx: usize;
                        if from_idx == vert_idx {
                            idx = to_idx;
                        }
                        else if to_idx == vert_idx {
                            idx = from_idx;
                        }
                        else {
                            continue;
                        }

                        // ...starting from it.
                        if idx > vert_idx {
                            /* This point not yet placed, ignore it */
                            continue;
                        }
                        // if from_idx != vert_idx {
                        //     continue;
                        // }
                        // if to_idx >= vert_idx {
                        //     continue;
                        // }

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
                            if vert_idx == 2 {
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
            // println!("Hole bruteforce reached its end. We have {} vertices left: {:?}.",
            //          vertices.len() - vert_idx, &vertices[vert_idx..]);
            println!("Full vertices now: {:?}", vertices);

            // // println!("Running plain bruteforce to complete the task");
            // return self.run_plain_bruteforce(self.solver.field_min, vert_idx, best_pose_score,
            //                                  vertices, distances, bonus);
            return self.run_bounding_box(vert_idx,
                                         best_pose_score,
                                         vertices, distances, bonus);
        }

        (best_pose_score, best_pose)
    }

    #[allow(dead_code)]
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
                             bonus: Option<problem::PoseBonus>) -> HashSet<problem::Point>{
        let mut pointset: HashSet<problem::Point> = HashSet::new();
        let mut pointset_ready = false;
        let total_factor = (self.solver.problem.figure.edges.len() as f64 * self.solver.problem.epsilon as f64) / 1000000_f64;
        let mut used_factor = 0_f64;

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
            else {
                let d_before = distances[from_idx * vertices.len() + to_idx];
                let d_after = problem::distance(&vertices[from_idx], &vertices[to_idx]);
                used_factor += (d_after as f64 / d_before as f64 - 1_f64).abs();
                continue;
            }

            // ...starting from it.
            if idx > vert_idx {
                /* This point not yet placed, ignore it */
                continue;
            }

            // TODO: here we need support for globalist, superflex, and other shit like that
            // let edge_budget = distances[vert_idx * vertices.len() + idx] * 2;
            let edge_distance = distances[vert_idx * vertices.len() + idx];

            let (edge_distance_min, edge_distance_max) = match bonus {
                Some(problem::PoseBonus::Globalist {..}) => {
                    let left_factor = total_factor - used_factor;
                    let min = (edge_distance as f64 - edge_distance as f64 * left_factor).floor() as i64;
                    let max = (edge_distance as f64 + edge_distance as f64 * left_factor).ceil() as i64;
                    (cmp::min(0, min), max)
                },
                _ => {
                    // let eps_factor = self.solver.problem.epsilon as f64 / 1000000_f64;
                    let min = (edge_distance as f64 - (edge_distance as f64 * self.solver.problem.epsilon as f64) / 1000000_f64).floor() as i64;
                    let max = (edge_distance as f64 + (edge_distance as f64 * self.solver.problem.epsilon as f64) / 1000000_f64).ceil() as i64;
                    (cmp::min(0, min), max)
                },
            };

            // let edge_distance_min = distances[vert_idx * vertices.len() + idx] - (distances[vert_idx * vertices.len() + idx] as f64 * 0.2).floor() as i64;
            // let edge_distance_max = distances[vert_idx * vertices.len() + idx] + (distances[vert_idx * vertices.len() + idx] as f64 * 0.2).ceil() as i64;

            let next_pointset = self.points_within_distance(vertices[idx], edge_distance_min, edge_distance_max);
            if !pointset_ready {
                pointset = next_pointset;
                pointset_ready = true;
            }
            else {
                pointset = pointset
                    .intersection(&next_pointset)
                    .cloned()
                    .collect();
            }
        }

        // println!("point_set_for_verticle: {:?}", pointset);
        pointset
    }

    fn points_within_distance(&self, point: problem::Point, distance_min: i64, distance_max: i64) -> HashSet<problem::Point>{
        // IMPORTANT: `distance` is SQUARE distance
        let length_min = (distance_min as f64).sqrt() as i64 - 1; // -1 just to be sure :)
        let length_max = (distance_max as f64).sqrt() as i64 + 1; // +1 just to be sure :)

        let outer_box = problem::BoundingBox(problem::Point(point.0 - length_max, point.1 - length_max),
                                             problem::Point(point.0 + length_max, point.1 + length_max));

        let inner_box = problem::BoundingBox(problem::Point(point.0 - length_min, point.1 - length_min),
                                             problem::Point(point.0 + length_min, point.1 + length_min));


        let pointset = problem::SquareRing(outer_box, inner_box)
            .point_set();

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

// #[cfg(test)]
// mod tests {
//     use super::*;

//     #[test]
//     fn bounding_ring_box() {
//         let outer = BoundingBox(problem::Point(0,0), problem::Point(4,4));
//         let inner = BoundingBox(problem::Point(1,1), problem::Point(3,3));

//         let ring = BoundingRingBox(outer, inner);

//         let right = [
//             problem::Point(0,0), problem::Point(1,0), problem::Point(2,0), problem::Point(3,0), problem::Point(4,0),
//             problem::Point(0,1), problem::Point(1,1), problem::Point(2,1), problem::Point(3,1), problem::Point(4,1),
//             problem::Point(0,2), problem::Point(1,2),                      problem::Point(3,2), problem::Point(4,2),
//             problem::Point(0,3), problem::Point(1,3), problem::Point(2,3), problem::Point(3,3), problem::Point(4,3),
//             problem::Point(0,4), problem::Point(1,4), problem::Point(2,4), problem::Point(3,4), problem::Point(4,4),
//             ];
//         assert_eq!(ring.point_set(), right.iter().cloned().collect());
//     }
// }
