use std::{
    hash::{
        Hash,
        Hasher,
    },
    sync::{
        Arc,
        RwLock,
    },
};

use rand::Rng;

use crate::{
    problem::{
        self,
        InvalidEdge,
    },
};

pub struct GeoHoleBloom {
    bits: bit_vec::BitVec,
    field_min: problem::Point,
    field_max: problem::Point,
    hash_fns_seeds: Vec<u64>,
    geo_hole: geo::Polygon<f64>,
}

#[derive(Debug)]
pub enum CreateError {
    NoPointsInHole,
}

impl GeoHoleBloom {
    pub fn new(problem: &problem::Problem) -> Result<GeoHoleBloom, CreateError> {
        if problem.hole.is_empty() {
            return Err(CreateError::NoPointsInHole);
        }

        let field_min = problem::Point(
            problem.hole.iter()
                .map(|p| p.0)
                .min()
                .unwrap(),
            problem.hole.iter()
                .map(|p| p.1)
                .min()
                .unwrap(),
        );
        let field_max = problem::Point(
            problem.hole.iter()
                .map(|p| p.0)
                .max()
                .unwrap(),
            problem.hole.iter()
                .map(|p| p.1)
                .max()
                .unwrap(),
        );
        let field_width = field_max.0 - field_min.0 + 1;
        let field_height = field_max.1 - field_min.1 + 1;
        let field_area = field_width * field_height;
        let sq_field_area = field_area * field_area;
        let bits_count = (sq_field_area / 10) as usize;

        let hash_fns_count = ((bits_count as f64) / (sq_field_area as f64) * (2.0_f64).ln()).ceil() as usize;

        // panic!("sq_field_area = {}, bits_count = {}, hash_fns_count = {}", sq_field_area, bits_count, hash_fns_count);

        let mut rng = rand::thread_rng();
        let hash_fns_seeds: Vec<u64> = (0 .. hash_fns_count).map(|_| rng.gen()).collect();

        let shared_bits = Arc::new(RwLock::new(
            bit_vec::BitVec::from_elem(bits_count, false),
        ));

        let geo_hole = problem.hole_polygon_f64();

        log::debug!(
            "generating GeoHoleBloom of {} false positive prob with sq_field_area = {}, bits_count = {}, hash_fns_count = {}",
            (1.0 - ((-(hash_fns_count as f64) * sq_field_area as f64) / bits_count as f64).exp()).powi(bits_count as i32),
            sq_field_area,
            bits_count,
            hash_fns_count,
        );

        use rayon::prelude::*;
        (0 .. sq_field_area)
            .into_par_iter()
            .flat_map_iter(|sq_position| {
                let position_a = sq_position / field_area;
                let position_b = sq_position % field_area;

                let point_a = problem::Point(
                    position_a % field_width,
                    position_a / field_width,
                );
                let point_b = problem::Point(
                    position_b % field_width,
                    position_b / field_width,
                );

                let key = BloomKey::new(point_a, point_b);

                let is_invalid = geo_hole.is_edge_invalid(point_a, point_b);
                let shared_bits = shared_bits.clone();
                hash_fns_seeds
                    .iter()
                    .cloned()
                    .flat_map(move |hash_fn_seed| {
                        if is_invalid {
                            let mut hasher = seahash::SeaHasher::default();
                            hash_fn_seed.hash(&mut hasher);
                            key.hash(&mut hasher);
                            let hash = hasher.finish();
                            let bit_index = (hash % bits_count as u64) as usize;

                            let read_lock = shared_bits.read().unwrap();
                            if !read_lock.get(bit_index).unwrap() {
                                Some(bit_index)
                            } else {
                                None
                            }
                        } else {
                            None
                        }
                    })
            })
            .for_each_with(
                shared_bits.clone(),
                |shared_bits, bit_index| {
                    let mut bits = shared_bits.write().unwrap();
                    bits.set(bit_index, true);
                },
            );

        let bits = Arc::try_unwrap(shared_bits)
            .unwrap()
            .into_inner()
            .unwrap();

        Ok(GeoHoleBloom {
            bits,
            field_min,
            field_max,
            hash_fns_seeds,
            geo_hole,
        })
    }
}

impl problem::InvalidEdge for GeoHoleBloom {
    fn is_edge_invalid(&self, edge_from: problem::Point, edge_to: problem::Point) -> bool {
        // fastest path: boundaries
        if edge_from.0 < self.field_min.0 || edge_from.0 > self.field_max.0 || edge_from.1 < self.field_min.1 || edge_from.1 > self.field_max.1 {
            return true;
        }
        if edge_to.0 < self.field_min.0 || edge_to.0 > self.field_max.0 || edge_to.1 < self.field_min.1 || edge_to.1 > self.field_max.1 {
            return true;
        }

        let key = BloomKey::new(edge_from, edge_to);

        // fast path: bloom filter
        for hash_fn_seed in &self.hash_fns_seeds {
            let mut hasher = seahash::SeaHasher::default();
            hash_fn_seed.hash(&mut hasher);
            key.hash(&mut hasher);
            let hash = hasher.finish();

            let bit_index = (hash % self.bits.len() as u64) as usize;
            if !self.bits[bit_index] {
                return false;
            }
        }

        // slow path: actually check polygon
        self.geo_hole.is_edge_invalid(edge_from, edge_to)
    }
}

#[derive(Hash)]
struct BloomKey {
    edge_min: problem::Point,
    edge_max: problem::Point,
}

impl BloomKey {
    fn new(edge_a: problem::Point, edge_b: problem::Point) -> Self {
        if edge_a < edge_b {
            BloomKey { edge_min: edge_a, edge_max: edge_b, }
        } else {
            BloomKey { edge_min: edge_b, edge_max: edge_a, }
        }
    }
}

#[cfg(test)]
mod tests {
use rand::Rng;
    use crate::{
        problem::{
            self,
            InvalidEdge,
        },
    };
    use super::{
        GeoHoleBloom,
    };

    #[test]
    fn autocheck_on_problem_18() {
        let problem_data = r#"{"bonuses":[{"bonus":"BREAK_A_LEG","problem":67,"position":[42,2]},{"bonus":"WALLHACK","problem":69,"position":[15,3]},{"bonus":"SUPERFLEX","problem":30,"position":[48,3]}],"hole":[[28,0],[56,4],[0,4]],"epsilon":0,"figure":{"edges":[[0,1],[0,2],[1,3],[2,3]],"vertices":[[0,20],[20,0],[20,40],[40,20]]}}"#;
        let problem: problem::Problem = serde_json::from_str(problem_data).unwrap();

        let geo_hole_bloom = GeoHoleBloom::new(&problem).unwrap();

        let geo_hole = problem.hole_polygon_f64();

        let mut rng = rand::thread_rng();
        for _ in 0 .. 10000 {
            let pa = problem::Point(
                rng.gen_range(0 .. 84),
                rng.gen_range(0 .. 20),
            );
            let pb = problem::Point(
                rng.gen_range(0 .. 84),
                rng.gen_range(0 .. 20),
            );
            let orig = geo_hole.is_edge_invalid(pa, pb);
            let test = geo_hole_bloom.is_edge_invalid(pa, pb);
            assert_eq!(orig, test);
        }
    }
}
