
use crate::{
    problem,
};

pub struct GeoHoleBloom {
    bits: bit_vec::BitVec,

}

impl GeoHoleBloom {
    pub fn new(problem: &problem::Problem, bits_count: usize) -> GeoHoleBloom {
        let bits = bit_vec::BitVec::from_elem(bits_count, false);

        GeoHoleBloom {
            bits,
        }
    }
}
