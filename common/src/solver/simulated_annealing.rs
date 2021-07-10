use rand::Rng;

use crate::{
    solver,
    problem,
};

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub max_temp: f64,
    pub cooling_step_temp: f64,
    pub minimum_temp: f64,
    pub iterations_per_cooling_step: usize,
}

pub struct SimulatedAnnealingSolver {
    solver: solver::Solver,
    params: Params,
    vertices_cur: Vec<problem::Point>,
    vertices_tmp: Vec<problem::Point>,
    fitness_cur: Fitness,
    temp: f64,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Fitness {
    FigureCorrupted { ratio_sum: f64, },
    NotFitHole { bad_edges_count: usize, },
    FigureScored { score: i64, },
}

#[derive(Debug)]
pub enum StepError {
    TempTooLow,
}

impl SimulatedAnnealingSolver {
    pub fn new(solver: solver::Solver, params: Params) -> SimulatedAnnealingSolver {
        let mut vertices_cur = Vec::new();
        generate_vertices(&solver, &mut vertices_cur);
        let temp = params.max_temp;
        let fitness_cur = Fitness::calc(&solver.problem, &vertices_cur);

        SimulatedAnnealingSolver {
            solver,
            params,
            vertices_cur,
            vertices_tmp: Vec::new(),
            fitness_cur,
            temp,
        }
    }

    pub fn reset(&mut self) {
        generate_vertices(&self.solver, &mut self.vertices_cur);
        self.temp = self.params.max_temp;
        self.fitness_cur = Fitness::calc(&self.solver.problem, &self.vertices_cur);
    }

    pub fn reheat(&mut self, temp_factor: f64) {
        self.temp = self.params.max_temp * temp_factor;
    }

    pub fn temp(&self) -> f64 {
        self.temp
    }

    pub fn fitness(&self) -> Fitness {
        self.fitness_cur
    }

    pub fn vertices(&self) -> &[problem::Point] {
        &self.vertices_cur
    }

    pub fn step(&mut self) -> Result<(), StepError> {
        if self.temp < self.params.minimum_temp {
            return Err(StepError::TempTooLow);
        }

        self.vertices_tmp.clear();
        self.vertices_tmp.extend(self.vertices_cur.iter().cloned());

        let mut rng = rand::thread_rng();
        for _ in 0 .. self.params.iterations_per_cooling_step {
            let vertex_index = rng.gen_range(0 .. self.vertices_tmp.len());

            let moved_vertex = loop {
                let x = rng.gen_range(self.solver.field_min.0 ..= self.solver.field_max.1);
                let y = rng.gen_range(self.solver.field_min.1 ..= self.solver.field_max.1);
                let vertex = problem::Point(x, y);
                if vertex != self.vertices_tmp[vertex_index] && self.solver.is_hole(&vertex) {
                    break vertex;
                }
            };
            self.vertices_tmp[vertex_index] = moved_vertex;
            let fitness_tmp = Fitness::calc(&self.solver.problem, &self.vertices_tmp);

            let energy_cur = self.fitness_cur.energy();
            let q_cur = energy_cur * self.params.max_temp * 10.0;
            let energy_tmp = fitness_tmp.energy();
            let q_tmp = energy_tmp * self.params.max_temp * 10.0;

            let accept_prob = if q_tmp < q_cur {
                1.0
            } else {
                (-(q_tmp - q_cur) / self.temp).exp()
            };
            if rng.gen_range(0.0 .. 1.0) < accept_prob {
                // accept

                // log::debug!(
                //     "accepted {:?} -> {:?} because fitness_cur = {:?}, fitness_tmp = {:?}, q_cur = {:?}, q_tmp = {:?}, accept_prob = {:?}",
                //     self.vertices_cur[vertex_index],
                //     self.vertices_tmp[vertex_index],
                //     self.fitness_cur,
                //     fitness_tmp,
                //     q_cur,
                //     q_tmp,
                //     accept_prob,
                // );

                self.vertices_cur[vertex_index] =
                    self.vertices_tmp[vertex_index];
                self.fitness_cur = fitness_tmp;
            } else {
                // reject
                self.vertices_tmp[vertex_index] =
                    self.vertices_cur[vertex_index];
            }
        }

        self.temp -= self.params.cooling_step_temp;
        Ok(())
    }
}

fn generate_vertices(solver: &solver::Solver, vertices: &mut Vec<problem::Point>) {
    let figure_vertices_iter = solver
        .problem
        .figure
        .vertices
        .iter();
    let mut rng = rand::thread_rng();
    vertices.clear();
    vertices.extend(
        figure_vertices_iter
            .map(|_vertex| {
                loop {
                    let x = rng.gen_range(solver.field_min.0 ..= solver.field_max.1);
                    let y = rng.gen_range(solver.field_min.1 ..= solver.field_max.1);
                    let point = problem::Point(x, y);
                    if solver.is_hole(&point) {
                        break point;
                    }
                }
            })
    );
}

impl Fitness {
    fn calc(problem: &problem::Problem, vertices: &[problem::Point]) -> Self {
        let mut is_ok = true;
        let mut ratio_sum = 0.0;
        for edge in &problem.figure.edges {
            let sample_vertex_a = problem.figure.vertices[edge.0];
            let sample_vertex_b = problem.figure.vertices[edge.1];

            let try_vertex_a = vertices[edge.0];
            let try_vertex_b = vertices[edge.1];

            let sample_sq_dist = (sample_vertex_a.0 - sample_vertex_b.0) * (sample_vertex_a.0 - sample_vertex_b.0)
                + (sample_vertex_a.1 - sample_vertex_b.1) * (sample_vertex_a.1 - sample_vertex_b.1);
            let try_sq_dist = (try_vertex_a.0 - try_vertex_b.0) * (try_vertex_a.0 - try_vertex_b.0)
                + (try_vertex_a.1 - try_vertex_b.1) * (try_vertex_a.1 - try_vertex_b.1);

            let ratio = ((try_sq_dist as f64 / sample_sq_dist as f64) - 1.0).abs();
            if ratio > problem.epsilon as f64 / 1000000.0 {
                is_ok = false;
            }
            ratio_sum += ratio;
        }
        if is_ok {
            match problem.score_vertices(vertices) {
                Ok(score) =>
                    Fitness::FigureScored { score, },
                Err(problem::PoseValidationError::VerticeCountMismatch) =>
                    panic!("unexpected PoseValidationError::VerticeCountMismatch on vertices_cur.len() = {}", vertices.len()),
                Err(problem::PoseValidationError::BrokenEdgesFound(broken_edges)) =>
                    panic!("unexpected PoseValidationError::Broken_Edges on broken_edges = {:?}", broken_edges),
                Err(problem::PoseValidationError::EdgesNotFitHole(not_fit_edges)) =>
                    Fitness::NotFitHole { bad_edges_count: not_fit_edges.len(), },
            }
        } else {
            Fitness::FigureCorrupted { ratio_sum, }
        }
    }

    pub fn energy(&self) -> f64 {
        match self {
            &Fitness::FigureScored { score, } if score == 0 =>
                0.0,
            &Fitness::FigureScored { score, } =>
                2.0 - (1.0 / score as f64),
            &Fitness::NotFitHole { bad_edges_count, } =>
                3.0 - (1.0 / bad_edges_count as f64),
            &Fitness::FigureCorrupted { ratio_sum, } =>
                if ratio_sum < 1.0 {
                    3.0 + ratio_sum
                } else {
                    5.0 - (1.0 / ratio_sum)
                },
        }
    }
}
