use rand::Rng;

use crate::{
    solver,
    problem,
};

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub max_temp: f64,
    pub cooling_step_temp: f64,
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
        for _ in 0 .. self.params.iterations_per_cooling_step {
            generate_vertices(&self.solver, &mut self.vertices_tmp);
            let fitness_tmp = Fitness::calc(&self.solver.problem, &self.vertices_tmp);



        }
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

    fn is_better_than(&self, other: &Self) -> bool {
        match (self, other) {
            (Fitness::FigureScored { score: score_a, }, Fitness::FigureScored { score: score_b, }) =>
                score_a > score_b,
            (Fitness::FigureScored { .. }, _) =>
                true,
            (_, Fitness::FigureScored { .. }) =>
                false,
            (Fitness::NotFitHole { bad_edges_count: bad_edges_count_a, }, Fitness::NotFitHole { bad_edges_count: bad_edges_count_b, }) =>
                bad_edges_count_a < bad_edges_count_b,
            (Fitness::NotFitHole { .. }, Fitness::FigureCorrupted { .. }) =>
                true,
            (Fitness::FigureCorrupted { .. }, Fitness::NotFitHole { .. }) =>
                false,
            (Fitness::FigureCorrupted { ratio_sum: ratio_sum_a, }, Fitness::FigureCorrupted { ratio_sum: ratio_sum_b, }) =>
                ratio_sum_a < ratio_sum_b,
        }
    }
}
