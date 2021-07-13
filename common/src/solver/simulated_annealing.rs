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
    pub valid_edge_accept_prob: f64,
    pub frozen_swap_prob: f64,
    pub iterations_per_cooling_step: usize,
    pub operating_mode: OperatingMode,
}

#[derive(Clone, Copy, Debug)]
pub enum OperatingMode {
    ScoreMaximizer,
    BonusCollector {
        target_problem: problem::ProblemId,
    },
    BonusHunter,
    ZeroHunter,
}

pub struct SimulatedAnnealingSolver {
    solver: solver::Solver,
    params: Params,
    vertices_cur: Vec<problem::Point>,
    vertices_tmp: Vec<problem::Point>,
    frozen_vertices_indices: Vec<usize>,
    fitness_cur: Fitness,
    temp: f64,
    steps: usize,
}

#[derive(Clone, Copy, PartialEq, Debug)]
pub enum Fitness {
    FigureCorrupted { ratio_avg: f64, },
    NotFitHole { bad_edges_count: usize, },
    FigureScored { score: i64, },
}

#[derive(Debug)]
pub enum StepError {
    TempTooLow,
    ProbablyInfiniteLoopInVertexIndex,
    ProbablyInfiniteLoopInMovedVertex,
    ProbablyInfiniteLoopInFrozenIndex,
    GenerateVertices(GenerateVerticesError),
}

#[derive(Debug)]
pub enum CreateError {
    GenerateVertices(GenerateVerticesError),
}

impl SimulatedAnnealingSolver {
    pub fn new(solver: solver::Solver, params: Params) -> Result<SimulatedAnnealingSolver, CreateError> {
        let mut vertices_cur = Vec::new();
        let mut frozen_vertices_indices = Vec::new();
        generate_vertices(&solver, &mut vertices_cur, &mut frozen_vertices_indices, params.operating_mode)
            .map_err(CreateError::GenerateVertices)?;

        let temp = params.max_temp;
        let fitness_cur = Fitness::calc(&solver.problem, &solver.geo_hole, &vertices_cur, &solver.use_bonus);

        Ok(SimulatedAnnealingSolver {
            solver,
            params,
            vertices_cur,
            vertices_tmp: Vec::new(),
            frozen_vertices_indices,
            fitness_cur,
            temp,
            steps: 0,
        })
    }

    pub fn reset(&mut self) -> Result<(), GenerateVerticesError> {
        generate_vertices(&self.solver, &mut self.vertices_cur, &mut self.frozen_vertices_indices, self.params.operating_mode)?;
        self.temp = self.params.max_temp;
        self.steps = 0;
        self.fitness_cur = Fitness::calc(&self.solver.problem, &self.solver.geo_hole, &self.vertices_cur, &self.solver.use_bonus);
        Ok(())
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
            if !self.frozen_vertices_indices.is_empty() && rng.gen_range(0.0 .. 1.0) < self.params.frozen_swap_prob {
                let frozen_index = rng.gen_range(0 .. self.frozen_vertices_indices.len());
                let mut counter = 0;
                let pose_vertices_index = loop {
                    counter += 1;
                    if counter > 10000000 {
                        return Err(StepError::ProbablyInfiniteLoopInFrozenIndex);
                    }

                    let index = rng.gen_range(0 .. self.vertices_tmp.len());
                    if !self.frozen_vertices_indices.contains(&index) {
                        break index;
                    }
                };

                let prev_index = self.frozen_vertices_indices[frozen_index];
                let curr_index = pose_vertices_index;

                self.vertices_tmp.swap(prev_index, curr_index);
                let fitness_tmp = Fitness::calc(&self.solver.problem, &self.solver.geo_hole, &self.vertices_tmp, &self.solver.use_bonus);

                let energy_cur = self.fitness_cur.energy();
                let q_cur = energy_cur * self.params.max_temp * self.solver.problem.figure.edges.len() as f64;
                let energy_tmp = fitness_tmp.energy();
                let q_tmp = energy_tmp * self.params.max_temp * self.solver.problem.figure.edges.len() as f64;

                let accept_prob = if q_tmp < q_cur {
                    1.0
                } else {
                    (-(q_tmp - q_cur) / self.temp).exp()
                };
                if rng.gen_range(0.0 .. 1.0) < accept_prob {
                    // accept
                    self.frozen_vertices_indices[frozen_index] = pose_vertices_index;
                    self.vertices_cur.swap(prev_index, curr_index);
                    self.fitness_cur = fitness_tmp;
                } else {
                    // reject
                    self.vertices_tmp.swap(prev_index, curr_index);
                }
            } else {
                let mut counter = 0;
                let vertex_index = loop {
                    counter += 1;
                    if counter > 10000000 {
                        return Err(StepError::ProbablyInfiniteLoopInVertexIndex);
                    }

                    let edge_index = rng.gen_range(0 .. self.solver.problem.figure.edges.len());
                    let edge = &self.solver.problem.figure.edges[edge_index];
                    if self.solver.use_bonus.is_none() {
                        let (is_valid, _ratio) = solver::is_edge_ratio_valid(
                            edge,
                            &self.vertices_tmp,
                            &self.solver.problem,
                        );
                        if is_valid {
                            let accept_prob = rng.gen_range(0.0 .. 1.0);
                            if accept_prob >= self.params.valid_edge_accept_prob {
                                continue;
                            }
                        }
                    }
                    let try_index = if rng.gen_range(0.0 .. 1.0) < 0.5 {
                        edge.0
                    } else {
                        edge.1
                    };
                    if !self.frozen_vertices_indices.contains(&try_index) {
                        break try_index;
                    }
                };
                // let vertex_index = rng.gen_range(0 .. self.vertices_tmp.len());
                let vertex = self.vertices_tmp[vertex_index];

                let mut counter = 0;
                let moved_vertex = loop {
                    counter += 1;
                    if counter > 10000000 {
                        return Err(StepError::ProbablyInfiniteLoopInMovedVertex);
                    }

                    let x = vertex.0 + rng.gen_range(-1 ..= 1);
                    let y = vertex.1 + rng.gen_range(-1 ..= 1);
                    let try_vertex = problem::Point(x, y);

                    let allow_hole = match self.solver.use_bonus {
                        None |
                        Some(problem::ProblemBonusType::BreakALeg) |
                        Some(problem::ProblemBonusType::Globalist) |
                        Some(problem::ProblemBonusType::Superflex) =>
                            self.solver.is_hole(&try_vertex),
                        Some(problem::ProblemBonusType::Wallhack) => {
                            let mut already_has_outside = false;
                            for vertex in &self.vertices_tmp {
                                if !self.solver.is_hole(vertex) {
                                    already_has_outside = true;
                                    break;
                                }
                            }
                            if already_has_outside {
                                self.solver.is_hole(&try_vertex)
                            } else {
                                true
                            }
                        },
                    };

                    if try_vertex != vertex && allow_hole {
                        break try_vertex;
                    }
                };
                self.vertices_tmp[vertex_index] = moved_vertex;
                let fitness_tmp = Fitness::calc(&self.solver.problem, &self.solver.geo_hole, &self.vertices_tmp, &self.solver.use_bonus);

                let energy_cur = self.fitness_cur.energy();
                let q_cur = energy_cur * self.params.max_temp * self.solver.problem.figure.edges.len() as f64;
                let energy_tmp = fitness_tmp.energy();
                let q_tmp = energy_tmp * self.params.max_temp * self.solver.problem.figure.edges.len() as f64;

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
        }

        let temp_delta = (self.temp * 2.0 / self.params.max_temp) * self.params.cooling_step_temp;

        self.temp -= temp_delta;
        self.steps += 1;
        Ok(())
    }
}

#[derive(Debug)]
pub enum GenerateVerticesError {
    ProbablyInfiniteLoopInFrozenIndexInBonusCollector,
    ProbablyInfiniteLoopInFrozenIndexInBonusHunter,
    ProbablyInfiniteLoopInFrozenIndexInZeroHunter,
}

fn generate_vertices(
    solver: &solver::Solver,
    vertices: &mut Vec<problem::Point>,
    frozen_vertices_indices: &mut Vec<usize>,
    operating_mode: OperatingMode,
)
    -> Result<(), GenerateVerticesError>
{
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
    match operating_mode {
        OperatingMode::ScoreMaximizer =>
            (),
        OperatingMode::BonusCollector { target_problem, } =>
            match &solver.problem.bonuses {
                Some(bonuses) if bonuses.is_empty() => {
                    for bonus in bonuses {
                        if bonus.problem != target_problem {
                            continue;
                        }
                        let mut count = 0;
                        let frozen_vertex_index = loop {
                            count += 1;
                            if count > 10000000 {
                                return Err(GenerateVerticesError::ProbablyInfiniteLoopInFrozenIndexInBonusCollector);
                            }

                            let index = rng.gen_range(0 .. vertices.len());
                            if !frozen_vertices_indices.contains(&index) {
                                break index;
                            }
                        };
                        frozen_vertices_indices.push(frozen_vertex_index);
                        vertices[frozen_vertex_index] = bonus.position;
                    }
                },
                Some(..) | None =>
                    (),
            },
        OperatingMode::BonusHunter =>
            match &solver.problem.bonuses {
                Some(bonuses) if !bonuses.is_empty() => {
                    for bonus in bonuses {
                        let mut count = 0;
                        let frozen_vertex_index = loop {
                            count += 1;
                            if count > 10000000 {
                                return Err(GenerateVerticesError::ProbablyInfiniteLoopInFrozenIndexInBonusHunter);
                            }
                            let index = rng.gen_range(0 .. vertices.len());
                            if !frozen_vertices_indices.contains(&index) {
                                break index;
                            }
                        };
                        frozen_vertices_indices.push(frozen_vertex_index);
                        vertices[frozen_vertex_index] = bonus.position;
                    }
                },
                Some(..) | None =>
                    (),
            },
        OperatingMode::ZeroHunter =>
            for &hole_vertex in &solver.problem.hole {
                let mut count = 0;
                let frozen_vertex_index = loop {
                    count += 1;
                    if count > 10000000 {
                        return Err(GenerateVerticesError::ProbablyInfiniteLoopInFrozenIndexInZeroHunter);
                    }
                    let index = rng.gen_range(0 .. vertices.len());
                    if !frozen_vertices_indices.contains(&index) {
                        break index;
                    }
                };
                frozen_vertices_indices.push(frozen_vertex_index);
                vertices[frozen_vertex_index] = hole_vertex;
            },
    }
    Ok(())
}

impl Fitness {
    fn calc(
        problem: &problem::Problem,
        geo_hole: &geo::Polygon<f64>,
        vertices: &[problem::Point],
        use_bonus: &Option<problem::ProblemBonusType>,
    )
        -> Self
    {
        let maybe_pose_bonus = match use_bonus {
            None =>
                None,
            Some(problem::ProblemBonusType::BreakALeg) =>
                todo!(),
            Some(problem::ProblemBonusType::Globalist) =>
                Some(problem::PoseBonus::Globalist { problem: problem::ProblemId(0), }),
            Some(problem::ProblemBonusType::Wallhack) =>
                Some(problem::PoseBonus::Wallhack { problem: problem::ProblemId(0), }),
            Some(problem::ProblemBonusType::Superflex) =>
                Some(problem::PoseBonus::Superflex { problem: problem::ProblemId(0), }),
        };

        match problem.score_vertices(geo_hole, vertices, maybe_pose_bonus) {
            Ok(score) =>
                Fitness::FigureScored { score, },
            Err(problem::PoseValidationError::VerticeCountMismatch) =>
                panic!("unexpected PoseValidationError::VerticeCountMismatch on vertices_cur.len() = {}", vertices.len()),
            Err(problem::PoseValidationError::BrokenEdgesFound { ratio_sum, .. }) => {
                let ratio_avg = ratio_sum / problem.figure.edges.len() as f64;
                Fitness::FigureCorrupted { ratio_avg, }
            },
            Err(problem::PoseValidationError::EdgesNotFitHole(not_fit_edges)) =>
                Fitness::NotFitHole { bad_edges_count: not_fit_edges, },
        }
    }

    pub fn energy(&self) -> f64 {
        match self {
            &Fitness::FigureScored { score, } if score == 0 =>
                0.0,
            &Fitness::FigureScored { score, } =>
                2.0 - (1.0 / score as f64),
            &Fitness::FigureCorrupted { ratio_avg, } =>
                if ratio_avg < 1.0 {
                    2.0 + ratio_avg
                } else {
                    4.0 - (1.0 / ratio_avg)
                },
            &Fitness::NotFitHole { bad_edges_count, } =>
                4.0 + bad_edges_count as f64, // / problem.figure.edges.len() as f64),
        }
    }
}
