use structopt::{
    StructOpt,
};

use common::{
    cli,
    problem,
    solver,
};

#[derive(Clone, StructOpt, Debug)]
pub struct CliArgs {
    #[structopt(flatten)]
    pub common: cli::CommonCliArgs,

    /// collect bonus for this task
    #[structopt(long = "collect-bonus-problem")]
    pub collect_bonus_problem: Option<usize>,
    /// use given bonus during solving (json format)
    #[structopt(long = "unlocked-bonus")]
    pub unlocked_bonus: Option<String>,

    /// maximum reheats count
    #[structopt(long = "max-reheats-count", default_value = "5")]
    pub max_reheats_count: usize,
    /// reheat maximum temperature factor [0.0 - 1.0]
    #[structopt(long = "reheat-factor", default_value = "0.33")]
    pub reheat_factor: f64,
    /// iterations count per one cooling step
    #[structopt(long = "iterations-per-cooling-step", default_value = "32768")]
    pub iterations_per_cooling_step: usize,
    /// addition probability of valid edge mutation
    #[structopt(long = "valid-edge-accept-prob", default_value = "0.5")]
    pub valid_edge_accept_prob: f64,
    /// frozen edges swap probability
    #[structopt(long = "frozen-swap-prob", default_value = "0.15")]
    pub frozen_swap_prob: f64,
    /// cooling step base temperature
    #[structopt(long = "cooling-step-temp", default_value = "1.0")]
    pub cooling_step_temp: f64,
}


#[derive(Debug)]
pub enum Error {
    ProblemLoad(problem::FromFileError),
    SolverCreate(solver::CreateError),
    PoseExport(problem::WriteFileError),
    IncorrectBonus(serde_json::Error),
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();
    let cli_args = CliArgs::from_args();
    log::info!("program starts as: {:?}", cli_args);

    let problem = problem::Problem::from_file(&cli_args.common.problem_file)
        .map_err(Error::ProblemLoad)?;
    log::debug!(" ;; problem loaded: {:?}", problem);

    let pose = problem::Pose::from_file(&cli_args.common.pose_file).ok();
    log::debug!(" ;; pose loaded: {:?}", pose);

    let unlocked_bonus: Option<problem::PoseBonus> = if let Some(bonus) = cli_args.unlocked_bonus {
        Some(serde_json::from_str(&bonus).map_err(Error::IncorrectBonus)?)
    } else {
        None
    };

    let mut solver = solver::simulated_annealing::SimulatedAnnealingSolver::new(
        solver::Solver::with_bonus(
            &problem,
            pose,
            match unlocked_bonus {
                None =>
                    None,
                Some(problem::PoseBonus::BreakALeg { .. }) =>
                    Some(problem::ProblemBonusType::BreakALeg),
                Some(problem::PoseBonus::Globalist { .. }) =>
                    Some(problem::ProblemBonusType::Globalist),
                Some(problem::PoseBonus::Wallhack { .. }) =>
                    Some(problem::ProblemBonusType::Wallhack),
                Some(problem::PoseBonus::Superflex { .. }) =>
                    Some(problem::ProblemBonusType::Superflex),
            },
        ).map_err(Error::SolverCreate)?,
        solver::simulated_annealing::Params {
            max_temp: 100.0,
            cooling_step_temp: cli_args.cooling_step_temp,
            minimum_temp: 2.0,
            valid_edge_accept_prob: cli_args.valid_edge_accept_prob,
            frozen_swap_prob: cli_args.frozen_swap_prob,
            iterations_per_cooling_step: cli_args.iterations_per_cooling_step,
            operating_mode: match cli_args.collect_bonus_problem {
                Some(problem_id) =>
                    solver::simulated_annealing::OperatingMode::BonusCollector {
                        target_problem: problem::ProblemId(problem_id),
                    },
                None =>
                    solver::simulated_annealing::OperatingMode::ScoreMaximizer,
            },
        },
    );

    let mut reheats_count = 0;
    let mut best_solution = None;
    loop {
        match solver.step() {
            Ok(()) =>
                (),
            Err(solver::simulated_annealing::StepError::TempTooLow) if reheats_count < cli_args.max_reheats_count => {
                log::info!("temperature is too low: performing reheat ({} left)", cli_args.max_reheats_count - reheats_count);
                solver.reheat(cli_args.reheat_factor);
                reheats_count += 1;
            },
            Err(solver::simulated_annealing::StepError::TempTooLow) => {
                log::info!("annealing done");
                return Ok(());
            },
            Err(solver::simulated_annealing::StepError::ProbablyInfiniteLoopInVertexIndex) => {
                log::error!("probably infinite loop in vertex index stopping");
                return Ok(());
            },
            Err(solver::simulated_annealing::StepError::ProbablyInfiniteLoopInMovedVertex) => {
                log::error!("probably infinite loop in moved vertex stopping");
                return Ok(());
            },
        }
        match solver.fitness() {
            solver::simulated_annealing::Fitness::FigureScored { score, } =>
                if best_solution.map_or(true, |best_score| score < best_score) {
                    best_solution = Some(score);
                    let pose = problem::Pose {
                        vertices: solver.vertices().to_vec(),
                        bonuses: if let Some(bonus) = unlocked_bonus {
                            Some(vec![bonus])
                        } else {
                            None
                        },
                    };
                    pose.write_to_file(&cli_args.common.pose_file)
                        .map_err(Error::PoseExport)?;
                    log::info!("SCORE: {} | new best solution found, pose has been written to {:?}", score, cli_args.common.pose_file);
                },
            solver::simulated_annealing::Fitness::FigureCorrupted { .. } |
            solver::simulated_annealing::Fitness::NotFitHole { .. } =>
                (),
        }
    }
}
