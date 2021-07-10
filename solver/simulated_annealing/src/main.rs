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

    /// maximum reheats count
    #[structopt(long = "max-reheats-count", default_value = "5")]
    pub max_reheats_count: usize,
    /// reheat maximum temperature factor [0.0 - 1.0]
    #[structopt(long = "reheat-factor", default_value = "0.33")]
    pub reheat_factor: f64,
}


#[derive(Debug)]
pub enum Error {
    ProblemLoad(problem::FromFileError),
    SolverCreate(solver::CreateError),
    PoseExport(problem::WriteFileError),
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();
    let cli_args = CliArgs::from_args();
    log::info!("program starts as: {:?}", cli_args);

    let problem = problem::Problem::from_file(&cli_args.common.problem_file)
        .map_err(Error::ProblemLoad)?;
    log::debug!(" ;; problem loaded: {:?}", problem);

    let mut solver = solver::simulated_annealing::SimulatedAnnealingSolver::new(
        solver::Solver::new(&problem)
            .map_err(Error::SolverCreate)?,
        solver::simulated_annealing::Params {
            max_temp: 100.0,
            cooling_step_temp: 1.0,
            minimum_temp: 2.0,
            valid_edge_accept_prob: 0.5,
            iterations_per_cooling_step: 10000,
        },
    );

    let mut reheats_count = 0;
    let mut best_solution = None;
    loop {
        match solver.step() {
            Ok(()) =>
                (),
            Err(solver::simulated_annealing::StepError::TempTooLow) if reheats_count < cli_args.max_reheats_count => {
                log::info!("temperature is too low: performing reheat ({} left)", reheats_count);
                solver.reheat(cli_args.reheat_factor);
                reheats_count += 1;
            },
            Err(solver::simulated_annealing::StepError::TempTooLow) => {
                log::info!("annealing done");
                return Ok(());
            }
        }
        match solver.fitness() {
            solver::simulated_annealing::Fitness::FigureScored { score, } =>
                if best_solution.map_or(true, |best_score| score < best_score) {
                    best_solution = Some(score);
                    let pose = problem::Pose {
                        vertices: solver.vertices().to_vec(),
                        bonuses: None,
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
