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
    #[structopt(long = "use-bonus")]
    pub use_bonus: Option<String>,
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

    let mut pose: problem::Pose = problem::Pose::from_file(&cli_args.common.pose_file)
        .ok()
        .unwrap_or_else(|| problem.export_pose());
    log::debug!(" ;; pose loaded: {:?}", pose);

    match cli_args.use_bonus {
        Some(ref a) if a == "GLOBALIST" => {
            pose.bonuses = Some(vec![ problem::PoseBonus::Globalist {
                problem: problem::ProblemId(0), // todo: add correct number here!
            }]);
        },
        Some(ref a) if a == "WALLHACK" => {
            pose.bonuses = Some(vec![ problem::PoseBonus::Wallhack {
                problem: problem::ProblemId(0), // todo: add correct number here!
            }]);
        },
        Some(ref a) if a == "SUPERFLEX" => {
            pose.bonuses = Some(vec![ problem::PoseBonus::Superflex {
                problem: problem::ProblemId(0), // todo: add correct number here!
            }]);
        },
        None => {
            pose.bonuses = None;
        }
        Some(a) => unimplemented!("Unknown bonus type '{}'", a),
    };

    let solver = solver::bruteforce_hole::BruteforceHoleSolver::new(
        solver::Solver::new(&problem, Some(pose))
            .map_err(Error::SolverCreate)?,
        );

    let pose = solver.solve();
    match pose {
        None => {},
        Some(pose) => {
            pose.write_to_file(&cli_args.common.pose_file)
                .map_err(Error::PoseExport)?;
            log::info!("pose {:?} has been written to {:?}", pose, cli_args.common.pose_file);
        }
    }

    Ok(())
}
