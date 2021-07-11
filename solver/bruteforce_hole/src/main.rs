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

    let pose = problem::Pose::from_file(&cli_args.common.pose_file).ok();
    log::debug!(" ;; pose loaded: {:?}", pose);

    let mut solver = solver::bruteforce_hole::BruteforceHoleSolver::new(
        solver::Solver::new(&problem, pose)
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
