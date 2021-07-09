use structopt::StructOpt;

use common::{
    cli,
    problem,
};

#[derive(Debug)]
pub enum Error {
    LoadProblem(problem::FromFileError),
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();
    let cli_args = cli::CliArgs::from_args();
    log::info!("program starts as: {:?}", cli_args);

    let problem = problem::Problem::from_file(&cli_args.problem_file)
        .map_err(Error::LoadProblem)?;

    println!(" ;; problem loaded: {:?}", problem);

    Ok(())
}
