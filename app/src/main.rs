use std::{
    path::PathBuf,
};

use structopt::{
    clap::{
        AppSettings,
    },
    StructOpt,
};

use common::{
    cli,
    problem,
};

#[derive(Clone, StructOpt, Debug)]
#[structopt(setting = AppSettings::DeriveDisplayOrder)]
#[structopt(setting = AppSettings::AllowLeadingHyphen)]
pub struct CliArgs {
    #[structopt(flatten)]
    pub common: cli::CommonCliArgs,

    /// asserts directory
    #[structopt(long = "assets-directory", default_value = "./assets")]
    pub assets_directory: PathBuf,
}

#[derive(Debug)]
pub enum Error {
    LoadProblem(problem::FromFileError),
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();
    let cli_args = CliArgs::from_args();
    log::info!("program starts as: {:?}", cli_args);

    let problem = problem::Problem::from_file(&cli_args.common.problem_file)
        .map_err(Error::LoadProblem)?;

    println!(" ;; problem loaded: {:?}", problem);

    Ok(())
}
