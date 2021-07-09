use std::{
    path::PathBuf,
};

use structopt::{
    clap::{
        AppSettings,
    },
    StructOpt,
};

#[derive(Clone, StructOpt, Debug)]
#[structopt(setting = AppSettings::DeriveDisplayOrder)]
#[structopt(setting = AppSettings::AllowLeadingHyphen)]
pub struct CliArgs {
    /// file with problem
    #[structopt(long = "problem-file", default_value = "./tasks/1.problem")]
    pub problem_file: PathBuf,
}
