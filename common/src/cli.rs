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
pub struct CommonCliArgs {
    /// input file with problem
    #[structopt(long = "problem-file", default_value = "./tasks/1.problem")]
    pub problem_file: PathBuf,
    /// output file with pose
    #[structopt(long = "pose-file", default_value = "./poses/1.pose")]
    pub pose_file: PathBuf,
}
