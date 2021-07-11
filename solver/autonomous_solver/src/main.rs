use std::{
    fs,
    io,
    thread,
    sync::{
        mpsc,
    },
    path::{
        PathBuf,
        Component,
    },
};

use structopt::{
    StructOpt,
};

use common::{
    problem,
    solver,
};

#[derive(Clone, StructOpt, Debug)]
pub struct CliArgs {
    /// input directory with problems
    #[structopt(long = "problems-directory", default_value = "./tasks")]
    pub problems_directory: PathBuf,
    /// input directory with poses
    #[structopt(long = "poses-directory", default_value = "./poses")]
    pub poses_directory: PathBuf,

    /// api token for submit authorization
    #[structopt(long = "api-token", default_value = "29a3adf2-b0d3-4166-8891-9c990df11546")]
    pub api_token: String,

    /// worker slaves count
    #[structopt(long = "worker-slaves-count", default_value = "4")]
    pub worker_slaves_count: usize,

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
    /// cooling step base temperature
    #[structopt(long = "cooling-step-temp", default_value = "1.0")]
    pub cooling_step_temp: f64,
}


#[derive(Debug)]
pub enum Error {
    FsReadDir { directory: PathBuf, error: io::Error, },
    FsDirEntry { directory: PathBuf, error: io::Error, },
    ProblemLoad(problem::FromFileError),
    SolverCreate(solver::CreateError),
    PoseExport(problem::WriteFileError),
    WorkerSpawn(io::Error),
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();
    let cli_args = CliArgs::from_args();
    log::info!("program starts as: {:?}", cli_args);

    loop {
        let mut problems = sync_problems_directory(&cli_args)?;

        let (slaves_tx, slaves_rx) = mpsc::channel();
        let mut current_workers_count = 0;

        loop {
            if current_workers_count == 0 && problems.is_empty() {
                break;
            }
            if current_workers_count >= cli_args.worker_slaves_count || problems.is_empty() {
                let task_id = slaves_rx.recv().unwrap()?;
                log::info!("slave done with task = {}", task_id);
                current_workers_count -= 1;
                continue;
            }

            let problem = problems.pop().unwrap();
            let slaves_tx = slaves_tx.clone();
            let cli_args = cli_args.clone();
            thread::Builder::new()
                .name(format!("autonomous_solver worker for {:?}", problem.task_id))
                .spawn(move || slave_run(slaves_tx, problem, cli_args))
                .map_err(Error::WorkerSpawn)?;
        }

        return Ok(())
    }
}

#[derive(Debug)]
struct ProblemDesc {
    problem_file: PathBuf,
    pose_file: PathBuf,
    task_id: String,
}

fn slave_run(slaves_tx: mpsc::Sender<Result<String, Error>>, problem: ProblemDesc, cli_args: CliArgs) {
    slaves_tx.send(
        match slave_run_task(&problem, &cli_args) {
            Ok(()) =>
                Ok(problem.task_id),
            Err(error) =>
                Err(error),
        }
    ).ok();
}

fn slave_run_task(problem: &ProblemDesc, cli_args: &CliArgs) -> Result<(), Error> {

    todo!()
}

fn sync_problems_directory(cli_args: &CliArgs) -> Result<Vec<ProblemDesc>, Error> {
    let mut problems = Vec::new();

    let dir_entries = fs::read_dir(&cli_args.problems_directory)
        .map_err(|error| Error::FsReadDir { directory: cli_args.problems_directory.clone(), error, })?;
    for maybe_dir_entry in dir_entries {
        let dir_entry = maybe_dir_entry
            .map_err(|error| Error::FsDirEntry { directory: cli_args.problems_directory.clone(), error, })?;
        let problem_path = dir_entry.path();
        if let Some(Component::Normal(problem_file)) = problem_path.components().last() {
            if let Some(problem_file_str) = problem_file.to_str() {
                let mut split_iter = problem_file_str.split('.');
                if let Some(task_id) = split_iter.next() {
                    if let Some("problem") = split_iter.next() {
                        if let None = split_iter.next() {
                            let mut pose_file = cli_args.poses_directory.clone();
                            pose_file.push(format!("{}.pose", task_id));

                            problems.push(ProblemDesc {
                                task_id: task_id.to_string(),
                                problem_file: problem_path,
                                pose_file,
                            });
                        }
                    }
                }
            }
        }
    }
    Ok(problems)
}
