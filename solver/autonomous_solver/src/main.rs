use std::{
    fs,
    io,
    time,
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

use rand::prelude::SliceRandom;

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
    /// worker solving timeout in seconds
    #[structopt(long = "worker-solving-timeout-s", default_value = "600")]
    pub worker_solving_timeout_s: u64,

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
    PoseLoad(problem::FromFileError),
    LoadPoseInvalidContent(problem::PoseValidationError),
    SolverCreate(solver::CreateError),
    PoseExport(problem::WriteFileError),
    PoseSerialize(serde_json::Error),
    WorkerSpawn(io::Error),
    WebClientBuilder(reqwest::Error),
    WebClientSend(reqwest::Error),
    WebClientHeader(reqwest::header::InvalidHeaderValue),
}

fn main() -> Result<(), Error> {
    pretty_env_logger::init();
    let cli_args = CliArgs::from_args();
    log::info!("program starts as: {:?}", cli_args);

    loop {
        let mut available_problems = sync_problems_directory(&cli_args)?;
        available_problems.problems.shuffle(&mut rand::thread_rng());

        gather_unlocked_bonuses(&mut available_problems.problems)?;

        let (slaves_tx, slaves_rx) = mpsc::channel();
        let mut current_workers_count = 0;
        let mut tasks_done = 0;

        loop {
            if current_workers_count == 0 && available_problems.problems.is_empty() {
                break;
            }
            if current_workers_count >= cli_args.worker_slaves_count || available_problems.problems.is_empty() {
                let task_id = slaves_rx.recv().unwrap()?;
                current_workers_count -= 1;
                tasks_done += 1;
                log::info!("slave done with task = {}; current_workers_count = {}, tasks_done = {}", task_id, current_workers_count, tasks_done);
                continue;
            }

            let problem = available_problems.problems.pop().unwrap();
            let slaves_tx = slaves_tx.clone();
            let cli_args = cli_args.clone();
            thread::Builder::new()
                .name(format!("autonomous_solver worker for {:?}", problem.task_id))
                .spawn(move || slave_run(slaves_tx, problem, cli_args))
                .map_err(Error::WorkerSpawn)?;
            current_workers_count += 1;
        }

        log::info!("directory processing finished, {} tasks done", tasks_done);
    }
}

#[derive(Debug)]
struct ProblemDesc {
    problem_file: PathBuf,
    pose_file: PathBuf,
    task_id: String,
    unlocked_bonuses: Vec<problem::ProblemBonusType>,
}

struct AvailableProblems {
    problems: Vec<ProblemDesc>,
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

fn slave_run_task(problem_desc: &ProblemDesc, cli_args: &CliArgs) -> Result<(), Error> {

    let problem = problem::Problem::from_file(&problem_desc.problem_file)
        .map_err(Error::ProblemLoad)?;

    let mut has_unlocked_bonuses = false;
    let maybe_pose_score = match problem::Pose::from_file(&problem_desc.pose_file) {
        Ok(pose) => {
            if let Some(ref bonuses) = problem.bonuses {
                for bonus in bonuses {
                    if pose.vertices.iter().find(|v| v == &&bonus.position).is_some() {
                        has_unlocked_bonuses = true;
                        log::debug!("task {} has unlocked bonuses", problem_desc.task_id);
                        break;
                    }
                }
            }
            match problem.score_pose(&pose) {
                Ok(0) => {
                    log::info!("skipping task {} because of zero score", problem_desc.task_id);
                    return Ok(());
                },
                Ok(score) =>
                    Some((pose, score)),
                Err(error) =>
                    return Err(Error::LoadPoseInvalidContent(error)),
            }
        },
        Err(problem::FromFileError::OpenFile(error)) if error.kind() == io::ErrorKind::NotFound =>
            None,
        Err(error) =>
            return Err(Error::PoseLoad(error)),
    };

    let (mut best_solution, maybe_pose) = match maybe_pose_score {
        Some((pose, score)) =>
            (Some(score), Some(pose)),
        None =>
            (None, None),
    };

    let allowed_unlocked_bonuses: Vec<_> = problem_desc
        .unlocked_bonuses
        .iter()
        .filter(|unlocked_bonus| match unlocked_bonus {
            problem::ProblemBonusType::Globalist |
            problem::ProblemBonusType::Superflex |
            problem::ProblemBonusType::Wallhack =>
                true,
            problem::ProblemBonusType::BreakALeg =>
                false,
        })
        .collect();

    if allowed_unlocked_bonuses.is_empty() {
        slave_run_task_with(
            problem_desc,
            &problem,
            &maybe_pose,
            &mut best_solution,
            cli_args,
            None,
            solver::simulated_annealing::OperatingMode::ScoreMaximizer,
        )?;
    } else {
        for &unlocked_bonus in allowed_unlocked_bonuses {
            slave_run_task_with(
                problem_desc,
                &problem,
                &maybe_pose,
                &mut best_solution,
                cli_args,
                Some(unlocked_bonus),
                solver::simulated_annealing::OperatingMode::ScoreMaximizer,
            )?;
        }
    };

    Ok(())
}

fn slave_run_task_with(
    problem_desc: &ProblemDesc,
    problem: &problem::Problem,
    maybe_pose: &Option<problem::Pose>,
    best_solution: &mut Option<i64>,
    cli_args: &CliArgs,
    use_bonus: Option<problem::ProblemBonusType>,
    operating_mode: solver::simulated_annealing::OperatingMode,
)
    -> Result<(), Error>
{
    log::info!(
        "slave started task {}, current pose score: {:?}, use_bonus: {:?}, operating_mode = {:?}",
        problem_desc.task_id,
        best_solution,
        use_bonus,
        operating_mode,
    );

    let mut solver = solver::simulated_annealing::SimulatedAnnealingSolver::new(
        solver::Solver::with_bonus(problem, maybe_pose.as_ref().map(Clone::clone), use_bonus)
            .map_err(Error::SolverCreate)?,
        solver::simulated_annealing::Params {
            max_temp: 100.0,
            cooling_step_temp: cli_args.cooling_step_temp,
            minimum_temp: 2.0,
            valid_edge_accept_prob: cli_args.valid_edge_accept_prob,
            iterations_per_cooling_step: cli_args.iterations_per_cooling_step,
            operating_mode,
        },
    );

    let solving_start_time = time::Instant::now();

    let mut reheats_count = 0;
    let mut submission = None;
    loop {
        if solving_start_time.elapsed().as_secs() > cli_args.worker_solving_timeout_s {
            log::info!("forcing terminate task {} because of timeout {} s", problem_desc.task_id, cli_args.worker_solving_timeout_s);
            break;
        }

        match solver.step() {
            Ok(()) =>
                (),
            Err(solver::simulated_annealing::StepError::TempTooLow) if reheats_count < cli_args.max_reheats_count => {
                // log::debug!(
                //     "temperature is too low for task {}: performing reheat ({} left)",
                //     problem_desc.task_id,
                //     cli_args.max_reheats_count - reheats_count,
                // );
                solver.reheat(cli_args.reheat_factor);
                reheats_count += 1;
            },
            Err(solver::simulated_annealing::StepError::TempTooLow) => {
                log::debug!("annealing done for task {}", problem_desc.task_id);
                break;
            }
        }
        match solver.fitness() {
            solver::simulated_annealing::Fitness::FigureScored { score, } =>
                if best_solution.map_or(true, |best_score| score < best_score) {
                    *best_solution = Some(score);
                    let pose = problem::Pose {
                        vertices: solver.vertices().to_vec(),
                        bonuses: None,
                    };
                    pose.write_to_file(&problem_desc.pose_file)
                        .map_err(Error::PoseExport)?;
                    log::info!(
                        "SCORE: {} | new best solution found for task {}, pose has been written to {:?}",
                        score,
                        problem_desc.task_id,
                        problem_desc.pose_file,
                    );
                    submission = Some((pose, score));
                },
            solver::simulated_annealing::Fitness::FigureCorrupted { .. } |
            solver::simulated_annealing::Fitness::NotFitHole { .. } =>
                (),
        }
    }

    if let Some((pose, score)) = submission {
        let url = format!("https://poses.live/api/problems/{}/solutions", problem_desc.task_id);
        let mut headers = reqwest::header::HeaderMap::new();
        let auth_value = reqwest::header::HeaderValue::from_str(&format!("Bearer {}", cli_args.api_token))
            .map_err(Error::WebClientHeader)?;
        // auth_value.set_sensitive(true);
        headers.insert("Authorization", auth_value);
        let body = serde_json::to_string(&pose)
            .map_err(Error::PoseSerialize)?;

        log::info!(
            "preparing submission for for task {} with score {} to {:?}, headers: {:?}",
            problem_desc.task_id,
            score,
            url,
            headers,
        );

        let send_result = reqwest::blocking::Client::builder()
            .default_headers(headers)
            .build().map_err(Error::WebClientBuilder)?
            .post(&url)
            .body(body)
            .send().map_err(Error::WebClientSend)?;
        log::info!("solution submitted for task = {}, result = {:?}", problem_desc.task_id, send_result);
    }
    Ok(())
}

fn sync_problems_directory(cli_args: &CliArgs) -> Result<AvailableProblems, Error> {
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
                                unlocked_bonuses: vec![],
                            });
                        }
                    }
                }
            }
        }
    }

    Ok(AvailableProblems { problems, })
}

fn gather_unlocked_bonuses(problems: &mut [ProblemDesc]) -> Result<(), Error> {
    for problem_index in 0 .. problems.len() {
        let problem_desc = &problems[problem_index];

        let problem = problem::Problem::from_file(&problem_desc.problem_file)
            .map_err(Error::ProblemLoad)?;
        let available_bonuses = if let Some(bonuses) = problem.bonuses {
            bonuses
        } else {
            continue;
        };

        let pose = match problem::Pose::from_file(&problem_desc.pose_file) {
            Ok(pose) =>
                pose,
            Err(problem::FromFileError::OpenFile(error)) if error.kind() == io::ErrorKind::NotFound =>
                continue,
            Err(error) =>
                return Err(Error::PoseLoad(error)),
        };

        for bonus in available_bonuses {
            if pose.vertices.iter().find(|v| v == &&bonus.position).is_some() {
                let target_task_id = format!("{}", bonus.problem.0);
                if let Some(target_problem) = problems.iter_mut().find(|p| p.task_id == target_task_id) {
                    log::debug!("unlocked {:?} for task {}", bonus, target_task_id);
                    target_problem.unlocked_bonuses.push(bonus.bonus);
                }
            }
        }
    }

    Ok(())
}
