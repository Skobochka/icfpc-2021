use std::{
    collections::BTreeMap,
    fs,
    ffi::OsStr,
};

use common::{
    problem,
};


#[derive(Debug)]
pub enum Error {
    IO(std::io::Error),
    ProblemLoad(problem::FromFileError),
    PoseExport(problem::WriteFileError),
}

fn load_problems() -> Result<BTreeMap<u64, problem::Problem>, Error> {
    let mut problems = BTreeMap::new();

    for path in fs::read_dir("./tasks").map_err(Error::IO)? {
        let problem_path = path.map_err(Error::IO)?.path();
        if problem_path.extension() != Some(OsStr::new("problem")) {
            continue;
        }

        let problem = problem::Problem::from_file(&problem_path)
            .map_err(Error::ProblemLoad)?;

        let problem_key = problem_path.file_stem().unwrap().to_str().unwrap().to_string().parse::<u64>().unwrap();
        problems.insert(problem_key, problem);
        // println!("Found problem {}", problem_path.display());
    }

    Ok(problems)
}

fn task_benefits_from(key: u64, data: &BTreeMap<u64, problem::Problem>) -> Vec<(u64, problem::ProblemBonusType)> {
    let mut benefits = Vec::new();


    for it_key in data.keys() {
        let problem = &data[it_key];
        match &problem.bonuses {
            Some(bonus_vec) => {
                for problem_bonus in bonus_vec.iter() {
                    let problem::ProblemId(problem_key) = problem_bonus.problem;
                    if problem_key as u64 == key {
                        benefits.push((*it_key, problem_bonus.bonus));
                    }
                }
            }
            None => continue,
        }
    }
    benefits
}

fn main() -> Result<(), Error> {
    let problems = load_problems()?;

    println!("|----------|---------------------------------------------------------|---------------------------------------------------------|");
    println!("| Task     | Benefits from                                           | Gives to                                                |");
    println!("|----------|---------------------------------------------------------|---------------------------------------------------------|");
    for key in problems.keys() {
        let benefits = task_benefits_from(*key, &problems).iter()
            .map(|(task_id, bonus)| format!("{} => {:?}", task_id, bonus))
            .collect::<Vec<String>>()
            .join(", ");
        let gives = problems[key].bonuses.as_ref()
            .map_or("".to_string(), |bonus_vec|
                    bonus_vec.iter()
                    .map(|problem::ProblemBonus { position:_position, bonus, problem: problem::ProblemId(problem_key) }|
                         format!("{} => {:?}", problem_key, bonus))
                    .collect::<Vec<String>>()
                    .join(", "));

        println!("| {:>8} | {:<55} | {:<55} |", key, benefits, gives);
        println!("|----------|---------------------------------------------------------|---------------------------------------------------------|");
    }

    Ok(())
}
