use common::{
    problem,
};


#[derive(Debug)]
pub enum Error {
    ProblemLoad(problem::FromFileError),
    PoseExport(problem::WriteFileError),
}

fn main() -> Result<(), Error> {
    Ok(())
}
