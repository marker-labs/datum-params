//! Compiled-artifact CLI over the shared evaluation core.
use anyhow::{bail, Result};
use datum_params_tools::evaluate;
use std::{env, path::Path};
fn main()->Result<()>{
    let args:Vec<String>=env::args().collect();
    if args.len()!=7 || args[1]!="generate" { bail!("Usage: datum-params generate PROJECT_ROOT SPEC_JSON WORKTREE OUTPUT_JSON OUTPUT_HPP") }
    let evaluation=evaluate(Path::new(&args[2]),Path::new(&args[3]),&args[4])?;
    evaluation.write_artifacts(Path::new(&args[5]),Path::new(&args[6]))
}
