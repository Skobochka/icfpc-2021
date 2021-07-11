#!/bin/bash

RUST_LOG="app=debug,common=debug,solver_bruteforce=debug" cargo run --release --bin solver-bruteforce-hole -- --problem-file ./tasks/$1.problem --pose-file ./poses/$1.pose ${@:2}
