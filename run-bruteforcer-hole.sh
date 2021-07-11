#!/bin/bash

RUST_LOG="app=debug,common=debug,solver_bruteforce=debug" ./target/release/solver-bruteforce-hole --problem-file ./tasks/$1.problem --pose-file ./poses/$1.pose
