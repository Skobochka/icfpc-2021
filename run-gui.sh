#!/bin/bash

cargo build
exec ./target/debug/app --border-width 100 --problem-file ./tasks/$1.problem  --pose-file ./poses/$1.pose
