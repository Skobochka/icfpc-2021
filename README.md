# icfpc-2021
ICFP Programming Contest 2021

# Team members
* Alexey Voznyuk <me@swizard.info> @swizard0
* Gleb Golubitsky <sectoid@gnolltech.org> @sectoid

**ICFPC 2021 solution for team Skobochka.**

---

You can find the following projects in the repository:

- Problems visualizer / manual solver / solvers debugger: `./app`
- Naive bruteforce solver with some heuristics:
  - `./solver/bruteforce` : command line utility
  - `./common/src/solver/bruteforce.rs` : implementation code
- Zero score bruteforce solver:
  - `./solver/bruteforce_hole` : command line utility
  - `./common/src/solver/bruteforce_hole.rs` : implementation code
- Monte-carlo solver based on [`simulated annealing`]: https://en.wikipedia.org/wiki/Simulated_annealing probabilistic algorithm:
  - `./solver/simulated_annealing` : command line utility
  - `./common/src/solver/simulated_annealing.rs` : implementation code
- Background service for iterative problems solving with results submission: `./solver/autonomous_solver`

Every command line utility supports the `--help` flag.
