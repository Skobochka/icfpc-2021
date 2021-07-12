# icfpc-2021
ICFP Programming Contest 2021

# Team members
* Alexey Voznyuk <me@swizard.info> @swizard0
* Gleb Golubitsky <sectoid@gnolltech.org> @sectoid

**ICFPC 2021 solution for team Skobochka.**

Position 47 in scoreboard after freeze with 602880 points and all 132 problems solved.

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
  - `./common/src/solver/simulated_annealing.rs` : implementation code including several solving modes:
    - `OperatingMode::ScoreMaximizer` : only try to maximize score
    - `OperatingMode::BonusCollector { target_problem }` : try to maximize score while collecting this particular bonus
    - `OperatingMode::BonusHunter` : be sure to collect all available bonuses, then maximize score
    - `OperatingMode::ZeroHunter` : try to obtain zero score for this problem

    There is support for `GLOBALIST`, `WALLHACK` and `SUPERFLEX` bonus. We didn't manage to use `BREAK_A_LEG`.
- Background service for iterative problems solving with results submission: `./solver/autonomous_solver`
  This tries to maximize bonuses collected in each problem in order to use them while solving corresponding target problems.

Every command line utility supports the `--help` flag.
