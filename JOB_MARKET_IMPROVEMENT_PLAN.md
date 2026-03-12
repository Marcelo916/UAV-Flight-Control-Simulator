# Job-Market Impact Review and Improvement Plan

## Weak points making the project look toy-like
1. **No test suite or CI signal**: there are no automated tests or workflow badges to demonstrate engineering rigor.
2. **Single-demo executable only**: behavior is only visible through console logs in `main.cpp`, with no repeatable experiment scripts or deterministic runs.
3. **Sparse project documentation**: current README is very short and lacks build/run instructions, architecture notes, and expected outcomes.
4. **Control model is intentionally simplistic without context**: proportional-only control and simple physics are fine for a portfolio starter, but the repository does not explain trade-offs or limitations.
5. **No coding-quality gate**: no formatting/linting setup, and no explicit compile flags for warnings.

## Prioritized improvements (small-project realistic)
1. **Add minimal tests for controller behavior** (highest impact for credibility).
2. **Add CI to build + run tests on push/PR** (strong recruiter signal).
3. **Expand README with build/run instructions and architecture sketch**.
4. **Add deterministic simulation mode via optional fixed random seed**.
5. **Add basic build system quality defaults (`-Wall -Wextra -Werror` in CI at least)**.
6. **Add one short "limitations and next steps" section** to frame engineering judgment.

## Recommended milestone plan
### Milestone 1 (1 day): Engineering baseline
- Add unit tests for `Controller::step` and `Sensor` thread-safe state changes.
- Add GitHub Actions CI to compile and run tests.
- Add `CONTRIBUTING` section in README for local test command.

### Milestone 2 (1 day): Better project narrative
- Expand README with:
  - project goal
  - architecture/components
  - compile/run instructions
  - sample output
  - known limitations
- Add fixed-seed CLI option for reproducible behavior in demos.

### Milestone 3 (1–2 days): Professional polish
- Add warning-clean build profile and ensure CI enforces it.
- Add one benchmark/experiment note (e.g., convergence time for different `kp`, noise levels).
- Add lightweight release/tag with changelog notes.

## Best next implementation task for Codex (do immediately)
**Implement a minimal unit test target for control logic and wire it into CI.**

Why this first:
- It yields the strongest trust signal per hour invested.
- It turns the repository from demo-only into an engineering artifact.
- It enables safe future improvements (README, deterministic mode, tuning experiments).

Concrete scope:
- Introduce a tiny C++ test framework (or simple assert-based test binary).
- Add tests for: zero error, positive/negative saturation, and altitude update after step.
- Add CI workflow running build + tests on every push/PR.
