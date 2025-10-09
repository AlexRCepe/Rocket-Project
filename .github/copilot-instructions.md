## Purpose

Help AI coding agents be immediately productive in this repository. This project contains two small MATLAB/Octave scripts for rocket-related simulations: trajectory and grain modeling. Both main entry points live at the paths below and are currently empty placeholders.

## Repo snapshot (important files)

- `Trajectory code/main.m` — trajectory simulation entry script (MATLAB .m)
- `Grain Code/main.m` — grain geometry/propellant model entry script (MATLAB .m)
- `README.md` — project README (currently a minimal placeholder)

## Big picture and architecture

This is a very small, script-based MATLAB project (no packages or build system). Expect code to be organized as:
- top-level folders containing a `main.m` script that runs a scenario
- helper functions in separate `.m` files in the same folder (or subfolders) if added

There are no services, daemons, or external APIs in the repo. State and data flow are file/script-local: `main.m` should set parameters, call helper functions, and produce figures or numeric output.

## Conventions and patterns to follow (discoverable from repo)

- Use plain MATLAB/Octave `.m` files. Prefer functions over scripts for reusability. If adding a function, put it in a file named `functionName.m` and include a short header comment describing inputs/outputs.
- Keep top-level `main.m` as the scenario runner: it should be idempotent (able to run repeatedly without side-effects) and exit cleanly.
- Use clear units in comments (e.g., meters, seconds, N). Document any physical constants inline or in a single `constants.m` if added.
- Avoid modifying repository-level configuration files. New files should live under existing folders or a new `tests/` or `lib/` folder if adding libraries.

## Running locally

No build system or CI is present. To run the scripts:

- MATLAB (Windows PowerShell):
  - matlab -batch "run(fullfile(pwd,'Trajectory code','main.m'))"
  - matlab -batch "run(fullfile(pwd,'Grain Code','main.m'))"

- Octave (CLI):
  - octave --eval "run('Trajectory code/main.m')"
  - octave --eval "run('Grain Code/main.m')"

Adjust paths to the repository root when invoking from another working directory.

## What an AI agent should do first

1. Confirm the goal with the user if `main.m` files are empty (they are empty now). Ask whether to implement trajectory simulation, grain geometry, or both.
2. When adding code, include small unit tests or a `run_examples.m` script that demonstrates expected numeric output or plots.
3. If creating helper functions, add short doc comments and a usage example in `main.m`.

## Examples (use these concrete file paths)

- If asked to implement a trajectory integrator, add `Trajectory code/propagateTrajectory.m` and call it from `Trajectory code/main.m`.
- If asked to add grain geometry helpers, add `Grain Code/computeGrainArea.m` and a simple example call in `Grain Code/main.m`.

## When to ask the user (explicit clarifying questions)

- Confirm expected inputs/outputs and units for any physics code (mass, thrust, time step).
- Confirm target runtime environment: MATLAB or Octave, and whether plotting is required.
- Confirm desired output format (figures, CSV, MAT-file).

## PR and commit guidance

- Keep changes small and focused (one feature or fix per PR). Include a short test/demo script and update `README.md` when behavior changes.

## Notes and limitations

- There are currently no tests or CI; an agent should not assume any test harness exists. Any added tests must be runnable with MATLAB or Octave commands described above.
- Because the repo is small, prefer interactive questions over making large design assumptions.

---
If anything here is unclear or you want more project background (sample inputs, target outputs, or preferred numerical methods), tell me which part to expand and I will update this file.
