# Git Hooks for Tests and Python Quality

## Summary

Add a version-controlled Git `pre-commit` hook that runs both the unit-test suite and Python quality checks before every commit. Use `uv` for consistent dependency and command execution, matching the existing CI workflow.

## Implementation Changes

- Add Ruff as a development dependency in `pyproject.toml` and update `uv.lock`.
- Configure Ruff for Python 3.12 and the repository's Python sources and tests.
- Create a version-controlled `.githooks/pre-commit` script that:
  - Resolves the repository root.
  - Runs Ruff formatting and linting on staged Python files.
  - Runs the existing coverage-enabled unit-test command used by CI.
  - Stops on the first failure and reports the failing check.
- Document hook installation with `git config core.hooksPath .githooks` and provide manual commands for each check.
- Keep CI's existing test workflow unchanged; repository-wide Ruff cleanup can be
  introduced separately once legacy files have been reformatted.

## Test Plan

- Run the hook successfully against the current repository with no staged Python files.
- Confirm temporary formatting and lint violations are rejected.
- Confirm a failing test is rejected.
- Confirm successful Ruff checks and tests permit a commit.
- Confirm hardware tests remain skipped on non-Raspberry Pi systems.
- Run the CI-equivalent commands directly to verify local and remote validation stay consistent.

## Assumptions

- Both checks belong on `pre-commit`.
- Ruff runs in check-only mode and does not rewrite files during a commit.
- The existing CI pytest command is the authoritative unit-test command.
- `.githooks/` is versioned; each clone must enable it locally.
- Fresh-clone hook installation verification is intentionally omitted from this implementation.
