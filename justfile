# Lint. Fixes in place by default; pass fix=false for CI mode (report and fail, change nothing).
lint fix="true":
    uv run ruff check {{ if fix == "true" { "--fix" } else { "--no-fix" } }}

# Format. Pass diff=1 to check without writing (CI mode).
format diff="":
    uv run ruff format {{ if diff != "" { "--diff" } else { "" } }}

# basedpyright in its `recommended` mode, so warnings fail too.
type-check:
    uv run basedpyright

# Static checks in CI mode (no autofix): lint, format, type-check.
static:
    just lint fix=false
    just format diff=1
    just type-check

# pytest, including the doctests (see pyproject.toml). Needs ROS sourced.
test:
    uv run pytest

# Everything CI runs, without changing any file.
check: static test

# `just check` in the app image, from a host without ROS.
check-in-docker:
    docker compose build api
    docker compose run --rm --no-deps api just check
