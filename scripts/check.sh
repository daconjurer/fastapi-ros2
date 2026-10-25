#! /bin/bash
set -e
echo "ruff check --diff ."
ruff check --diff .
echo "ruff format --diff ."
ruff format --diff .
echo "basedpyright"
basedpyright
echo "pytest"
pytest
