#!/bin/bash
set -e
echo "Installing project..."
poetry install -v
echo "Running checks..."
poetry run ./scripts/check.sh
