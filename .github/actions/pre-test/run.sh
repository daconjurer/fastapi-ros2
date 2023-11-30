#!/bin/bash
set -e
poetry install -v
poetry run ./scripts/check.sh
