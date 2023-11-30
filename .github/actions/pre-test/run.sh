#!/bin/bash
set -e
poetry install -v
poetry shell
./scripts/check.sh
