#!/bin/bash
set -e
poetry install -v
./scripts/check.sh
