#!/usr/bin/env bash
set -e

python ..Headless-Tools/live_viewer.py \
  --port 14660 \
  --range 24 \
  --zmin 0 \
  --zmax 25
