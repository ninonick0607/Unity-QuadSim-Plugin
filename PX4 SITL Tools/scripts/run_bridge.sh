#!/usr/bin/env bash
set -e

python ../px4_bridge.py \
  --quadsim-host localhost \
  --px4-port 4560 \
  --hz 250 \
  --speed 0
