#!/usr/bin/env bash
set -e

python ../test_offboard_figure8_3d.py \
  --speed 10.0 \
  --viz-udp 127.0.0.1:14660
