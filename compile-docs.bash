#!/bin/bash
# Install pydoctor then run this script to compile the docs to ./docs/
pydoctor ./planner \
  --config=pydoctor.cfg \
  --privacy=HIDDEN:planner.algo_bnb \
  --privacy=HIDDEN:planner.canny \
  --privacy=HIDDEN:planner.hough
