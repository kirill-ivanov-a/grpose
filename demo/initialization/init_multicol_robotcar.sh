#!/bin/bash
python3 py/collect_init_data.py --first_frame 6101 --last_frame 6200 --frames_per_init 50 --out_dir out/init_robotcar_sample --num_repeats 3 ext/multicol-init/build/multi_col_slam demo/data-samples/robotcar/multicol_settings demo/data-samples/robotcar/2015-10-30-11-56-36
