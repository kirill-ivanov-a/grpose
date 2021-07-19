#!/bin/bash
python3 py/collect_init_data.py --first_frame 1552 --last_frame 1651 --frames_per_init 50 --out_dir out/init_mcam_sample --num_repeats 3 ext/multicol-init/build/multi_col_slam demo/data-samples/mcam/multicol_settings demo/data-samples/mcam
