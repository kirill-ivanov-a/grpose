#!/usr/bin/python3

from os import system
from pathlib import Path

THIS_SCRIPT_FPATH = Path(__file__).resolve()
THIS_SCRIPT_DIRPATH = THIS_SCRIPT_FPATH.parent

# *** ORB-SLAM3 ***
RUN_ORB_SCRIPT_FPATH = (THIS_SCRIPT_DIRPATH / '../../py/run/orbslam3.py').resolve()
BASE_DIRPATH = (THIS_SCRIPT_DIRPATH / '../').resolve()
DATA_SAMPLES_DIRPATH = BASE_DIRPATH / 'data-samples'
RESULTS_DIRPATH = BASE_DIRPATH / 'results'

# EuRoC
ORB_EUROC_DATASET_NAME = 'V103_500-599'
ORB_EUROC_SAMPLES_BASE_DIRPATH = DATA_SAMPLES_DIRPATH / 'euroc'
ORB_EUROC_OUTPUT_BASE_DIRPATH = RESULTS_DIRPATH / 'euroc'
ORB_EUROC_FRAMES_PER_CHUNK = 20
ORB_EUROC_TIMESTAMPS_FPATH = \
    ORB_EUROC_SAMPLES_BASE_DIRPATH / '{}/timestamps.txt'.format(ORB_EUROC_DATASET_NAME)

# Robotcar
ORB_ROBOTCAR_DATASET_NAME = '2015-10-30-11-56-36_150-249'
ORB_ROBOTCAR_SAMPLES_BASE_DIRPATH = DATA_SAMPLES_DIRPATH / 'robotcar'
ORB_ROBOTCAR_OUTPUT_BASE_DIRPATH = RESULTS_DIRPATH / 'robotcar'
ORB_ROBOTCAR_FRAMES_PER_CHUNK = 20
ORB_ROBOTCAR_TIMESTAMPS_FPATH = \
    ORB_ROBOTCAR_SAMPLES_BASE_DIRPATH / '{}/timestamps.txt'.format(ORB_ROBOTCAR_DATASET_NAME)


def orb_euroc_initialization_demo():
    assert RUN_ORB_SCRIPT_FPATH.exists()
    assert ORB_EUROC_SAMPLES_BASE_DIRPATH.exists()
    assert ORB_EUROC_TIMESTAMPS_FPATH.exists()
    run_orb_cmd = ('{} euroc {} {} {} --frames-per-chunk {} --use-visualizer '
                   '--use-separate-session-per-chunk --timestamps-fpath {}').format(
        RUN_ORB_SCRIPT_FPATH, ORB_EUROC_SAMPLES_BASE_DIRPATH,
        ORB_EUROC_OUTPUT_BASE_DIRPATH, ORB_EUROC_DATASET_NAME,
        ORB_EUROC_FRAMES_PER_CHUNK, ORB_EUROC_TIMESTAMPS_FPATH)
    print(run_orb_cmd)
    system(run_orb_cmd)


def orb_robotcar_initialization_demo():
    assert RUN_ORB_SCRIPT_FPATH.exists()
    assert ORB_ROBOTCAR_SAMPLES_BASE_DIRPATH.exists()
    assert ORB_ROBOTCAR_TIMESTAMPS_FPATH.exists()
    run_orb_cmd = ('{} robotcar {} {} {} --frames-per-chunk {} --use-visualizer '
                   '--use-separate-session-per-chunk --timestamps-fpath {}').format(
        RUN_ORB_SCRIPT_FPATH, ORB_ROBOTCAR_SAMPLES_BASE_DIRPATH,
        ORB_ROBOTCAR_OUTPUT_BASE_DIRPATH, ORB_ROBOTCAR_DATASET_NAME,
        ORB_ROBOTCAR_FRAMES_PER_CHUNK, ORB_ROBOTCAR_TIMESTAMPS_FPATH)
    print(run_orb_cmd)
    system(run_orb_cmd)


def all_initialization_demos():
    orb_euroc_initialization_demo()
    orb_robotcar_initialization_demo()


def main(args):
    all_initialization_demos()


if __name__ == '__main__':
    from argparse import ArgumentParser

    parser = ArgumentParser()

    args = parser.parse_args()
    main(args)
