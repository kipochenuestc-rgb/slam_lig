#!/usr/bin/env python3
"""Validate the selected raw recording before starting the web mapping job."""
import argparse
import fcntl
import json
import pathlib
import sys

import yaml

import run_slam


def mapping_inputs(bag):
    from bag_tf import inspect_bag
    from fusion_core import TOPICS
    config = run_slam.ROOT / 'config'
    bridge = yaml.safe_load((config / 'bridge.yaml').read_text())['/**']['ros__parameters']
    gnss = bridge['gnss_topic']
    if bag:
        return inspect_bag(bag, gnss)
    topics = [*TOPICS, '/imu/data', gnss]
    names = ['HASCO 1','HASCO 2','HASCO 3','HASCO 4','IMU','GNSS']
    return dict(rows=[dict(name=n,topic=t,count=None,pose=None) for n,t in zip(names,topics)],
                         filesOk=False, source='')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=pathlib.Path, nargs='?')
    parser.add_argument('run', nargs='?')
    parser.add_argument('--inspect', action='store_true')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--rviz', action='store_true')
    args = parser.parse_args()
    bag = args.bag.resolve() if args.bag else None
    if args.inspect:
        inputs = mapping_inputs(bag)
        print(json.dumps(inputs, allow_nan=False))
        return
    if not bag or not args.run:
        parser.error('bag and run required')
    options = ['--mode', 'replay', '--input', 'raw', '--bag', str(bag), '--run', args.run, '--rate', str(args.rate)]
    if not args.rviz:
        options.append('--no-rviz')
    with (run_slam.ROOT / 'runs/.web_mapping.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise ValueError('已有网页建图任务正在运行')
        return run_slam.main(options)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ValueError, OSError, KeyError, yaml.YAMLError) as error:
        sys.exit(str(error))
