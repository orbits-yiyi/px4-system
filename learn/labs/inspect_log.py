#!/usr/bin/env python3
"""Validate the isolated 100 -> 200 ms teaching log; requires pyulog and numpy."""
import argparse
from pathlib import Path

import numpy as np
from pyulog import ULog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    args = parser.parse_args()
    log = ULog(str(args.log))
    topic = log.get_dataset('learn_status')
    data = topic.data
    expected = {'timestamp', 'sample_count', 'period_us',
                'requested_period_ms', 'period_limited'}
    if not expected.issubset(data):
        raise RuntimeError(f'missing fields: {expected.difference(data)}')
    if len(data['timestamp']) < 2:
        raise RuntimeError('need at least two samples')
    for name in ('timestamp', 'sample_count'):
        if not np.all(data[name][1:] > data[name][:-1]):
            raise RuntimeError(f'{name} is not strictly increasing')
    periods = set(map(int, data['period_us']))
    if periods != {100000, 200000}:
        raise RuntimeError(f'expected both periods, observed {periods}')
    if not np.all(data['period_us'] == data['requested_period_ms'] * 1000):
        raise RuntimeError('unexpected raw/effective period mismatch')
    if np.any(data['period_limited']):
        raise RuntimeError('unexpected limiting in this normal-input experiment')
    print(f'PASS: {len(data["timestamp"])} learn_status samples')
    print(f'fields: {sorted(data)}')
    print(f'period_us: {sorted(periods)}')
    print(f'timestamp: {int(data["timestamp"][0])} .. {int(data["timestamp"][-1])} us')


if __name__ == '__main__':
    main()
