#!/usr/bin/env python3
"""Check an already-running, isolated PX4 instance's teaching module.

Does not launch PX4 or send flight commands. Temporarily changes LEARN_PERIOD,
starts/stops learn_status, and restores the original parameter in finally.
Use only on the course instance with learn_status initially stopped.
"""
import argparse
import re
import subprocess
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bin-dir', required=True, type=Path)
    parser.add_argument('--instance', required=True, type=int)
    args = parser.parse_args()
    if args.instance < 0:
        parser.error('instance must be non-negative')
    binary_dir = args.bin_dir.resolve()

    def command(name, *params, check=True):
        invocation = [str(binary_dir / ('px4-' + name)), '--instance', str(args.instance), *params]
        result = subprocess.run(invocation, text=True, stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT, timeout=8)
        output = re.sub(r'\x1b\[[0-9;]*[A-Za-z]', '', result.stdout)
        print('$', name, *params, flush=True)
        print(output.rstrip(), flush=True)
        if check and result.returncode:
            raise RuntimeError(f'{name} failed: {result.returncode}')
        return result.returncode, output

    status, _ = command('learn_status', 'status', check=False)
    if status == 0:
        raise RuntimeError('learn_status already running; use a stopped, isolated course instance')
    _, raw = command('param', 'show', '-q', 'LEARN_PERIOD')
    original = str(int(raw.strip()))
    started = False

    def sample():
        _, output = command('listener', 'learn_status', '-n', '1')
        values = {}
        for key in ['timestamp', 'sample_count', 'period_us', 'requested_period_ms']:
            match = re.search(r'^\s*' + key + r':\s*(-?\d+)', output, re.MULTILINE)
            if match:
                values[key] = int(match.group(1))
        match = re.search(r'period_limited:\s*(True|False)', output)
        if match:
            values['period_limited'] = match.group(1) == 'True'
        return values

    def await_period(expected, limited=False):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            current = sample()
            if current.get('period_us') == expected and current.get('period_limited') == limited:
                return current
            time.sleep(.12)
        raise AssertionError(f'period {expected} not observed')

    try:
        command('param', 'set', 'LEARN_PERIOD', '100')
        command('learn_status', 'start')
        started = True
        first = await_period(100000)
        time.sleep(.35)
        following = sample()
        assert following['timestamp'] > first['timestamp']
        assert following['sample_count'] > first['sample_count']
        command('param', 'set', 'LEARN_PERIOD', '200')
        await_period(200000)
        command('param', 'set', 'LEARN_PERIOD', '-1')
        clipped = await_period(20000, limited=True)
        assert clipped['requested_period_ms'] == -1
        command('param', 'set', 'LEARN_PERIOD', '100')
        before_stop = await_period(100000)
        command('learn_status', 'stop')
        started = False
        stopped = sample()
        time.sleep(.25)
        assert sample() == stopped, 'publication continued after stop'
        command('learn_status', 'start')
        started = True
        time.sleep(.15)
        restarted = sample()
        assert restarted['timestamp'] > before_stop['timestamp']
        assert restarted['sample_count'] < before_stop['sample_count'], 'counter did not reset'
        print('PASS: publication, parameter update, clamp, stop and restart', flush=True)
    finally:
        if started:
            command('learn_status', 'stop', check=False)
        command('param', 'set', 'LEARN_PERIOD', original)


if __name__ == '__main__':
    main()
