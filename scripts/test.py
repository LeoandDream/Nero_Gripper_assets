"""Unified static and dynamic validation; optional persisted control acceptance."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys
import unittest

ROOT=Path(__file__).resolve().parents[1]


def discover_suites(layer='all'):
    if layer not in ['all','static','dynamic']:raise ValueError('Unknown validation layer')
    locations={'static':ROOT/'tests','dynamic':ROOT/'experiments/dynamics'}
    return {name:unittest.TestLoader().discover(str(path),pattern='test_*.py',top_level_dir=str(path))
            for name,path in locations.items() if layer in ['all',name]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--layer',choices=['all','static','dynamic'],default='all')
    parser.add_argument('--control',action='store_true',help='also persist the full21phase/37000step control acceptance')
    parser.add_argument('--run-id',help='unique run directory for --control')
    args=parser.parse_args()
    if args.run_id and not args.control:parser.error('--run-id requires --control')
    success=True
    for name,suite in discover_suites(args.layer).items():
        print(f'Validation layer: {name} ({suite.countTestCases()} tests)',flush=True)
        result=unittest.TextTestRunner(verbosity=2).run(suite)
        success=result.wasSuccessful() and success
    if success and args.control:
        run_id=args.run_id or 'control_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        result=subprocess.run([sys.executable,str(ROOT/'experiments/dynamics/run.py'),'--stage','control','--filtered','--run-id',run_id],cwd=ROOT)
        success=result.returncode==0
    return 0 if success else 1


if __name__=='__main__':raise SystemExit(main())
