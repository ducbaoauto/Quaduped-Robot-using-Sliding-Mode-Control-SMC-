#!/usr/bin/env python3
"""
analyze.py — SMC Performance Analysis Master Script
=====================================================
Usage:
    python3 analyze.py                          # uses default CSV path
    python3 analyze.py --csv /path/to/data.csv
    python3 analyze.py --csv data.csv --show    # display plots interactively
    python3 analyze.py --csv data.csv --lambda 2.0

Outputs (saved to smc_evaluation/output/):
    trajectory.png / .pdf
    errors.png     / .pdf
    sliding.png    / .pdf
    cmdvel.png     / .pdf
"""

import argparse
import os
import sys
import time

# Ensure we can import sibling modules regardless of cwd
sys.path.insert(0, os.path.dirname(__file__))

from utils import load_csv, print_metrics_table
import plot_trajectory
import plot_errors
import plot_sliding
import plot_cmdvel

DEFAULT_CSV    = '/home/ducbao/go1_ws/smc_data.csv'
OUTPUT_DIR     = os.path.join(os.path.dirname(__file__), 'output')


def banner(text: str):
    w = 60
    print('\n' + '=' * w)
    print(f"  {text}")
    print('=' * w)


def main():
    parser = argparse.ArgumentParser(
        description='SMC Performance Analysis Toolkit for Unitree GO1')
    parser.add_argument('--csv',    default=DEFAULT_CSV,
                        help='Path to smc_data.csv')
    parser.add_argument('--lambda', dest='lam', type=float, default=1.0,
                        help='Sliding surface gain λ (default: 1.0)')
    parser.add_argument('--show',   action='store_true',
                        help='Display plots interactively (requires display)')
    args = parser.parse_args()

    # ------------------------------------------------------------------ #
    # Validate input
    # ------------------------------------------------------------------ #
    if not os.path.isfile(args.csv):
        print(f"[ERROR] CSV not found: {args.csv}")
        print("        Run the data_logger_node first, then re-run this script.")
        sys.exit(1)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # ------------------------------------------------------------------ #
    # Dataset overview
    # ------------------------------------------------------------------ #
    banner("Loading dataset")
    df = load_csv(args.csv)
    duration   = df['time'].iloc[-1]
    n_samples  = len(df)
    avg_dt     = duration / (n_samples - 1) if n_samples > 1 else 0
    avg_hz     = 1.0 / avg_dt if avg_dt > 0 else 0

    print(f"  File       : {args.csv}")
    print(f"  Samples    : {n_samples}")
    print(f"  Duration   : {duration:.2f} s")
    print(f"  Avg rate   : {avg_hz:.1f} Hz")
    print(f"  Output dir : {OUTPUT_DIR}")

    # ------------------------------------------------------------------ #
    # Run all plots
    # ------------------------------------------------------------------ #
    t_start = time.time()

    banner("1 / 4  Trajectory plot")
    traj_stats = plot_trajectory.plot_trajectory(args.csv, OUTPUT_DIR, args.show)

    banner("2 / 4  Error plots + metrics table")
    err_stats  = plot_errors.plot_errors(args.csv, OUTPUT_DIR, args.show)

    banner("3 / 4  Sliding surface + Lyapunov analysis")
    slide_stats = plot_sliding.plot_sliding(args.csv, args.lam, OUTPUT_DIR, args.show)

    banner("4 / 4  Control signals + chattering (FFT)")
    cmd_stats  = plot_cmdvel.plot_cmdvel(args.csv, OUTPUT_DIR, args.show)

    elapsed = time.time() - t_start

    # ------------------------------------------------------------------ #
    # Summary report
    # ------------------------------------------------------------------ #
    banner("SMC Performance Summary")

    print(f"\n{'─'*55}")
    print("  TRAJECTORY TRACKING")
    print(f"{'─'*55}")
    print(f"  Path RMSE        : {traj_stats['rmse']:.5f} m")
    print(f"  Path MAE         : {traj_stats['mae']:.5f} m")
    print(f"  Max path error   : {traj_stats['max_error']:.5f} m")

    print(f"\n{'─'*55}")
    print("  POSITION ERRORS")
    print(f"{'─'*55}")
    for col in ('ex', 'ey', 'e_total'):
        s = err_stats[col]
        print(f"  {col:<10}  RMSE={s['rmse']:.5f}  MAE={s['mae']:.5f}  "
              f"Max={s['max_error']:.5f}  (m)")

    print(f"\n{'─'*55}")
    print("  SLIDING SURFACE  (λ = {:.2f})".format(slide_stats['lambda_used']))
    print(f"{'─'*55}")
    print(f"  RMSE sx          : {slide_stats['s_rmse_x']:.5f}")
    print(f"  RMSE sy          : {slide_stats['s_rmse_y']:.5f}")
    print(f"  V_dot ≤ 0        : {slide_stats['pct_stable']:.1f}%  "
          "(reaching condition)")

    print(f"\n{'─'*55}")
    print("  CHATTERING  (high-freq energy ratio)")
    print(f"{'─'*55}")
    print(f"  linear.x         : {cmd_stats['chattering_ratio_linear']*100:.1f}%  "
          f"(peak {cmd_stats['dominant_freq_linear']:.2f} Hz)")
    print(f"  angular.z        : {cmd_stats['chattering_ratio_angular']*100:.1f}%  "
          f"(peak {cmd_stats['dominant_freq_angular']:.2f} Hz)")

    print(f"\n{'─'*55}")
    print(f"  Analysis completed in {elapsed:.1f} s")
    print(f"  Outputs → {OUTPUT_DIR}")
    print(f"{'─'*55}\n")


if __name__ == '__main__':
    main()
