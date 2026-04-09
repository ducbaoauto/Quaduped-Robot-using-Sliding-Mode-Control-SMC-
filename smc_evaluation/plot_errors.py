"""
plot_errors.py
==============
3-subplot figure: ex(t), ey(t), ||e||(t) with RMSE/MAE table printed to console.
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from utils import (load_csv, compute_rmse, compute_mae, compute_max_error,
                   print_metrics_table, STYLE, apply_grid)

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), 'output')


def plot_errors(csv_path: str, output_dir: str = OUTPUT_DIR, show: bool = False):
    df = load_csv(csv_path)
    os.makedirs(output_dir, exist_ok=True)

    fig, axes = plt.subplots(3, 1, figsize=STYLE['fig_size_wide'], sharex=True)
    fig.patch.set_facecolor('white')
    fig.suptitle('SMC Position Tracking Errors', fontsize=STYLE['title_fs'], fontweight='bold')

    panels = [
        ('ex', 'X-axis Error  $e_x$ (m)',   'steelblue'),
        ('ey', 'Y-axis Error  $e_y$ (m)',   'darkorange'),
        ('e_total', 'Euclidean Error  $\|e\|$ (m)', 'crimson'),
    ]

    stats = {}
    for ax, (col, ylabel, color) in zip(axes, panels):
        arr = df[col].values

        ax.plot(df['time'].values, arr, color=color, linewidth=STYLE['lw'], label=ylabel)
        ax.axhline(0, color='black', linewidth=0.8, linestyle='-')

        rmse = compute_rmse(arr)
        mae  = compute_mae(arr)
        maxe = compute_max_error(arr)
        stats[col] = dict(rmse=rmse, mae=mae, max_error=maxe)

        # RMSE reference line
        if col != 'e_total':
            ax.axhline( rmse, color=color, linewidth=1.0, linestyle=':', alpha=0.7)
            ax.axhline(-rmse, color=color, linewidth=1.0, linestyle=':', alpha=0.7)

        ax.fill_between(df['time'].values, arr, 0, alpha=0.12, color=color)

        annotation = f"RMSE={rmse:.4f}  MAE={mae:.4f}  Max={maxe:.4f}"
        ax.set_title(annotation, fontsize=10, loc='right', pad=4)
        ax.set_ylabel(ylabel, fontsize=STYLE['label_fs'])
        apply_grid(ax)

    axes[-1].set_xlabel('Time (s)', fontsize=STYLE['label_fs'])
    fig.tight_layout()

    base = os.path.join(output_dir, 'errors')
    fig.savefig(base + '.png', dpi=300, bbox_inches='tight', facecolor='white')
    fig.savefig(base + '.pdf', bbox_inches='tight', facecolor='white')
    print(f"[errors] Saved → {base}.png / .pdf")

    if show:
        plt.show()
    plt.close(fig)

    # Console metrics table
    print("\n=== SMC Error Metrics ===")
    print_metrics_table(df)

    return stats


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv',  default='/home/ducbao/go1_ws/smc_data.csv')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    plot_errors(args.csv, show=args.show)
