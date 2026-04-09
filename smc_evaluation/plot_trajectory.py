"""
plot_trajectory.py
==================
2D x-y trajectory: desired vs actual + Euclidean path error statistics.
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from utils import load_csv, compute_rmse, compute_mae, compute_max_error, STYLE, apply_grid

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), 'output')


def plot_trajectory(csv_path: str, output_dir: str = OUTPUT_DIR, show: bool = False):
    df = load_csv(csv_path)
    os.makedirs(output_dir, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=STYLE['fig_size_wide'])
    fig.patch.set_facecolor('white')

    # ------------------------------------------------------------------ #
    # Left panel: 2D trajectory
    # ------------------------------------------------------------------ #
    ax = axes[0]
    ax.plot(df['x_des'].values, df['y_des'].values,
            linestyle=STYLE['ls_desired'],
            color=STYLE['color_desired'],
            linewidth=STYLE['lw'],
            label='Desired trajectory')
    ax.plot(df['x_act'].values, df['y_act'].values,
            linestyle=STYLE['ls_actual'],
            color=STYLE['color_actual'],
            linewidth=STYLE['lw'],
            label='Actual trajectory')

    # Mark start / end
    ax.plot(df['x_act'].values[0],  df['y_act'].values[0],
            'go', markersize=8, label='Start')
    ax.plot(df['x_act'].values[-1], df['y_act'].values[-1],
            'rs', markersize=8, label='End')

    ax.set_xlabel('X (m)', fontsize=STYLE['label_fs'])
    ax.set_ylabel('Y (m)', fontsize=STYLE['label_fs'])
    ax.set_title('2D Trajectory: Desired vs Actual', fontsize=STYLE['title_fs'])
    ax.set_aspect('equal')
    ax.legend(fontsize=STYLE['label_fs'] - 1)
    apply_grid(ax)

    # ------------------------------------------------------------------ #
    # Right panel: Euclidean path error vs time
    # ------------------------------------------------------------------ #
    ax2 = axes[1]
    ax2.plot(df['time'].values, df['e_total'].values,
             color='crimson', linewidth=STYLE['lw'], label='Euclidean error ||e||')
    ax2.axhline(compute_rmse(df['e_total'].values), color='navy',
                linestyle='--', linewidth=1.2, label=f"RMSE = {compute_rmse(df['e_total'].values):.4f} m")
    ax2.fill_between(df['time'].values, 0, df['e_total'].values, alpha=0.15, color='crimson')

    ax2.set_xlabel('Time (s)', fontsize=STYLE['label_fs'])
    ax2.set_ylabel('Position Error (m)', fontsize=STYLE['label_fs'])
    ax2.set_title('Euclidean Tracking Error vs Time', fontsize=STYLE['title_fs'])
    ax2.legend(fontsize=STYLE['label_fs'] - 1)
    apply_grid(ax2)

    # Metrics annotation
    rmse = compute_rmse(df['e_total'].values)
    mae  = compute_mae(df['e_total'].values)
    maxe = compute_max_error(df['e_total'].values)
    textstr = f"RMSE = {rmse:.4f} m\nMAE  = {mae:.4f} m\nMax  = {maxe:.4f} m"
    ax2.text(0.97, 0.97, textstr,
             transform=ax2.transAxes, fontsize=10,
             verticalalignment='top', horizontalalignment='right',
             bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    fig.tight_layout()

    base = os.path.join(output_dir, 'trajectory')
    fig.savefig(base + '.png', dpi=300, bbox_inches='tight', facecolor='white')
    fig.savefig(base + '.pdf', bbox_inches='tight', facecolor='white')
    print(f"[trajectory] Saved → {base}.png / .pdf")

    if show:
        plt.show()
    plt.close(fig)

    return dict(rmse=rmse, mae=mae, max_error=maxe)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv',  default='/home/ducbao/go1_ws/smc_data.csv')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    plot_trajectory(args.csv, show=args.show)
