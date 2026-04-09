"""
Shared utilities for SMC performance analysis.
"""

import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------

REQUIRED_COLUMNS = {
    'timestamp',
    'x_des', 'y_des',
    'x_act', 'y_act', 'yaw_act',
    'sliding_surface_x', 'sliding_surface_y',
    'cmd_vel_linear_x', 'cmd_vel_angular_z',
}


def load_csv(path: str) -> pd.DataFrame:
    """Load SMC data CSV, validate columns, and add derived columns."""
    df = pd.read_csv(path)

    missing = REQUIRED_COLUMNS - set(df.columns)
    if missing:
        raise ValueError(f"CSV is missing columns: {missing}")

    # Normalise time to start at 0
    df['time'] = df['timestamp'] - df['timestamp'].iloc[0]

    # Global-frame position errors
    df['ex'] = df['x_des'] - df['x_act']
    df['ey'] = df['y_des'] - df['y_act']
    df['e_total'] = np.sqrt(df['ex'] ** 2 + df['ey'] ** 2)

    return df


# ---------------------------------------------------------------------------
# Metrics
# ---------------------------------------------------------------------------

def compute_rmse(series: np.ndarray) -> float:
    return float(np.sqrt(np.mean(series ** 2)))


def compute_mae(series: np.ndarray) -> float:
    return float(np.mean(np.abs(series)))


def compute_max_error(series: np.ndarray) -> float:
    return float(np.max(np.abs(series)))


def print_metrics_table(df: pd.DataFrame) -> None:
    """Print a formatted RMSE / MAE / Max summary table."""
    metrics = {
        'ex  (m)':      df['ex'].values,
        'ey  (m)':      df['ey'].values,
        'e_total (m)':  df['e_total'].values,
        'sx (body)':    df['sliding_surface_x'].values,
        'sy (body)':    df['sliding_surface_y'].values,
    }

    header = f"{'Signal':<18} {'RMSE':>10} {'MAE':>10} {'Max|e|':>10}"
    sep    = '-' * len(header)
    print(sep)
    print(header)
    print(sep)
    for name, arr in metrics.items():
        print(f"{name:<18} {compute_rmse(arr):>10.5f} {compute_mae(arr):>10.5f} "
              f"{compute_max_error(arr):>10.5f}")
    print(sep)


# ---------------------------------------------------------------------------
# Plot style helpers
# ---------------------------------------------------------------------------

STYLE = dict(
    fig_size_wide=(12, 8),
    fig_size_std=(10, 6),
    grid_alpha=0.3,
    grid_ls='--',
    label_fs=12,
    title_fs=14,
    lw=1.5,
    color_desired='k',
    ls_desired='--',
    color_actual='b',
    ls_actual='-',
)


def apply_grid(ax):
    ax.grid(True, alpha=STYLE['grid_alpha'], linestyle=STYLE['grid_ls'])
    ax.set_facecolor('white')
