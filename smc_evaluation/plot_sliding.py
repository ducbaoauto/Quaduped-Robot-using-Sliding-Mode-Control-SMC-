"""
plot_sliding.py
===============
Sliding surface s(t), Lyapunov function V(t) = 0.5*s^2, and V_dot(t).

The classical SMC sliding surface is:
    s(t) = e_dot(t) + lambda * e(t)

where e is the body-frame position error already logged as
sliding_surface_x / sliding_surface_y (see data_logger_node.py).

e_dot is computed numerically via central finite differences.

Stability is confirmed when V_dot = s * s_dot <= 0 (reaching condition).
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from utils import load_csv, STYLE, apply_grid

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), 'output')

DEFAULT_LAMBDA = 1.0   # sliding surface gain λ


def _numerical_derivative(values: np.ndarray, times: np.ndarray) -> np.ndarray:
    """Central finite differences with forward/backward at boundaries."""
    dt = np.diff(times)
    # central diff for interior
    deriv = np.empty_like(values)
    deriv[1:-1] = (values[2:] - values[:-2]) / (times[2:] - times[:-2])
    # forward / backward at edges
    deriv[0]  = (values[1]  - values[0])  / dt[0]
    deriv[-1] = (values[-1] - values[-2]) / dt[-1]
    return deriv


def plot_sliding(csv_path: str, lam: float = DEFAULT_LAMBDA,
                 output_dir: str = OUTPUT_DIR, show: bool = False):
    df = load_csv(csv_path)
    os.makedirs(output_dir, exist_ok=True)

    t  = df['time'].values
    ex = df['sliding_surface_x'].values   # body-frame ex (logged)
    ey = df['sliding_surface_y'].values   # body-frame ey (logged)

    # e_dot via numerical differentiation
    ex_dot = _numerical_derivative(ex, t)
    ey_dot = _numerical_derivative(ey, t)

    # Classical sliding surface
    sx = ex_dot + lam * ex
    sy = ey_dot + lam * ey

    # Scalar (combined) surface magnitude
    s_mag = np.sqrt(sx**2 + sy**2)

    # Lyapunov candidate V = 0.5 * (sx^2 + sy^2)
    V = 0.5 * (sx**2 + sy**2)

    # V_dot numerical
    V_dot = _numerical_derivative(V, t)

    # Percentage of time V_dot <= 0  (reaching condition satisfied)
    pct_stable = 100.0 * np.mean(V_dot <= 0)

    # ------------------------------------------------------------------ #
    # Plot
    # ------------------------------------------------------------------ #
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    fig.patch.set_facecolor('white')
    fig.suptitle(f'SMC Sliding Surface Analysis  (λ = {lam})',
                 fontsize=STYLE['title_fs'], fontweight='bold')

    # Panel 1: sx, sy
    ax = axes[0]
    ax.plot(t, sx, color='steelblue',   linewidth=STYLE['lw'], label=r'$s_x = \dot{e}_x + \lambda e_x$')
    ax.plot(t, sy, color='darkorange',  linewidth=STYLE['lw'], label=r'$s_y = \dot{e}_y + \lambda e_y$')
    ax.axhline(0, color='black', linewidth=0.8)
    ax.set_ylabel('s(t)', fontsize=STYLE['label_fs'])
    ax.set_title('Sliding Surface Components', fontsize=11)
    ax.legend(fontsize=10)
    apply_grid(ax)

    # Panel 2: |s|
    ax = axes[1]
    ax.plot(t, s_mag, color='purple', linewidth=STYLE['lw'], label=r'$\|s\| = \sqrt{s_x^2+s_y^2}$')
    ax.fill_between(t, 0, s_mag, alpha=0.15, color='purple')
    ax.set_ylabel(r'$\|s(t)\|$', fontsize=STYLE['label_fs'])
    ax.set_title('Sliding Surface Magnitude', fontsize=11)
    ax.legend(fontsize=10)
    apply_grid(ax)

    # Panel 3: V(t)
    ax = axes[2]
    ax.plot(t, V, color='darkgreen', linewidth=STYLE['lw'],
            label=r'$V(t) = \frac{1}{2}\|s\|^2$')
    ax.fill_between(t, 0, V, alpha=0.15, color='darkgreen')
    ax.set_ylabel('V(t)', fontsize=STYLE['label_fs'])
    ax.set_title('Lyapunov Function', fontsize=11)
    ax.legend(fontsize=10)
    apply_grid(ax)

    # Panel 4: V_dot(t)
    ax = axes[3]
    ax.plot(t, V_dot, color='crimson', linewidth=STYLE['lw'],
            label=r'$\dot{V}(t)$ — numerical')
    ax.axhline(0, color='black', linewidth=1.0, linestyle='-')
    ax.fill_between(t, V_dot, 0,
                    where=(V_dot > 0), alpha=0.25, color='red',
                    label='V_dot > 0 (unstable region)')
    ax.fill_between(t, V_dot, 0,
                    where=(V_dot <= 0), alpha=0.15, color='green',
                    label='V_dot ≤ 0 (stable)')
    ax.set_xlabel('Time (s)', fontsize=STYLE['label_fs'])
    ax.set_ylabel(r'$\dot{V}(t)$', fontsize=STYLE['label_fs'])
    ax.set_title(f'Lyapunov Derivative  '
                 f'(V_dot ≤ 0: {pct_stable:.1f}% of time)', fontsize=11)
    ax.legend(fontsize=9)
    apply_grid(ax)

    fig.tight_layout()

    base = os.path.join(output_dir, 'sliding')
    fig.savefig(base + '.png', dpi=300, bbox_inches='tight', facecolor='white')
    fig.savefig(base + '.pdf', bbox_inches='tight', facecolor='white')
    print(f"[sliding] Saved → {base}.png / .pdf")
    print(f"[sliding] Lyapunov stability: V_dot ≤ 0 in {pct_stable:.1f}% of samples")

    if show:
        plt.show()
    plt.close(fig)

    return dict(lambda_used=lam, pct_stable=pct_stable,
                s_rmse_x=float(np.sqrt(np.mean(sx**2))),
                s_rmse_y=float(np.sqrt(np.mean(sy**2))))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv',    default='/home/ducbao/go1_ws/smc_data.csv')
    parser.add_argument('--lambda', dest='lam', type=float, default=DEFAULT_LAMBDA)
    parser.add_argument('--show',   action='store_true')
    args = parser.parse_args()
    plot_sliding(args.csv, lam=args.lam, show=args.show)
