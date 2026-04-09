"""
plot_cmdvel.py
==============
Control signal visualization:
  - linear.x and angular.z vs time
  - FFT of each signal to detect chattering (high-frequency components)

Chattering in SMC manifests as high-amplitude, high-frequency oscillations
in the control output. The FFT subplot makes this clearly quantifiable.
"""

import argparse
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from utils import load_csv, STYLE, apply_grid

OUTPUT_DIR = os.path.join(os.path.dirname(__file__), 'output')


def _compute_fft(signal: np.ndarray, dt_mean: float):
    """Return one-sided FFT frequencies and magnitudes."""
    n    = len(signal)
    freq = np.fft.rfftfreq(n, d=dt_mean)
    mag  = np.abs(np.fft.rfft(signal)) / n
    return freq, mag


def plot_cmdvel(csv_path: str, output_dir: str = OUTPUT_DIR, show: bool = False):
    df = load_csv(csv_path)
    os.makedirs(output_dir, exist_ok=True)

    t      = df['time'].values
    v_lin  = df['cmd_vel_linear_x'].values
    v_ang  = df['cmd_vel_angular_z'].values
    dt_mean = np.mean(np.diff(t))          # average sample interval

    freq_lin, mag_lin = _compute_fft(v_lin, dt_mean)
    freq_ang, mag_ang = _compute_fft(v_ang, dt_mean)

    # Dominant frequency detection (exclude DC)
    dom_idx_lin = np.argmax(mag_lin[1:]) + 1
    dom_idx_ang = np.argmax(mag_ang[1:]) + 1

    # ------------------------------------------------------------------ #
    # Figure: 2 × 2 grid
    #   Row 0: time-domain signals
    #   Row 1: FFT
    # ------------------------------------------------------------------ #
    fig, axes = plt.subplots(2, 2, figsize=STYLE['fig_size_wide'])
    fig.patch.set_facecolor('white')
    fig.suptitle('SMC Control Signals & Chattering Analysis (FFT)',
                 fontsize=STYLE['title_fs'], fontweight='bold')

    # ---- linear.x time domain ----
    ax = axes[0, 0]
    ax.plot(t, v_lin, color='steelblue', linewidth=STYLE['lw'])
    ax.axhline(0, color='black', linewidth=0.6)
    ax.set_ylabel('v_linear_x (m/s)', fontsize=STYLE['label_fs'])
    ax.set_title('Linear Velocity Command', fontsize=11)
    apply_grid(ax)

    # ---- angular.z time domain ----
    ax = axes[0, 1]
    ax.plot(t, v_ang, color='darkorange', linewidth=STYLE['lw'])
    ax.axhline(0, color='black', linewidth=0.6)
    ax.set_ylabel('v_angular_z (rad/s)', fontsize=STYLE['label_fs'])
    ax.set_title('Angular Velocity Command', fontsize=11)
    apply_grid(ax)

    # ---- FFT linear.x ----
    ax = axes[1, 0]
    ax.semilogy(freq_lin, mag_lin + 1e-10,
                color='steelblue', linewidth=STYLE['lw'])
    ax.axvline(freq_lin[dom_idx_lin], color='red', linestyle='--', linewidth=1.2,
               label=f'Peak: {freq_lin[dom_idx_lin]:.2f} Hz')
    ax.set_xlabel('Frequency (Hz)', fontsize=STYLE['label_fs'])
    ax.set_ylabel('Magnitude (log)', fontsize=STYLE['label_fs'])
    ax.set_title('FFT — Linear Velocity', fontsize=11)
    ax.legend(fontsize=10)
    apply_grid(ax)

    # ---- FFT angular.z ----
    ax = axes[1, 1]
    ax.semilogy(freq_ang, mag_ang + 1e-10,
                color='darkorange', linewidth=STYLE['lw'])
    ax.axvline(freq_ang[dom_idx_ang], color='red', linestyle='--', linewidth=1.2,
               label=f'Peak: {freq_ang[dom_idx_ang]:.2f} Hz')
    ax.set_xlabel('Frequency (Hz)', fontsize=STYLE['label_fs'])
    ax.set_ylabel('Magnitude (log)', fontsize=STYLE['label_fs'])
    ax.set_title('FFT — Angular Velocity', fontsize=11)
    ax.legend(fontsize=10)
    apply_grid(ax)

    # Shared x-axis label for time-domain panels
    for ax in axes[0]:
        ax.set_xlabel('Time (s)', fontsize=STYLE['label_fs'])

    fig.tight_layout()

    base = os.path.join(output_dir, 'cmdvel')
    fig.savefig(base + '.png', dpi=300, bbox_inches='tight', facecolor='white')
    fig.savefig(base + '.pdf', bbox_inches='tight', facecolor='white')
    print(f"[cmdvel] Saved → {base}.png / .pdf")

    # Chattering assessment
    fs = 1.0 / dt_mean
    high_freq_threshold = fs / 4          # anything above Nyquist/2
    high_idx = freq_lin >= high_freq_threshold
    chattering_ratio_lin = (np.sum(mag_lin[high_idx]) /
                            (np.sum(mag_lin) + 1e-12))
    chattering_ratio_ang = (np.sum(mag_ang[high_idx]) /
                            (np.sum(mag_ang) + 1e-12))
    print(f"[cmdvel] Sample rate: {fs:.1f} Hz  |  "
          f"High-freq energy ratio → "
          f"linear: {chattering_ratio_lin*100:.1f}%  "
          f"angular: {chattering_ratio_ang*100:.1f}%")

    if show:
        plt.show()
    plt.close(fig)

    return dict(
        dominant_freq_linear=float(freq_lin[dom_idx_lin]),
        dominant_freq_angular=float(freq_ang[dom_idx_ang]),
        chattering_ratio_linear=float(chattering_ratio_lin),
        chattering_ratio_angular=float(chattering_ratio_ang),
    )


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--csv',  default='/home/ducbao/go1_ws/smc_data.csv')
    parser.add_argument('--show', action='store_true')
    args = parser.parse_args()
    plot_cmdvel(args.csv, show=args.show)
