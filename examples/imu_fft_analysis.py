# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "matplotlib",
#     "numpy",
#     "scipy",
# ]
# ///
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from _imu_spectral import (
    DEFAULT_CHANNELS,
    DEFAULT_OVERLAP_SEC,
    DEFAULT_WINDOW_SEC,
    EPSILON,
    analyze_signal,
    attenuation_db,
    average_metrics,
    average_psd,
    channel_label,
    dominant_average_frequency,
    heatmap_extent,
    load_sensor_csv,
    normalize_channels,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline FFT and PSD analysis for IMU sensor CSV logs")
    parser.add_argument("sensor_csv", type=Path, help="Path to a *_sensor.csv field test log")
    parser.add_argument(
        "--channels",
        nargs="+",
        default=list(DEFAULT_CHANNELS),
        help="Sensor columns to analyze (default: acc_lin_x acc_lin_y acc_lin_z)",
    )
    parser.add_argument(
        "--window-sec",
        type=float,
        default=DEFAULT_WINDOW_SEC,
        help=f"Window length in seconds (default: {DEFAULT_WINDOW_SEC})",
    )
    parser.add_argument(
        "--overlap-sec",
        type=float,
        default=DEFAULT_OVERLAP_SEC,
        help=f"Window overlap in seconds (default: {DEFAULT_OVERLAP_SEC})",
    )
    return parser.parse_args()


def print_axis_summary(
    axis_name: str,
    sample_rate_hz: float,
    nyquist_hz: float,
    raw_result,
    filtered_result,
) -> None:
    raw_freq_hz, raw_power = dominant_average_frequency(raw_result)
    filtered_freq_hz, filtered_power = dominant_average_frequency(filtered_result)
    raw_metrics = average_metrics(raw_result)
    filtered_metrics = average_metrics(filtered_result)

    print(f"{axis_name}: fs={sample_rate_hz:.2f} Hz | Nyquist={nyquist_hz:.2f} Hz | windows={len(raw_result.centers_s)}")
    print(
        "  dominant PSD raw={:.2f} Hz ({:.3e}) | filtered={:.2f} Hz ({:.3e}) | attenuation={:.2f} dB".format(
            raw_freq_hz,
            raw_power,
            filtered_freq_hz,
            filtered_power,
            attenuation_db(raw_power, filtered_power),
        )
    )
    print(
        "  raw avg metrics: FFTMean={FFTMean:.3e}, FFTMax={FFTMax:.3e}, FFT_FreqMax={FFT_FreqMax:.2f}, "
        "FFT_Power={FFT_Power:.3e}, FFT_BW={FFT_BW:.2f}".format(**raw_metrics)
    )
    print(
        "  filtered avg metrics: FFTMean={FFTMean:.3e}, FFTMax={FFTMax:.3e}, FFT_FreqMax={FFT_FreqMax:.2f}, "
        "FFT_Power={FFT_Power:.3e}, FFT_BW={FFT_BW:.2f}".format(**filtered_metrics)
    )


def plot_axis_rows(
    figure: plt.Figure,
    axis_rows: list[plt.Axes],
    time_s: np.ndarray,
    raw_signal: np.ndarray,
    filtered_signal: np.ndarray,
    raw_result,
    filtered_result,
    axis_name: str,
    window_sec: float,
) -> None:
    time_ax, spectrum_ax, heatmap_ax = axis_rows

    time_ax.plot(time_s, raw_signal, color="0.65", linewidth=1.0, label="Raw")
    time_ax.plot(time_s, filtered_signal, color="tab:red", linewidth=1.2, label="SG filtered")
    time_ax.set_title(f"{axis_name}: Time Domain")
    time_ax.set_ylabel("m/s^2")
    time_ax.grid(True, alpha=0.3)
    time_ax.legend(loc="upper right")

    raw_avg_psd = average_psd(raw_result)
    filtered_avg_psd = average_psd(filtered_result)
    spectrum_ax.plot(raw_result.freqs_hz, raw_avg_psd, color="tab:blue", linewidth=1.3, label="Raw avg PSD")
    spectrum_ax.plot(
        filtered_result.freqs_hz,
        filtered_avg_psd,
        color="tab:orange",
        linewidth=1.3,
        label="Filtered avg PSD",
    )
    spectrum_ax.set_title(f"{axis_name}: Averaged One-Sided Spectrum / PSD")
    spectrum_ax.set_ylabel("Power / Hz")
    spectrum_ax.set_xlim(0.0, raw_result.freqs_hz[-1])
    spectrum_ax.grid(True, alpha=0.3)
    spectrum_ax.legend(loc="upper right")

    heatmap = 10.0 * np.log10(raw_result.psd + EPSILON)
    extent = heatmap_extent(raw_result.centers_s, raw_result.freqs_hz, window_sec)
    image = heatmap_ax.imshow(
        heatmap,
        aspect="auto",
        origin="lower",
        extent=extent,
        cmap="viridis",
    )
    figure.colorbar(image, ax=heatmap_ax, pad=0.01, label="PSD (dB)")
    heatmap_ax.set_title(f"{axis_name}: Windowed Spectral Heatmap (Raw)")
    heatmap_ax.set_xlabel("Frequency (Hz)")
    heatmap_ax.set_ylabel("Time (s)")


def main() -> int:
    args = parse_args()
    channels = normalize_channels(args.channels)

    try:
        data = load_sensor_csv(args.sensor_csv, channels)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Loaded {data.path.name} using {data.time_column}; resampled to {data.sample_rate_hz:.2f} Hz")
    print()

    figure, axes = plt.subplots(
        nrows=3 * len(channels),
        ncols=1,
        figsize=(12, max(8, 4 * len(channels))),
        constrained_layout=True,
    )
    axes = np.atleast_1d(axes).reshape(-1)

    for channel_index, channel in enumerate(channels):
        axis_name = channel_label(channel)
        raw_signal = data.channels[channel]
        filtered_signal = data.filtered_channels[channel]

        try:
            raw_result = analyze_signal(
                data.time_s,
                raw_signal,
                data.sample_rate_hz,
                window_sec=args.window_sec,
                overlap_sec=args.overlap_sec,
            )
            filtered_result = analyze_signal(
                data.time_s,
                filtered_signal,
                data.sample_rate_hz,
                window_sec=args.window_sec,
                overlap_sec=args.overlap_sec,
            )
        except ValueError as exc:
            print(f"Error: {axis_name}: {exc}", file=sys.stderr)
            return 1

        print_axis_summary(
            axis_name=axis_name,
            sample_rate_hz=data.sample_rate_hz,
            nyquist_hz=data.nyquist_hz,
            raw_result=raw_result,
            filtered_result=filtered_result,
        )
        print()

        row_start = channel_index * 3
        plot_axis_rows(
            figure=figure,
            axis_rows=list(axes[row_start : row_start + 3]),
            time_s=data.time_s,
            raw_signal=raw_signal,
            filtered_signal=filtered_signal,
            raw_result=raw_result,
            filtered_result=filtered_result,
            axis_name=axis_name,
            window_sec=args.window_sec,
        )

    figure.suptitle(f"Offline IMU FFT Analysis: {data.path.name}", fontsize=14)
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
