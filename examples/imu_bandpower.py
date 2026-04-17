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
    analyze_signal,
    attenuation_db,
    build_default_bands,
    channel_label,
    compute_band_powers,
    load_sensor_csv,
    normalize_channels,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Band-power-over-time analysis for IMU sensor CSV logs")
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


def print_bandpower_summary(axis_name: str, raw_band_powers: dict[str, np.ndarray], filtered_band_powers: dict[str, np.ndarray]) -> None:
    print(axis_name)
    print(f"{'Band':<14} {'Raw Avg':>12} {'Filtered Avg':>14} {'Atten (dB)':>12}")
    for band_label, raw_values in raw_band_powers.items():
        filtered_values = filtered_band_powers.get(band_label)
        if filtered_values is None:
            continue
        raw_avg = float(np.mean(raw_values))
        filtered_avg = float(np.mean(filtered_values))
        print(
            f"{band_label:<14} {raw_avg:>12.3e} {filtered_avg:>14.3e} "
            f"{attenuation_db(raw_avg, filtered_avg):>12.2f}"
        )
    print()


def main() -> int:
    args = parse_args()
    channels = normalize_channels(args.channels)

    try:
        data = load_sensor_csv(args.sensor_csv, channels)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Loaded {data.path.name} using {data.time_column}; resampled to {data.sample_rate_hz:.2f} Hz")
    print(f"Nyquist frequency: {data.nyquist_hz:.2f} Hz")
    print()

    bands_hz = build_default_bands(data.nyquist_hz)
    if not bands_hz:
        print("Error: No valid band-power ranges are available for this file's sample rate", file=sys.stderr)
        return 1

    figure, axes = plt.subplots(
        nrows=len(channels),
        ncols=1,
        figsize=(12, max(4, 3.5 * len(channels))),
        constrained_layout=True,
        sharex=True,
    )
    axes = np.atleast_1d(axes).reshape(-1)

    for axis_index, channel in enumerate(channels):
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

        raw_band_powers = compute_band_powers(raw_result, bands_hz)
        filtered_band_powers = compute_band_powers(filtered_result, bands_hz)
        print_bandpower_summary(axis_name, raw_band_powers, filtered_band_powers)

        axis = axes[axis_index]
        for band_label, band_values in raw_band_powers.items():
            axis.plot(raw_result.centers_s, band_values, linewidth=1.4, label=band_label)
        axis.set_title(f"{axis_name}: Raw Band Power Over Time")
        axis.set_ylabel("Band Power")
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")

    axes[-1].set_xlabel("Time (s)")
    figure.suptitle(f"Offline IMU Band Power: {data.path.name}", fontsize=14)
    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
