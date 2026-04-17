from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter


DEFAULT_CHANNELS = ("acc_lin_x", "acc_lin_y", "acc_lin_z")
DEFAULT_WINDOW_SEC = 8.0
DEFAULT_OVERLAP_SEC = 4.0
DEFAULT_BANDS_HZ = (
    (0.0, 0.5),
    (0.5, 1.5),
    (1.5, 3.0),
    (3.0, 4.0),
)
SG_WINDOW_LENGTH = 5
SG_POLYORDER = 3
EPSILON = 1e-12

CHANNEL_LABELS = {
    "acc_lin_x": "Acc X",
    "acc_lin_y": "Acc Y",
    "acc_lin_z": "Acc Z",
}


@dataclass
class ResampledImuData:
    path: Path
    time_s: np.ndarray
    sample_rate_hz: float
    time_column: str
    channels: dict[str, np.ndarray]
    filtered_channels: dict[str, np.ndarray]

    @property
    def nyquist_hz(self) -> float:
        return 0.5 * self.sample_rate_hz


@dataclass
class SpectralMetrics:
    fft_mean: np.ndarray
    fft_max: np.ndarray
    fft_freq_max: np.ndarray
    fft_power: np.ndarray
    fft_bandwidth: np.ndarray


@dataclass
class SpectralResult:
    centers_s: np.ndarray
    freqs_hz: np.ndarray
    magnitude: np.ndarray
    power: np.ndarray
    psd: np.ndarray
    metrics: SpectralMetrics


def channel_label(channel: str) -> str:
    return CHANNEL_LABELS.get(channel, channel)


def normalize_channels(channels: list[str] | tuple[str, ...] | None) -> list[str]:
    if not channels:
        return list(DEFAULT_CHANNELS)
    return list(dict.fromkeys(channels))


def load_sensor_csv(path: Path | str, channels: list[str] | tuple[str, ...] | None = None) -> ResampledImuData:
    csv_path = Path(path)
    if not csv_path.is_file():
        raise ValueError(f"Sensor CSV not found: {csv_path}")

    selected_channels = normalize_channels(channels)
    with csv_path.open("r", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        fieldnames = reader.fieldnames or []
        time_column = _pick_time_column(fieldnames)
        missing_channels = [channel for channel in selected_channels if channel not in fieldnames]
        if missing_channels:
            missing = ", ".join(missing_channels)
            raise ValueError(f"Missing required IMU columns: {missing}")

        time_values: list[int | float] = []
        channel_values: dict[str, list[float]] = {channel: [] for channel in selected_channels}
        for row in reader:
            parsed_time = _parse_time_value(row.get(time_column, ""), time_column)
            if parsed_time is None:
                continue

            time_values.append(parsed_time)
            for channel in selected_channels:
                channel_values[channel].append(_parse_float(row.get(channel, "")))

    if len(time_values) < 2:
        raise ValueError(f"{csv_path.name} does not contain enough valid timestamp samples")

    if time_column == "timestamp_ns":
        raw_time = np.asarray(time_values, dtype=np.int64)
    else:
        raw_time = np.asarray(time_values, dtype=float)

    order = np.argsort(raw_time, kind="stable")
    raw_time = raw_time[order]
    unique_time, inverse = np.unique(raw_time, return_inverse=True)

    grouped_signals: dict[str, np.ndarray] = {}
    for channel in selected_channels:
        values = np.asarray(channel_values[channel], dtype=float)[order]
        sums = np.zeros(unique_time.shape[0], dtype=float)
        counts = np.zeros(unique_time.shape[0], dtype=float)
        finite_mask = np.isfinite(values)
        np.add.at(sums, inverse[finite_mask], values[finite_mask])
        np.add.at(counts, inverse[finite_mask], 1.0)

        averaged = np.full(unique_time.shape[0], np.nan, dtype=float)
        valid_counts = counts > 0
        averaged[valid_counts] = sums[valid_counts] / counts[valid_counts]
        grouped_signals[channel] = averaged

    time_s = _relative_time_seconds(unique_time, time_column)
    sample_rate_hz = estimate_sample_rate_hz(time_s)
    dt = 1.0 / sample_rate_hz

    starts: list[float] = []
    ends: list[float] = []
    for channel in selected_channels:
        signal = grouped_signals[channel]
        valid_signal = np.isfinite(signal)
        if np.count_nonzero(valid_signal) < 2:
            raise ValueError(f"{csv_path.name} does not contain enough valid samples for {channel}")
        starts.append(time_s[valid_signal][0])
        ends.append(time_s[valid_signal][-1])

    start_time = max(starts)
    end_time = min(ends)
    if end_time - start_time < dt:
        raise ValueError(f"{csv_path.name} does not span enough time after alignment to resample")

    resampled_time = np.arange(start_time, end_time + (0.5 * dt), dt)
    if resampled_time.size < 2:
        raise ValueError(f"{csv_path.name} does not contain enough samples after resampling")

    channels_out: dict[str, np.ndarray] = {}
    filtered_out: dict[str, np.ndarray] = {}
    for channel in selected_channels:
        signal = grouped_signals[channel]
        valid_signal = np.isfinite(signal)
        channels_out[channel] = np.interp(
            resampled_time,
            time_s[valid_signal],
            signal[valid_signal],
        )
        filtered_out[channel] = apply_savgol_filter(channels_out[channel])

    resampled_time = resampled_time - resampled_time[0]
    return ResampledImuData(
        path=csv_path,
        time_s=resampled_time,
        sample_rate_hz=sample_rate_hz,
        time_column=time_column,
        channels=channels_out,
        filtered_channels=filtered_out,
    )


def estimate_sample_rate_hz(time_s: np.ndarray) -> float:
    diffs = np.diff(time_s)
    positive_diffs = diffs[diffs > 0]
    if positive_diffs.size == 0:
        raise ValueError("Could not estimate sample rate from non-increasing timestamps")
    return 1.0 / float(np.median(positive_diffs))


def apply_savgol_filter(signal: np.ndarray) -> np.ndarray:
    if signal.size < 3:
        return signal.copy()

    if signal.size <= SG_POLYORDER + 1:
        return signal.copy()

    window_length = min(SG_WINDOW_LENGTH, signal.size)
    if window_length % 2 == 0:
        window_length -= 1
    if window_length <= SG_POLYORDER:
        return signal.copy()

    return savgol_filter(signal, window_length=window_length, polyorder=SG_POLYORDER)


def validate_window_settings(sample_rate_hz: float, window_sec: float, overlap_sec: float) -> tuple[int, int]:
    if window_sec <= 0:
        raise ValueError("window_sec must be positive")
    if overlap_sec < 0:
        raise ValueError("overlap_sec cannot be negative")
    if overlap_sec >= window_sec:
        raise ValueError("overlap_sec must be smaller than window_sec")

    window_size = int(round(window_sec * sample_rate_hz))
    overlap_size = int(round(overlap_sec * sample_rate_hz))
    step_size = window_size - overlap_size

    if window_size < 2:
        raise ValueError("window_sec is too small for the estimated sample rate")
    if step_size < 1:
        raise ValueError("window_sec and overlap_sec result in an invalid step size")

    return window_size, step_size


def analyze_signal(
    time_s: np.ndarray,
    signal: np.ndarray,
    sample_rate_hz: float,
    window_sec: float = DEFAULT_WINDOW_SEC,
    overlap_sec: float = DEFAULT_OVERLAP_SEC,
) -> SpectralResult:
    window_size, step_size = validate_window_settings(sample_rate_hz, window_sec, overlap_sec)
    if signal.size < window_size:
        raise ValueError(
            f"Signal has {signal.size} samples but needs at least {window_size} "
            f"for one {window_sec:.1f}s window"
        )

    starts = np.arange(0, signal.size - window_size + 1, step_size, dtype=int)
    if starts.size == 0:
        raise ValueError("No full analysis windows could be created")

    windows = np.stack([signal[start : start + window_size] for start in starts], axis=0)
    centers_s = np.array([time_s[start : start + window_size].mean() for start in starts], dtype=float)

    detrended = windows - windows.mean(axis=1, keepdims=True)
    hann = np.hanning(window_size)
    tapered = detrended * hann

    fft_values = np.fft.rfft(tapered, axis=1)
    freqs_hz = np.fft.rfftfreq(window_size, d=(1.0 / sample_rate_hz))
    magnitude = np.abs(fft_values)
    power = (magnitude ** 2) / window_size

    if freqs_hz.size > 1:
        bin_width_hz = freqs_hz[1] - freqs_hz[0]
        psd = power / bin_width_hz
    else:
        psd = power.copy()

    dominant_indices = np.argmax(magnitude, axis=1)
    dominant_freqs = freqs_hz[dominant_indices]
    mag_sum = magnitude.sum(axis=1)
    bandwidth = np.zeros(magnitude.shape[0], dtype=float)
    nonzero_mask = mag_sum > 0
    if np.any(nonzero_mask):
        centered = freqs_hz[np.newaxis, :] - dominant_freqs[:, np.newaxis]
        numerator = np.sum((centered ** 2) * magnitude, axis=1)
        bandwidth[nonzero_mask] = np.sqrt(numerator[nonzero_mask] / mag_sum[nonzero_mask])

    metrics = SpectralMetrics(
        fft_mean=magnitude.mean(axis=1),
        fft_max=magnitude.max(axis=1),
        fft_freq_max=dominant_freqs,
        fft_power=np.sum(magnitude ** 2, axis=1) / window_size,
        fft_bandwidth=bandwidth,
    )

    return SpectralResult(
        centers_s=centers_s,
        freqs_hz=freqs_hz,
        magnitude=magnitude,
        power=power,
        psd=psd,
        metrics=metrics,
    )


def build_default_bands(nyquist_hz: float) -> list[tuple[float, float]]:
    max_high = 0.95 * nyquist_hz
    bands: list[tuple[float, float]] = []
    for low_hz, high_hz in DEFAULT_BANDS_HZ:
        clipped_high_hz = min(high_hz, max_high)
        if clipped_high_hz <= low_hz:
            continue
        bands.append((low_hz, clipped_high_hz))
    return bands


def compute_band_powers(
    spectral_result: SpectralResult,
    bands_hz: list[tuple[float, float]],
) -> dict[str, np.ndarray]:
    freqs_hz = spectral_result.freqs_hz
    if freqs_hz.size == 0:
        return {}

    band_powers: dict[str, np.ndarray] = {}
    bin_width_hz = freqs_hz[1] - freqs_hz[0] if freqs_hz.size > 1 else 0.0

    for band_index, (low_hz, high_hz) in enumerate(bands_hz):
        is_last_band = band_index == len(bands_hz) - 1
        if is_last_band:
            mask = (freqs_hz >= low_hz) & (freqs_hz <= high_hz)
        else:
            mask = (freqs_hz >= low_hz) & (freqs_hz < high_hz)
        if not np.any(mask):
            continue

        psd_slice = spectral_result.psd[:, mask]
        freq_slice = freqs_hz[mask]
        if freq_slice.size == 1:
            power = psd_slice[:, 0] * (bin_width_hz if bin_width_hz > 0 else 1.0)
        else:
            power = np.trapezoid(psd_slice, freq_slice, axis=1)
        band_powers[format_band_label(low_hz, high_hz)] = power

    return band_powers


def format_band_label(low_hz: float, high_hz: float) -> str:
    return f"{low_hz:.1f}-{high_hz:.1f} Hz"


def average_psd(spectral_result: SpectralResult) -> np.ndarray:
    return spectral_result.psd.mean(axis=0)


def dominant_average_frequency(spectral_result: SpectralResult) -> tuple[float, float]:
    avg_psd = average_psd(spectral_result)
    dominant_index = int(np.argmax(avg_psd))
    return spectral_result.freqs_hz[dominant_index], avg_psd[dominant_index]


def average_metrics(spectral_result: SpectralResult) -> dict[str, float]:
    metrics = spectral_result.metrics
    return {
        "FFTMean": float(np.mean(metrics.fft_mean)),
        "FFTMax": float(np.mean(metrics.fft_max)),
        "FFT_FreqMax": float(np.mean(metrics.fft_freq_max)),
        "FFT_Power": float(np.mean(metrics.fft_power)),
        "FFT_BW": float(np.mean(metrics.fft_bandwidth)),
    }


def attenuation_db(raw_value: float, filtered_value: float) -> float:
    return 10.0 * np.log10((raw_value + EPSILON) / (filtered_value + EPSILON))


def heatmap_extent(centers_s: np.ndarray, freqs_hz: np.ndarray, window_sec: float) -> tuple[float, float, float, float]:
    half_window = 0.5 * window_sec
    if centers_s.size == 1:
        start_time = centers_s[0] - half_window
        end_time = centers_s[0] + half_window
    else:
        start_time = centers_s[0] - half_window
        end_time = centers_s[-1] + half_window
    return (freqs_hz[0], freqs_hz[-1], start_time, end_time)


def _pick_time_column(columns: list[str] | tuple[str, ...]) -> str:
    if "timestamp_ns" in columns:
        return "timestamp_ns"
    if "time_ms" in columns:
        return "time_ms"
    raise ValueError("Sensor CSV is missing both 'timestamp_ns' and 'time_ms'")


def _relative_time_seconds(time_values: np.ndarray, time_column: str) -> np.ndarray:
    if time_column == "timestamp_ns":
        time_values = time_values.astype(np.int64)
        return (time_values - time_values[0]).astype(np.float64) / 1_000_000_000.0

    time_values = time_values.astype(np.float64)
    return (time_values - time_values[0]) / 1_000.0


def _parse_time_value(raw_value: str | None, time_column: str) -> int | float | None:
    if raw_value is None or raw_value == "":
        return None
    try:
        if time_column == "timestamp_ns":
            return int(float(raw_value))
        return float(raw_value)
    except ValueError:
        return None


def _parse_float(raw_value: str | None) -> float:
    if raw_value is None or raw_value == "":
        return np.nan
    try:
        return float(raw_value)
    except ValueError:
        return np.nan
