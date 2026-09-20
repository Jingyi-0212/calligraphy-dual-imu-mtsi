"""Complete standalone implementation of the two supplied MTSI screenshots.

Python 3.9+, standard library only. Run with --demo or --input paired.jsonl.
Input: synchronized pairs, seconds, m/s^2 including gravity, rad/s, wxyz
body-to-world quaternions in one common +Z-up world frame.
This is a preliminary signal smoothness measure, not handwriting quality.
"""

import argparse
import csv
import json
import math
from collections import deque
from pathlib import Path
from statistics import mean

GRAVITY = (0.0, 0.0, 9.80665)


def vector(values, length, name, allow_nan=False):
    result = tuple(float(v) for v in values)
    if len(result) != length or any(
        not math.isfinite(v) and not (allow_nan and math.isnan(v)) for v in result
    ):
        raise ValueError(f"{name}: expected {length} finite numbers")
    return result


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    length = norm(a)
    if not math.isfinite(length) or length < 1e-12:
        raise ValueError("Cannot normalize a zero or invalid vector/quaternion")
    return tuple(v / length for v in a)


def multiply(a, b):
    """Hamilton quaternion product; components are w, x, y, z."""
    w, x, y, z = a
    s, i, j, k = b
    return (w*s-x*i-y*j-z*k, w*i+x*s+y*k-z*j,
            w*j-x*k+y*s+z*i, w*k+x*j-y*i+z*s)


def rotate(q, v):
    """Rotate a body-frame vector into the world frame."""
    q = unit(vector(q, 4, "quaternion"))
    v = vector(v, 3, "vector")
    conjugate = (q[0], -q[1], -q[2], -q[3])
    return multiply(multiply(q, (0.0, *v)), conjugate)[1:]


def rms(values):
    values = list(values)
    if not values:
        raise ValueError("RMS needs at least one value")
    return math.sqrt(mean(v * v for v in values))


def paired_vectors(values, length, name):
    if len(values) != 2:
        raise ValueError(f"{name}: exactly two IMUs are required")
    return tuple(vector(v, length, name) for v in values)


class IMUPreprocessor:
    """Screenshot 2: gravity removal, projection, filtering, and jerk.

    Quaternions must already express both IMUs in the same world frame.
    direction_unit is a FIXED world-frame test direction, not a tracked stroke.
    gyro_biases are per-sensor body-frame biases from a stationary calibration.
    """

    def __init__(self, direction_unit=(1, 0, 0), gyro_biases=((0, 0, 0), (0, 0, 0)),
                 gravity=GRAVITY, cutoff_hz=8.0, rate_hz=100.0):
        self.direction_unit = unit(vector(direction_unit, 3, "direction"))
        self.gyro_biases = paired_vectors(gyro_biases, 3, "gyro_biases")
        self.gravity = vector(gravity, 3, "gravity")
        self.cutoff_hz = float(cutoff_hz)
        self.rate_hz = float(rate_hz)
        if not (math.isfinite(self.rate_hz) and math.isfinite(self.cutoff_hz)
                and 0 < self.cutoff_hz < self.rate_hz / 2):
            raise ValueError("Require 0 < cutoff_hz < rate_hz / 2")
        self.max_gap = 1.5 / self.rate_hz
        self.reset()

    def reset(self):
        self.previous_timestamp = None
        self.previous_a = [None, None]

    def update(self, timestamp, accelerations, gyros, quaternions):
        timestamp = float(timestamp)
        if not math.isfinite(timestamp):
            raise ValueError("Timestamp must be finite, in seconds")
        accelerations = paired_vectors(accelerations, 3, "accelerations")
        gyros = paired_vectors(gyros, 3, "gyros")
        quaternions = tuple(unit(q) for q in paired_vectors(quaternions, 4, "quaternions"))
        dt = (1 / self.rate_hz if self.previous_timestamp is None
              else timestamp - self.previous_timestamp)
        if dt <= 0:
            raise ValueError("Timestamps must strictly increase")
        # Never differentiate across a missing-data interval.
        old_values = ([None, None] if dt > self.max_gap else self.previous_a)
        alpha = 1 - math.exp(-2 * math.pi * self.cutoff_hz * dt)
        sample = {"timestamp": timestamp, "acceleration": [], "omega": [], "jerk": []}
        for i in range(2):
            a_world = rotate(quaternions[i], accelerations[i])
            a_linear = tuple(a_world[k] - self.gravity[k] for k in range(3))
            a_direction = dot(a_linear, self.direction_unit)
            omega = norm(tuple(g - b for g, b in zip(gyros[i], self.gyro_biases[i])))
            old = old_values[i]
            a_filtered = a_direction if old is None else old + alpha * (a_direction - old)
            jerk = math.nan if old is None else (a_filtered - old) / dt
            sample["acceleration"].append(a_filtered)
            # Match the screenshot: omega is NOT low-pass filtered here.
            sample["omega"].append(omega)
            sample["jerk"].append(jerk)
        self.previous_a = list(sample["acceleration"])
        self.previous_timestamp = timestamp
        return sample


class MTSIAnalyzer:
    """Screenshot 1: weighted scores from a sliding sample window.

    scales: rad/s^2 for d|omega|/dt, m/s^3 for directional jerk,
    and m/s^2 for detrended directional acceleration residual.
    Defaults reproduce the screenshot's 100 samples and 0.015 s gap rule.
    """

    def __init__(self, weights=(0.35, 0.40, 0.25), scales=(3.0, 12.0, 0.6),
                 rate_hz=100.0, window_samples=100):
        self.weights = vector(weights, 3, "weights")
        self.scales = vector(scales, 3, "scales")
        if any(w < 0 for w in self.weights) or not math.isclose(sum(self.weights), 1):
            raise ValueError("Weights must be nonnegative and sum to one")
        if any(s <= 0 for s in self.scales):
            raise ValueError("Scales must be positive")
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ValueError("rate_hz must be positive and finite")
        if type(window_samples) is not int or window_samples < 3:
            raise ValueError("window_samples must be an integer >= 3")
        self.max_gap = 1.5 / rate_hz
        self.window = deque(maxlen=window_samples)

    def reset(self):
        self.window.clear()

    def update(self, sample):
        t = float(sample["timestamp"])
        if not math.isfinite(t):
            raise ValueError("Timestamp must be finite")
        # Copy the sample so a caller cannot mutate the analysis history.
        row = {"timestamp": t}
        for key in ("acceleration", "omega", "jerk"):
            row[key] = vector(sample[key], 2, key, allow_nan=(key == "jerk"))
        if self.window:
            dt = t - self.window[-1]["timestamp"]
            if dt <= 0:
                raise ValueError("Timestamps must strictly increase")
            if dt > self.max_gap:
                self.window.clear()
        self.window.append(row)
        if len(self.window) < self.window.maxlen:
            return None
        rows = list(self.window)
        jerks = [v for row in rows for v in row["jerk"]]
        if not all(math.isfinite(v) for v in jerks):
            return None
        angular_change = rms([
            (b["omega"][i] - a["omega"][i]) / (b["timestamp"] - a["timestamp"])
            for a, b in zip(rows, rows[1:]) for i in range(2)
        ])
        times = [row["timestamp"] - rows[0]["timestamp"] for row in rows]
        average_time = mean(times)
        centered = [t - average_time for t in times]
        denominator = sum(x*x for x in centered)
        residuals = []
        for i in range(2):
            values = [row["acceleration"][i] for row in rows]
            average = mean(values)
            slope = sum(x*(y-average) for x, y in zip(centered, values)) / denominator
            residuals.extend(y-average-slope*x for x, y in zip(centered, values))
        measures = (angular_change, rms(jerks), rms(residuals))
        scores = [100 / (1 + (value / scale)**2)
                  for value, scale in zip(measures, self.scales)]
        return dict(timestamp=t, S_omega=scores[0], S_jerk=scores[1], S_a=scores[2],
                    MTSI=sum(w*s for w, s in zip(self.weights, scores)))


class DualIMUPipeline:
    """Feed one synchronized pair per update; returns (features, score_or_None)."""

    def __init__(self, direction_unit=(1, 0, 0), gyro_biases=((0, 0, 0), (0, 0, 0)),
                 rate_hz=100.0, cutoff_hz=8.0, window_samples=100,
                 weights=(0.35, 0.40, 0.25), scales=(3.0, 12.0, 0.6)):
        self.preprocessor = IMUPreprocessor(direction_unit, gyro_biases,
                                            cutoff_hz=cutoff_hz, rate_hz=rate_hz)
        self.analyzer = MTSIAnalyzer(weights, scales, rate_hz, window_samples)

    def reset(self):
        self.preprocessor.reset()
        self.analyzer.reset()

    def update(self, timestamp, accelerations, gyros, quaternions):
        sample = self.preprocessor.update(timestamp, accelerations, gyros, quaternions)
        return sample, self.analyzer.update(sample)


def demo_records():
    """Deterministic synthetic examples, not measurements of a real brush.

    Identity orientation and gyro=0 describe pure translation consistently;
    angular scoring is tested separately in test_dual_imu_mtsi.py.
    """
    for k in range(1200):
        t = k / 100
        if t < 4:
            label, ax = "stationary", 0.0
        elif t < 8:
            label, ax = "smooth_translation", 0.15 * math.sin(2*math.pi*0.5*(t-4))
        else:
            label, ax = "abrupt_translation", 3.0 * math.sin(2*math.pi*5*(t-8))
        yield dict(timestamp=t, accelerations=[[ax, 0, GRAVITY[2]]] * 2,
                   gyros=[[0, 0, 0]] * 2, quaternions=[[1, 0, 0, 0]] * 2, label=label)


def read_jsonl(path):
    with Path(path).open(encoding="utf-8-sig") as source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("Expected a paired JSON object")
                yield row
            except (ValueError, TypeError) as exc:
                raise ValueError(f"Input line {line_number}: {exc}") from exc


def write_results(records, pipeline, output):
    fields = ["timestamp", "label", "a_dir_1", "a_dir_2", "omega_1", "omega_2",
              "jerk_1", "jerk_2", "features_valid", "S_omega", "S_jerk", "S_a", "MTSI"]
    count = valid = 0
    with Path(output).open("w", newline="", encoding="utf-8-sig") as target:
        writer = csv.DictWriter(target, fieldnames=fields)
        writer.writeheader()
        for record in records:
            sample, scores = pipeline.update(*(record[key] for key in
                ("timestamp", "accelerations", "gyros", "quaternions")))
            row = dict(timestamp=sample["timestamp"], label=record.get("label", ""),
                       features_valid=int(scores is not None))
            for name, key in (("a_dir", "acceleration"), ("omega", "omega"), ("jerk", "jerk")):
                for i, value in enumerate(sample[key], 1):
                    row[f"{name}_{i}"] = value if math.isfinite(value) else ""
            if scores is not None:
                row.update(scores)
                valid += 1
            writer.writerow(row)
            count += 1
    return count, valid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--demo", action="store_true", help="Run synthetic 100 Hz samples")
    mode.add_argument("--input", type=Path, help="Synchronized paired samples in JSONL")
    parser.add_argument("--output", type=Path, default=Path("mtsi_results.csv"))
    parser.add_argument("--direction", type=float, nargs=3, default=(1, 0, 0), metavar=("X", "Y", "Z"))
    parser.add_argument("--bias-1", type=float, nargs=3, default=(0, 0, 0))
    parser.add_argument("--bias-2", type=float, nargs=3, default=(0, 0, 0))
    args = parser.parse_args()
    pipeline = DualIMUPipeline(direction_unit=args.direction,
                              gyro_biases=(args.bias_1, args.bias_2))
    records = demo_records() if args.demo else read_jsonl(args.input)
    count, valid = write_results(records, pipeline, args.output)
    print(f"{'SIMULATED' if args.demo else 'INPUT'}: {count} samples; {valid} valid scores")
    print(f"Saved: {args.output.resolve()}")
    print("Preliminary smoothness only; stillness may score 100.")


if __name__ == "__main__":
    main()
