"""
HC-SR04 Calibration Analyzer

Reads the serial output from the Arduino calibration sketch,
computes noise profiles, and generates a sensor calibration
config for the simulation.

Usage:
    # Capture serial data to file:
    python calibration/capture.py --port /dev/cu.usbmodem* --output calibration/raw_data.txt

    # Or manually save Serial Monitor output to a .txt file, then:
    python calibration/analyze.py --input calibration/raw_data.txt

    # Outputs:
    #   calibration/noise_profile.yaml  — plug into sim
    #   calibration/noise_plots.png     — visual check
"""

import argparse
import sys
from pathlib import Path


def parse_burst_data(filepath: str) -> dict[float, list[float]]:
    """
    Parse calibration log file.
    Returns {actual_distance_cm: [list of valid readings]}.
    """
    data: dict[float, list[float]] = {}

    with open(filepath) as f:
        for line in f:
            line = line.strip()
            if not line.startswith("BURST,"):
                continue

            parts = line.split(",")
            if len(parts) < 5:
                continue

            try:
                actual = float(parts[1])
                distance = float(parts[4])
            except ValueError:
                continue

            if actual not in data:
                data[actual] = []

            # -1.0 = dropout/timeout
            data[actual].append(distance)

    return data


def compute_noise_profile(data: dict[float, list[float]]) -> list[dict]:
    """
    Compute noise statistics at each distance.
    Returns list of {actual, mean, std, bias, dropout_rate, min, max, n_valid, n_total}.
    """
    import numpy as np

    profile = []
    for actual in sorted(data.keys()):
        readings = data[actual]
        total = len(readings)
        valid = [r for r in readings if r > 0]
        n_valid = len(valid)
        dropouts = total - n_valid

        if n_valid == 0:
            profile.append({
                "actual": actual, "mean": 0, "std": 0, "bias": 0,
                "dropout_rate": 1.0, "min": 0, "max": 0,
                "n_valid": 0, "n_total": total,
            })
            continue

        arr = np.array(valid)
        mean = float(np.mean(arr))
        std = float(np.std(arr, ddof=1)) if n_valid > 1 else 0.0
        bias = mean - actual

        profile.append({
            "actual": actual,
            "mean": round(mean, 2),
            "std": round(std, 2),
            "bias": round(bias, 2),
            "dropout_rate": round(dropouts / total, 3) if total > 0 else 0,
            "min": round(float(np.min(arr)), 2),
            "max": round(float(np.max(arr)), 2),
            "n_valid": n_valid,
            "n_total": total,
        })

    return profile


def generate_noise_config(profile: list[dict], output_path: str):
    """Generate a YAML noise profile config for the simulation."""
    import yaml

    # Fit a simple linear model: noise_std = a + b * distance
    if len(profile) >= 2:
        distances = [p["actual"] for p in profile if p["n_valid"] > 5]
        stds = [p["std"] for p in profile if p["n_valid"] > 5]

        if len(distances) >= 2:
            import numpy as np
            coeffs = np.polyfit(distances, stds, 1)
            noise_slope = round(float(coeffs[0]), 4)
            noise_base = round(float(coeffs[1]), 2)
        else:
            noise_slope = 0.01
            noise_base = 1.0
    else:
        noise_slope = 0.01
        noise_base = 1.0

    # Average dropout rate
    dropout_rates = [p["dropout_rate"] for p in profile if p["n_total"] > 10]
    avg_dropout = round(sum(dropout_rates) / len(dropout_rates), 3) if dropout_rates else 0.02

    # Average bias
    biases = [p["bias"] for p in profile if p["n_valid"] > 5]
    avg_bias = round(sum(biases) / len(biases), 2) if biases else 0.0

    # Min range (find closest distance with >50% valid readings)
    min_range = 2.0
    for p in profile:
        if p["actual"] <= 5 and p["dropout_rate"] > 0.5:
            min_range = max(min_range, p["actual"] + 1)

    config = {
        "sensor_noise": {
            "model": "distance_dependent",
            "noise_base_std": noise_base,
            "noise_per_cm": noise_slope,
            "bias_cm": avg_bias,
            "dropout_chance": avg_dropout,
            "dropout_value": 0.0,
            "min_range_cm": min_range,
            "max_reliable_cm": 200.0,
        },
        "calibration_data": {
            "measurements": [
                {
                    "distance_cm": p["actual"],
                    "mean_reading": p["mean"],
                    "std": p["std"],
                    "bias": p["bias"],
                    "dropout_rate": p["dropout_rate"],
                    "samples": p["n_total"],
                }
                for p in profile
            ]
        },
    }

    with open(output_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    print(f"Noise config saved to: {output_path}")
    return config


def plot_noise_profile(profile: list[dict], output_path: str):
    """Generate plots of the noise profile."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib not installed — skipping plots. Install with: pip install matplotlib")
        return

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("HC-SR04 Calibration Profile", fontsize=14, fontweight="bold")

    distances = [p["actual"] for p in profile]
    means = [p["mean"] for p in profile]
    stds = [p["std"] for p in profile]
    biases = [p["bias"] for p in profile]
    dropouts = [p["dropout_rate"] * 100 for p in profile]

    # 1. Accuracy: actual vs measured
    ax = axes[0, 0]
    ax.plot(distances, means, "o-", color="#3B8BD4", markersize=6, label="Measured mean")
    ax.plot(distances, distances, "--", color="#999", label="Perfect (y=x)")
    ax.fill_between(distances,
                     [m - s for m, s in zip(means, stds)],
                     [m + s for m, s in zip(means, stds)],
                     alpha=0.2, color="#3B8BD4", label="±1 std")
    ax.set_xlabel("Actual distance (cm)")
    ax.set_ylabel("Measured distance (cm)")
    ax.set_title("Accuracy")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    # 2. Noise (std) vs distance
    ax = axes[0, 1]
    ax.bar(distances, stds, width=max(3, min(10, (max(distances) - min(distances)) / len(distances) * 0.6)),
           color="#D85A30", alpha=0.8)
    ax.set_xlabel("Actual distance (cm)")
    ax.set_ylabel("Std deviation (cm)")
    ax.set_title("Noise vs distance")
    ax.grid(True, alpha=0.3)

    # 3. Bias vs distance
    ax = axes[1, 0]
    colors = ["#1D9E75" if b >= 0 else "#D85A30" for b in biases]
    ax.bar(distances, biases, width=max(3, min(10, (max(distances) - min(distances)) / len(distances) * 0.6)),
           color=colors, alpha=0.8)
    ax.axhline(0, color="#999", linewidth=0.5)
    ax.set_xlabel("Actual distance (cm)")
    ax.set_ylabel("Bias (cm)")
    ax.set_title("Measurement bias (measured - actual)")
    ax.grid(True, alpha=0.3)

    # 4. Dropout rate vs distance
    ax = axes[1, 1]
    ax.bar(distances, dropouts, width=max(3, min(10, (max(distances) - min(distances)) / len(distances) * 0.6)),
           color="#7F77DD", alpha=0.8)
    ax.set_xlabel("Actual distance (cm)")
    ax.set_ylabel("Dropout rate (%)")
    ax.set_title("Failed readings")
    ax.set_ylim(0, max(max(dropouts) * 1.2, 5))
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Plots saved to: {output_path}")


def print_summary(profile: list[dict]):
    """Print a human-readable summary table."""
    print("\n╔══════════════════════════════════════════════════════════════════╗")
    print("║                   CALIBRATION RESULTS                          ║")
    print("╠══════════╦══════════╦═════════╦═════════╦═════════╦════════════╣")
    print("║ Actual   ║ Mean     ║ Std     ║ Bias    ║ Dropout ║ Samples    ║")
    print("╠══════════╬══════════╬═════════╬═════════╬═════════╬════════════╣")
    for p in profile:
        print(f"║ {p['actual']:7.1f}  ║ {p['mean']:7.2f}  ║ {p['std']:6.2f}  ║ {p['bias']:+6.2f}  ║ {p['dropout_rate']*100:5.1f}%  ║ {p['n_valid']:4d}/{p['n_total']:<4d}  ║")
    print("╚══════════╩══════════╩═════════╩═════════╩═════════╩════════════╝")


def main():
    parser = argparse.ArgumentParser(description="Analyze HC-SR04 calibration data")
    parser.add_argument("--input", required=True, help="Path to serial log file")
    parser.add_argument("--output-config", default="calibration/noise_profile.yaml", help="Output noise config")
    parser.add_argument("--output-plot", default="calibration/noise_plots.png", help="Output plot image")
    args = parser.parse_args()

    print(f"Reading: {args.input}")
    data = parse_burst_data(args.input)

    if not data:
        print("ERROR: No BURST data found in file. Make sure the file contains")
        print("lines starting with 'BURST,' from the Arduino calibration sketch.")
        sys.exit(1)

    print(f"Found data for {len(data)} distances: {sorted(data.keys())} cm")

    profile = compute_noise_profile(data)
    print_summary(profile)

    Path(args.output_config).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output_plot).parent.mkdir(parents=True, exist_ok=True)

    generate_noise_config(profile, args.output_config)
    plot_noise_profile(profile, args.output_plot)

    print("\nNext steps:")
    print("  1. Check the plots to make sure the data looks reasonable")
    print("  2. Copy noise_profile.yaml values into configs/physics.yaml")
    print("  3. Or load it directly in the sim with --calibration flag")


if __name__ == "__main__":
    main()
