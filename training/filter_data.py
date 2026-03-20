"""
Filter a combined training CSV using episode quality metadata.

Usage:
    python -m training.filter_data \\
        --input data/line_train.csv \\
        --meta  data/line_train_meta.json \\
        --output data/line_train_filtered.csv \\
        --strategy best_percent --percent 10
"""

import argparse
import csv
import json
import sys


def quality_score(ep: dict) -> float:
    """
    Higher is better.
    Penalizes slow runs, off-line time, and jerky steering.
    """
    return -ep["steps"] * 0.4 - ep["off_line_frames"] * 0.3 - ep["avg_steering"] * 0.3


def select_episodes(all_eps: list[dict], strategy: str, percent: float) -> list[dict]:
    successful = [ep for ep in all_eps if ep["reached_goal"]]

    if strategy == "all_successful":
        return successful

    k = max(1, round(len(successful) * percent / 100))

    if strategy == "best_percent":
        ranked = sorted(successful, key=quality_score, reverse=True)
    elif strategy == "fastest":
        ranked = sorted(successful, key=lambda e: e["steps"])
    elif strategy == "smoothest":
        ranked = sorted(successful, key=lambda e: e["avg_steering"])
    else:
        raise ValueError(f"Unknown strategy: {strategy}")

    return ranked[:k]


def main():
    parser = argparse.ArgumentParser(description="Filter training data by episode quality")
    parser.add_argument("--input",    required=True, help="Combined CSV from generate_line_data.py")
    parser.add_argument("--meta",     required=True, help="Metadata JSON (same base name as CSV)")
    parser.add_argument("--output",   required=True, help="Filtered output CSV path")
    parser.add_argument("--strategy", default="best_percent",
                        choices=["best_percent", "fastest", "smoothest", "all_successful"])
    parser.add_argument("--percent",  type=float, default=20.0,
                        help="Percentage of episodes to keep (ignored for all_successful)")
    args = parser.parse_args()

    with open(args.meta) as f:
        meta = json.load(f)
    all_eps: list[dict] = meta["episodes"]

    kept = select_episodes(all_eps, args.strategy, args.percent)
    kept_ids = {ep["id"] for ep in kept}

    if not kept:
        print("ERROR: no episodes matched the filter criteria.", file=sys.stderr)
        sys.exit(1)

    # Stats
    discarded = [ep for ep in all_eps if ep["id"] not in kept_ids]
    kept_scores    = [quality_score(ep) for ep in kept]
    discard_scores = [quality_score(ep) for ep in discarded] if discarded else []
    avg_kept    = sum(kept_scores)    / len(kept_scores)
    avg_discard = sum(discard_scores) / len(discard_scores) if discard_scores else float("nan")

    print(f"Strategy:        {args.strategy}" + (f"  ({args.percent}%)" if args.strategy != "all_successful" else ""))
    print(f"Total episodes:  {len(all_eps)}  ({sum(1 for e in all_eps if e['reached_goal'])} reached goal)")
    print(f"Keeping:         {len(kept)} episodes")
    print(f"Discarding:      {len(discarded)} episodes")
    print(f"Avg quality — kept: {avg_kept:.1f}  discarded: {avg_discard:.1f}  (higher is better)")

    # Stream the CSV, writing only rows from kept episodes
    # Rows are identified by episode_id column (last column)
    rows_written = 0
    with open(args.input, newline="") as fin, open(args.output, "w", newline="") as fout:
        reader = csv.reader(fin)
        writer = csv.writer(fout)

        header = next(reader)
        if "episode_id" not in header:
            print("ERROR: input CSV has no episode_id column. "
                  "Re-generate with current generate_line_data.py.", file=sys.stderr)
            sys.exit(1)

        # Drop episode_id from output — it's not a training feature
        ep_col = header.index("episode_id")
        out_header = [c for c in header if c != "episode_id"]
        writer.writerow(out_header)

        for row in reader:
            if int(row[ep_col]) in kept_ids:
                writer.writerow([v for i, v in enumerate(row) if i != ep_col])
                rows_written += 1

    print(f"Rows written:    {rows_written}  → {args.output}")


if __name__ == "__main__":
    main()
