"""
Merge multiple training CSVs (same schema) into one, renumbering episode ids
so they stay unique. Used to combine the DAgger aggregate with extra
randomization sweeps before a final training pass.

Usage:
    python -m training.merge_datasets --out data/final_train.csv \
        data/dagger/aggregate.csv data/bc_heavy.csv
"""

import argparse
import csv
import json
import shutil


def main():
    ap = argparse.ArgumentParser(description="Merge training CSVs")
    ap.add_argument("inputs", nargs="+")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    header = None
    next_ep = 0
    rows_out = 0
    with open(args.out, "w", newline="") as fout:
        writer = csv.writer(fout)
        for path in args.inputs:
            with open(path, newline="") as fin:
                reader = csv.reader(fin)
                h = next(reader)
                if header is None:
                    header = h
                    writer.writerow(h)
                    ep_col = h.index("episode_id")
                elif h != header:
                    raise SystemExit(f"schema mismatch in {path}")
                remap: dict[str, int] = {}
                for row in reader:
                    if row[ep_col] not in remap:
                        remap[row[ep_col]] = next_ep
                        next_ep += 1
                    row[ep_col] = str(remap[row[ep_col]])
                    writer.writerow(row)
                    rows_out += 1
            print(f"  + {path}: {len(remap)} episodes")

    # Carry over the schema file from the first input
    schema_src = args.inputs[0].replace(".csv", "_schema.json")
    schema_dst = args.out.replace(".csv", "_schema.json")
    try:
        shutil.copyfile(schema_src, schema_dst)
    except FileNotFoundError:
        print(f"  (no schema at {schema_src} — copy one manually)")
    print(f"{rows_out} rows, {next_ep} episodes -> {args.out}")


if __name__ == "__main__":
    main()
