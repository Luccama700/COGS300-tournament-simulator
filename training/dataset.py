"""
CSV dataset loader for line-following training data.

Skips the episode_id column if present — it's a metadata column,
not a training feature.
"""

import csv
from pathlib import Path


# Columns that are never model inputs
_SKIP_COLS = {"episode_id"}

# Label column
_LABEL_COL = "command_label"


def load_csv(path):
    """
    Load a training CSV produced by generate_line_data.py.

    Returns:
        features  — list of float rows (episode_id and command_label removed)
        labels    — list of int command labels
        feat_cols — column names matching the feature rows
    """
    path = Path(path)
    features: list[list[float]] = []
    labels:   list[int]         = []

    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        assert reader.fieldnames, f"Empty CSV: {path}"

        feat_cols = [c for c in reader.fieldnames
                     if c != _LABEL_COL and c not in _SKIP_COLS]

        for row in reader:
            features.append([float(row[c]) for c in feat_cols])
            labels.append(int(row[_LABEL_COL]))

    return features, labels, feat_cols


def load_numpy(path):
    """
    Same as load_csv but returns numpy arrays.
    Requires numpy to be installed.
    """
    import numpy as np
    features, labels, feat_cols = load_csv(path)
    return np.array(features, dtype=np.float32), np.array(labels, dtype=np.int64), feat_cols
