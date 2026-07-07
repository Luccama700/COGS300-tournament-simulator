"""
Train the command-classification MLP on data from generate_data_v2.py.

Pure numpy (no torch/sklearn dependency): a 2-hidden-layer ReLU network with
softmax cross-entropy, Adam, inverse-frequency class weights, and early
stopping. Small enough to train in seconds and to run at 10Hz on a laptop
relaying UDP packets to the Arduino.

Design choices that matter:
  * Validation split is BY EPISODE, not by row. Rows within an episode are
    strongly correlated; a row-level split leaks and reports fantasy accuracy.
  * Class weights: FORWARD dominates line-following data; unweighted training
    yields a model that never turns hard (or, in the old broken dataset, one
    that only ever turned). Weighted loss keeps every command learnable.
  * The saved .npz embeds the normalization stats and the feature schema, so
    inference (policy_runtime.MLPPolicy) is self-contained.

Usage:
    python -m training.train --data data/bc_train.csv --out models/policy.npz
"""

import argparse
import csv
import json
import os

import numpy as np


def load_dataset(path: str):
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = [r for r in reader]
    li = header.index("command_label")
    ei = header.index("episode_id")
    feat_idx = [i for i in range(len(header)) if i not in (li, ei)]
    X = np.array([[float(r[i]) for i in feat_idx] for r in rows], dtype=np.float32)
    y = np.array([int(r[li]) for r in rows], dtype=np.int64)
    ep = np.array([int(r[ei]) for r in rows], dtype=np.int64)
    names = [header[i] for i in feat_idx]
    return X, y, ep, names


def train(X, y, ep, hidden=(48, 48), n_classes=6, lr=1e-3, batch=256,
          epochs=200, patience=15, val_frac=0.15, seed=0, class_weight=True):
    rng = np.random.default_rng(seed)

    # Episode-level split
    eps = np.unique(ep)
    rng.shuffle(eps)
    n_val = max(1, int(len(eps) * val_frac))
    val_eps = set(eps[:n_val].tolist())
    val_mask = np.isin(ep, list(val_eps))
    Xtr, ytr = X[~val_mask], y[~val_mask]
    Xva, yva = X[val_mask], y[val_mask]

    mean = Xtr.mean(axis=0)
    std = Xtr.std(axis=0) + 1e-6
    Xtr = (Xtr - mean) / std
    Xva = (Xva - mean) / std

    counts = np.bincount(ytr, minlength=n_classes).astype(np.float64)
    if class_weight:
        w = np.where(counts > 0, counts.sum() / (n_classes * np.maximum(counts, 1)), 0.0)
        w = np.clip(w, 0.0, 20.0)
    else:
        w = np.ones(n_classes)
    sample_w = w[ytr].astype(np.float32)

    sizes = [X.shape[1], *hidden, n_classes]
    Ws = [rng.normal(0, np.sqrt(2.0 / sizes[i]), (sizes[i], sizes[i + 1])).astype(np.float32)
          for i in range(len(sizes) - 1)]
    bs = [np.zeros(sizes[i + 1], dtype=np.float32) for i in range(len(sizes) - 1)]

    mW = [np.zeros_like(W) for W in Ws]; vW = [np.zeros_like(W) for W in Ws]
    mb = [np.zeros_like(b) for b in bs]; vb = [np.zeros_like(b) for b in bs]
    b1, b2, eps_adam = 0.9, 0.999, 1e-8
    t_adam = 0

    def forward(Z):
        acts = [Z]
        for W, b in zip(Ws[:-1], bs[:-1]):
            Z = np.maximum(0.0, Z @ W + b)
            acts.append(Z)
        Z = Z @ Ws[-1] + bs[-1]
        acts.append(Z)
        return acts

    def val_metrics():
        logits = forward(Xva)[-1]
        pred = logits.argmax(axis=1)
        acc = float((pred == yva).mean())
        # balanced accuracy — the metric that actually predicts driving quality
        per_class = [float((pred[yva == c] == c).mean()) for c in range(n_classes)
                     if (yva == c).sum() > 0]
        return acc, float(np.mean(per_class)), pred

    best = None
    best_bal = -1.0
    stale = 0
    n = len(Xtr)
    for epoch in range(epochs):
        order = rng.permutation(n)
        for i in range(0, n, batch):
            idx = order[i:i + batch]
            Zs = forward(Xtr[idx])
            logits = Zs[-1]
            logits = logits - logits.max(axis=1, keepdims=True)
            e = np.exp(logits)
            p = e / e.sum(axis=1, keepdims=True)
            Y = np.zeros_like(p)
            Y[np.arange(len(idx)), ytr[idx]] = 1.0
            sw = sample_w[idx][:, None]
            g = (p - Y) * sw / len(idx)

            t_adam += 1
            grads_W, grads_b = [], []
            for li in range(len(Ws) - 1, -1, -1):
                A = Zs[li]
                grads_W.append(A.T @ g)
                grads_b.append(g.sum(axis=0))
                if li > 0:
                    g = (g @ Ws[li].T) * (Zs[li] > 0)
            grads_W.reverse(); grads_b.reverse()
            for li in range(len(Ws)):
                for P, G, m, v in ((Ws[li], grads_W[li], mW[li], vW[li]),
                                   (bs[li], grads_b[li], mb[li], vb[li])):
                    m[:] = b1 * m + (1 - b1) * G
                    v[:] = b2 * v + (1 - b2) * G * G
                    mhat = m / (1 - b1 ** t_adam)
                    vhat = v / (1 - b2 ** t_adam)
                    P -= lr * mhat / (np.sqrt(vhat) + eps_adam)

        acc, bal, _ = val_metrics()
        if bal > best_bal + 1e-4:
            best_bal = bal
            best = ([W.copy() for W in Ws], [b.copy() for b in bs], acc, bal, epoch)
            stale = 0
        else:
            stale += 1
        if epoch % 10 == 0 or stale == 0:
            print(f"  epoch {epoch:3d}  val_acc={acc:.3f}  val_balanced={bal:.3f}"
                  + ("  *" if stale == 0 else ""))
        if stale >= patience:
            break

    Ws, bs, acc, bal, at = best
    print(f"best: epoch {at}  val_acc={acc:.3f}  val_balanced_acc={bal:.3f}")

    # Confusion matrix on validation
    logits = ((Xva @ Ws[0] + bs[0]).clip(0) @ Ws[1] + bs[1]).clip(0) @ Ws[2] + bs[2]
    pred = logits.argmax(axis=1)
    cm = np.zeros((n_classes, n_classes), dtype=int)
    for t_, p_ in zip(yva, pred):
        cm[t_, p_] += 1
    names = ["FWD", "SL_L", "SL_R", "HD_L", "HD_R", "STOP"]
    print("val confusion (rows=true):")
    print("        " + " ".join(f"{s:>6}" for s in names))
    for i, row in enumerate(cm):
        print(f"  {names[i]:>5} " + " ".join(f"{v:6d}" for v in row))

    return Ws, bs, mean, std, {"val_acc": acc, "val_balanced_acc": bal}


def main():
    ap = argparse.ArgumentParser(description="Train BC command classifier")
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", default="models/policy.npz")
    ap.add_argument("--schema", default=None,
                    help="Schema JSON from generate_data_v2 (default: infer from --data path)")
    ap.add_argument("--hidden", type=int, nargs=2, default=[48, 48])
    ap.add_argument("--epochs", type=int, default=200)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    schema_path = args.schema or args.data.replace(".csv", "_schema.json")
    with open(schema_path) as f:
        schema = json.load(f)

    print(f"loading {args.data} ...")
    X, y, ep, names = load_dataset(args.data)
    assert names == schema["feature_names"], "CSV columns don't match schema!"
    counts = np.bincount(y, minlength=6)
    print(f"{len(X)} rows, {len(np.unique(ep))} episodes, features={X.shape[1]}")
    print(f"labels: FWD:{counts[0]} SL_L:{counts[1]} SL_R:{counts[2]} "
          f"HD_L:{counts[3]} HD_R:{counts[4]} STOP:{counts[5]}")

    Ws, bs, mean, std, metrics = train(X, y, ep, hidden=tuple(args.hidden),
                                       epochs=args.epochs, seed=args.seed)

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    meta = {**schema, **metrics, "hidden": args.hidden, "train_rows": int(len(X))}
    payload = {"n_layers": len(Ws), "mean": mean, "std": std,
               "meta_json": json.dumps(meta)}
    for i, (W, b) in enumerate(zip(Ws, bs)):
        payload[f"W{i}"] = W
        payload[f"b{i}"] = b
    np.savez(args.out, **payload)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
