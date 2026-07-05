"""
Train a recurrent (GRU) command policy on expert episode sequences.

Why recurrence: the expert is a hybrid state machine; the 26 hand-crafted
features expose the tape phase's hidden state (turn_accum, ir_mem, ...) but
not the maze phase's. A feedforward clone therefore rides the tape and then
degrades over the ~1,200-decision maze horizon. A GRU lets the network carry
its own memory instead of relying on hand-made state
(docs/AGENT_HANDOFF.md hypothesis #1).

Two rules this trainer enforces (paid-for lessons):
  * Checkpoints are selected by CLOSED-LOOP evaluation (success count, then
    median closest-approach), never by validation accuracy
    (docs/lessons/closed-loop-is-the-only-score.md). Every select_every
    epochs the current weights are exported and driven for select_episodes
    episodes; the best checkpoint wins. Selection seeds (20000+) are disjoint
    from the final claim seeds (31000+).
  * Hidden state in training matches inference: h starts at zero at the
    EPISODE start and is carried (detached) across BPTT chunks, exactly like
    the deployed GRUPolicy carries it across decision steps.

Training uses torch (GPU if available); inference stays numpy-only — weights
are exported to .npz and executed by policy_runtime.GRUPolicy.
train_gru_core() is importable (used by the DAgger loop for its retrains).

Usage:
    python -m training.train_gru --data data/train.csv --out models/policy_gru.npz
"""

import argparse
import csv
import json
import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

from training.closed_loop import (run_closed_loop, score_key,
                                  TRACK_DEFAULT, ROBOT_DEFAULT, PHYS_DEFAULT)


# ── Data ─────────────────────────────────────────────────────────────────────

def load_sequences(csv_path: str):
    """
    Load a generate_data_v2 CSV as per-episode sequences.
    Returns (episodes: list[(X (T,F) f32, y (T,) i64)], feature_names).
    Caches the parsed arrays in an .npz next to the CSV (parsing 300k CSV
    rows costs ~20s; training runs happen many times per session).
    """
    cache = csv_path + ".seqcache.npz"
    if os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(csv_path):
        z = np.load(cache, allow_pickle=False)
        X, y, ep = z["X"], z["y"], z["ep"]
        names = json.loads(str(z["names_json"]))
    else:
        with open(csv_path, newline="") as f:
            reader = csv.reader(f)
            header = next(reader)
            li = header.index("command_label")
            ei = header.index("episode_id")
            feat_idx = [i for i in range(len(header)) if i not in (li, ei)]
            names = [header[i] for i in feat_idx]
            Xr, yr, er = [], [], []
            for r in reader:
                Xr.append([float(r[i]) for i in feat_idx])
                yr.append(int(r[li]))
                er.append(int(r[ei]))
        X = np.asarray(Xr, dtype=np.float32)
        y = np.asarray(yr, dtype=np.int64)
        ep = np.asarray(er, dtype=np.int64)
        np.savez(cache, X=X, y=y, ep=ep, names_json=json.dumps(names))

    episodes = []
    # rows are written episode-contiguous by the generators; keep file order
    change = np.nonzero(np.diff(ep))[0] + 1
    for xs, ys in zip(np.split(X, change), np.split(y, change)):
        episodes.append((xs, ys))
    return episodes, names


def make_batches(episodes, idxs, batch_size):
    """Bucket episodes by length into padded (X, y) batches (-100 = pad)."""
    order = sorted(idxs, key=lambda i: len(episodes[i][1]))
    batches = []
    for i in range(0, len(order), batch_size):
        group = order[i:i + batch_size]
        T = max(len(episodes[j][1]) for j in group)
        F_ = episodes[group[0]][0].shape[1]
        X = np.zeros((len(group), T, F_), dtype=np.float32)
        y = np.full((len(group), T), -100, dtype=np.int64)
        for k, j in enumerate(group):
            xs, ys = episodes[j]
            X[k, :len(ys)] = xs
            y[k, :len(ys)] = ys
        batches.append((torch.from_numpy(X), torch.from_numpy(y)))
    return batches


# ── Model ────────────────────────────────────────────────────────────────────

class GRUNet(nn.Module):
    def __init__(self, n_in: int, hidden: int, n_out: int = 6):
        super().__init__()
        self.gru = nn.GRU(n_in, hidden, batch_first=True)
        self.head = nn.Linear(hidden, n_out)

    def forward(self, x, h=None):
        out, h = self.gru(x, h)
        return self.head(out), h


def export_npz(model: GRUNet, mean, std, meta: dict, path: str):
    sd = {k: v.detach().cpu().numpy() for k, v in model.state_dict().items()}
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    np.savez(path,
             W_ih=sd["gru.weight_ih_l0"], W_hh=sd["gru.weight_hh_l0"],
             b_ih=sd["gru.bias_ih_l0"], b_hh=sd["gru.bias_hh_l0"],
             W_out=sd["head.weight"], b_out=sd["head.bias"],
             mean=mean.astype(np.float32), std=std.astype(np.float32),
             meta_json=json.dumps(meta))


def parity_check(model: GRUNet, mean, std, meta, tmp_path: str) -> float:
    """Torch forward vs numpy GRUPolicy forward on a random sequence."""
    from policy_runtime import GRUPolicy
    export_npz(model, mean, std, meta, tmp_path)
    pol = GRUPolicy.load(tmp_path)
    rng = np.random.default_rng(0)
    seq = rng.normal(0.0, 1.0, (50, len(mean))).astype(np.float32)
    seq_feats = seq * std + mean                     # GRUPolicy re-normalizes
    model_cpu = GRUNet(len(mean), meta["hidden"]).to("cpu")
    model_cpu.load_state_dict({k: v.cpu() for k, v in model.state_dict().items()})
    model_cpu.eval()
    with torch.no_grad():
        logits, _ = model_cpu(torch.from_numpy(seq).unsqueeze(0))
        probs_t = F.softmax(logits[0], dim=-1).numpy()
    worst = 0.0
    for t in range(len(seq)):
        p = pol.predict_proba(seq_feats[t])
        worst = max(worst, float(np.abs(p - probs_t[t]).max()))
    return worst


# ── Training core (importable — DAgger retrains call this) ──────────────────

def balanced_acc(model, batches, device, n_classes=6):
    hits = np.zeros(n_classes)
    tot = np.zeros(n_classes)
    model.eval()
    with torch.no_grad():
        for X, y in batches:
            logits, _ = model(X.to(device))
            pred = logits.argmax(-1).cpu().numpy()
            yy = y.numpy()
            for c in range(n_classes):
                m = yy == c
                tot[c] += m.sum()
                hits[c] += (pred[m] == c).sum()
    model.train()
    per = hits[tot > 0] / tot[tot > 0]
    return float(per.mean())


def train_gru_core(episodes, schema: dict, ckpt_dir: str,
                   hidden=96, epochs=40, batch=32, bptt=64, lr=1e-3,
                   seed=0, val_frac=0.1,
                   select_every=4, select_episodes=10, select_seed=20000,
                   select_workers=4, select_randomization="mild",
                   track=TRACK_DEFAULT, robot=ROBOT_DEFAULT, phys=PHYS_DEFAULT,
                   run_parity=False, tag=""):
    """
    Train on (X, y) episode sequences; select checkpoints closed-loop.
    Returns (best_key, best_path, best_result, history).
    """
    device = "cuda" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(seed)
    rng = np.random.default_rng(seed)
    os.makedirs(ckpt_dir, exist_ok=True)

    idxs = np.arange(len(episodes))
    rng.shuffle(idxs)
    n_val = max(1, int(len(idxs) * val_frac))
    val_idx, tr_idx = idxs[:n_val], idxs[n_val:]

    Xtr_rows = np.concatenate([episodes[i][0] for i in tr_idx])
    ytr_rows = np.concatenate([episodes[i][1] for i in tr_idx])
    n_rows = sum(len(y) for _, y in episodes)
    mean = Xtr_rows.mean(axis=0)
    std = Xtr_rows.std(axis=0) + 1e-6

    counts = np.bincount(ytr_rows, minlength=6).astype(np.float64)
    w = np.where(counts > 0, counts.sum() / (6 * np.maximum(counts, 1)), 0.0)
    w = np.clip(w, 0.0, 20.0)
    weight_t = torch.tensor(w, dtype=torch.float32, device=device)

    normed = [(((xs - mean) / std).astype(np.float32), ys) for xs, ys in episodes]
    tr_batches = make_batches(normed, tr_idx, batch)
    va_batches = make_batches(normed, val_idx, batch)

    model = GRUNet(len(mean), hidden).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=lr)
    meta = {**schema, "arch": "gru", "hidden": hidden, "train_rows": int(n_rows)}

    if run_parity:
        diff = parity_check(model, mean, std, meta,
                            os.path.join(ckpt_dir, "_parity.npz"))
        print(f"torch-vs-numpy parity (max |dprob|): {diff:.2e}")
        assert diff < 1e-4, "numpy GRU forward does not match torch!"

    best = None      # (score_key, epoch, path, result)
    history = []
    for epoch in range(1, epochs + 1):
        t0 = time.time()
        order = rng.permutation(len(tr_batches))
        tot_loss, tot_n = 0.0, 0
        for bi in order:
            X, y = tr_batches[bi]
            X, y = X.to(device), y.to(device)
            h = None
            for c0 in range(0, X.shape[1], bptt):
                xs = X[:, c0:c0 + bptt]
                ys = y[:, c0:c0 + bptt]
                nvalid = int((ys != -100).sum())
                if nvalid == 0:
                    break
                logits, h = model(xs, h)
                loss = F.cross_entropy(logits.reshape(-1, 6), ys.reshape(-1),
                                       weight=weight_t, ignore_index=-100)
                opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), 1.0)
                opt.step()
                h = h.detach()
                tot_loss += loss.item() * nvalid
                tot_n += nvalid
        msg = f"{tag}epoch {epoch:3d}  loss={tot_loss / max(1, tot_n):.4f}  ({time.time() - t0:.1f}s)"

        if epoch % select_every == 0 or epoch == epochs:
            bal = balanced_acc(model, va_batches, device)
            path = os.path.join(ckpt_dir, f"gru{tag.strip()}_e{epoch}.npz")
            export_npz(model, mean, std,
                       {**meta, "epoch": epoch, "val_balanced_acc": bal}, path)
            res = run_closed_loop(path, track, robot, phys,
                                  episodes=select_episodes, seed=select_seed,
                                  randomization=select_randomization,
                                  workers=select_workers)
            key = score_key(res)
            history.append({"epoch": epoch, "val_balanced_acc": bal, **res})
            star = ""
            if best is None or key > best[0]:
                best = (key, epoch, path, res)
                star = "  ** new best"
            arcpct = (100.0 * res["median_max_arc"] / res["route_total"]
                      if res.get("route_total") else 0.0)
            msg += (f"  val_bal={bal:.3f}  CLOSED-LOOP: {res['success']}/{res['episodes']}"
                    f" arc={arcpct:.0f}%"
                    f" median_min={res['median_min_goal_dist']:.0f}cm"
                    f" spins={res['spin_events']}{star}")
        print(msg, flush=True)
    return best[0], best[2], best[3], history


# ── CLI ──────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser(description="Train GRU policy (torch) with "
                                             "closed-loop checkpoint selection")
    ap.add_argument("--data", default="data/train.csv")
    ap.add_argument("--out", default="models/policy_gru.npz")
    ap.add_argument("--hidden", type=int, default=96)
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--batch", type=int, default=32)
    ap.add_argument("--bptt", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--val-frac", type=float, default=0.1)
    ap.add_argument("--select-every", type=int, default=4)
    ap.add_argument("--select-episodes", type=int, default=10)
    ap.add_argument("--select-seed", type=int, default=20000)
    ap.add_argument("--select-workers", type=int, default=4)
    ap.add_argument("--select-randomization", default="mild")
    ap.add_argument("--track", default=TRACK_DEFAULT)
    ap.add_argument("--robot", default=ROBOT_DEFAULT)
    ap.add_argument("--physics", default=PHYS_DEFAULT)
    args = ap.parse_args()

    schema_path = args.data.replace(".csv", "_schema.json")
    with open(schema_path) as f:
        schema = json.load(f)

    print(f"loading {args.data} ...")
    episodes, names = load_sequences(args.data)
    assert names == schema["feature_names"], "CSV columns don't match schema!"
    n_rows = sum(len(y) for _, y in episodes)
    print(f"{len(episodes)} episodes, {n_rows} rows, "
          f"{episodes[0][0].shape[1]} features")

    ckpt_dir = os.path.join(os.path.dirname(args.out) or ".", "gru_ckpts")
    key, path, res, history = train_gru_core(
        episodes, schema, ckpt_dir,
        hidden=args.hidden, epochs=args.epochs, batch=args.batch,
        bptt=args.bptt, lr=args.lr, seed=args.seed, val_frac=args.val_frac,
        select_every=args.select_every, select_episodes=args.select_episodes,
        select_seed=args.select_seed, select_workers=args.select_workers,
        select_randomization=args.select_randomization,
        track=args.track, robot=args.robot, phys=args.physics,
        run_parity=True)

    data = np.load(path, allow_pickle=False)
    m = json.loads(str(data["meta_json"]))
    m["selection"] = {k: res[k] for k in
                      ("success", "episodes", "median_min_goal_dist", "seed",
                       "randomization")}
    np.savez(args.out, **{k: data[k] for k in data.files if k != "meta_json"},
             meta_json=json.dumps(m))
    print(f"\nbest checkpoint: {os.path.basename(path)} -- "
          f"{res['success']}/{res['episodes']} closed-loop, "
          f"median_min={res['median_min_goal_dist']:.0f}cm -> {args.out}")
    with open(args.out.replace(".npz", "_history.json"), "w") as f:
        json.dump(history, f, indent=2)


if __name__ == "__main__":
    main()
