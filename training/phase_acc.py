"""
Per-phase (tape vs maze) frame agreement of a policy on expert sequences.

Diagnostic, not selection (closed-loop remains the only score): if the
maze-phase agreement is much lower than tape-phase, the expert's maze
behavior is intrinsically ambiguous from the observations (fix = more
observability or auxiliary supervision); if both are high while closed-loop
is 0, distribution shift is the whole problem (fix = DAgger-style data).

Teacher-forced: features come from the recorded expert episodes, so this is
an on-distribution upper bound of agreement.

Usage:
    python -m training.phase_acc --policy models/policy_gru_v1.npz \
        [--policy2 models/policy_bc.npz] --data data/train.csv
"""

import argparse
import json

import numpy as np

from policy_runtime import load_policy
from training.train_gru import load_sequences

CMD_NAMES = ["FWD", "SL_L", "SL_R", "HD_L", "HD_R", "STOP"]


def eval_policy_phases(policy_path: str, episodes, names,
                       tape_end_norm: float, sample_every: int = 5):
    di = names.index("distance")
    pol = load_policy(policy_path)
    is_recurrent = hasattr(pol, "reset")

    rows = {"tape": {"y": [], "p": []}, "maze": {"y": [], "p": []}}
    for k in range(0, len(episodes), sample_every):
        X, y = episodes[k]
        if is_recurrent:
            pol.reset()
            preds = np.empty(len(y), dtype=np.int64)
            for t in range(len(y)):
                preds[t] = int(np.argmax(pol.predict_proba(X[t])))
        else:
            preds = np.argmax(pol.predict_proba(X), axis=-1)
        phase = np.where(X[:, di] < tape_end_norm, "tape", "maze")
        for ph in ("tape", "maze"):
            m = phase == ph
            rows[ph]["y"].append(y[m])
            rows[ph]["p"].append(preds[m])

    out = {}
    for ph in ("tape", "maze"):
        yy = np.concatenate(rows[ph]["y"])
        pp = np.concatenate(rows[ph]["p"])
        per_class = {}
        accs = []
        for c in range(6):
            m = yy == c
            if m.sum() == 0:
                continue
            a = float((pp[m] == c).mean())
            per_class[CMD_NAMES[c]] = (a, int(m.sum()))
            accs.append(a)
        out[ph] = {"balanced": float(np.mean(accs)), "n": int(len(yy)),
                   "per_class": per_class}
    return out


def main():
    ap = argparse.ArgumentParser(description="Tape-vs-maze frame agreement")
    ap.add_argument("--policy", required=True)
    ap.add_argument("--policy2", default=None)
    ap.add_argument("--data", default="data/train.csv")
    ap.add_argument("--tape-end-odom", type=float, default=725.0,
                    help="Odometer cm at tape end (measured ~721-731)")
    ap.add_argument("--sample-every", type=int, default=5)
    args = ap.parse_args()

    episodes, names = load_sequences(args.data)
    tape_end_norm = min(4.0, args.tape_end_odom / 500.0)  # FeatureBuilder scale

    for path in filter(None, [args.policy, args.policy2]):
        print(f"\n== {path} ==")
        res = eval_policy_phases(path, episodes, names, tape_end_norm,
                                 args.sample_every)
        for ph in ("tape", "maze"):
            r = res[ph]
            print(f"  {ph:5s} balanced={r['balanced']:.3f}  ({r['n']} rows)")
            for cname, (a, n) in r["per_class"].items():
                print(f"        {cname:5s} {a:.3f}  (n={n})")


if __name__ == "__main__":
    main()
