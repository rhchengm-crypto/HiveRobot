import argparse
import csv
import json

import numpy as np


def discover_joints(fieldnames):
    joints = []
    for name in fieldnames:
        if name.endswith(".q"):
            joints.append(name[:-2])
    return joints


def main():
    parser = argparse.ArgumentParser(description="Train a tiny ridge-regression behavior-cloning baseline.")
    parser.add_argument("--csv", required=True)
    parser.add_argument("--out", default="data/left_arm_bc_model.json")
    parser.add_argument("--lambda", dest="ridge_lambda", type=float, default=1e-3)
    args = parser.parse_args()

    with open(args.csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = [row for row in reader if row.get("phase") in ("move", "return")]
        joints = discover_joints(reader.fieldnames)

    if len(rows) < 20:
        raise RuntimeError("not enough rows for training")

    x = []
    y = []
    for row in rows:
        feat = []
        target = []
        for joint in joints:
            feat += [
                float(row[f"{joint}.q"]),
                float(row[f"{joint}.dq"]),
                float(row[f"{joint}.tau"]),
            ]
            target.append(float(row[f"{joint}.target_q"]))
        feat.append(1.0)
        x.append(feat)
        y.append(target)

    x = np.asarray(x, dtype=np.float64)
    y = np.asarray(y, dtype=np.float64)

    xtx = x.T @ x
    reg = args.ridge_lambda * np.eye(xtx.shape[0])
    weights = np.linalg.solve(xtx + reg, x.T @ y)
    pred = x @ weights
    rmse = np.sqrt(np.mean((pred - y) ** 2, axis=0))

    model = {
        "type": "ridge_bc",
        "input": ["q", "dq", "tau"] * len(joints) + ["bias"],
        "joints": joints,
        "weights": weights.tolist(),
        "rmse_by_joint": {joint: float(err) for joint, err in zip(joints, rmse)},
    }

    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(model, f, ensure_ascii=False, indent=2)

    print("saved", args.out)
    for joint, err in model["rmse_by_joint"].items():
        print(joint, "rmse(rad)=", err)


if __name__ == "__main__":
    main()

