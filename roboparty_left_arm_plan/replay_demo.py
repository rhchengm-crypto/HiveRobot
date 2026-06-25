import argparse
import csv
import time

from damiao_left_arm import LeftArm


def load_rows(path):
    with open(path, "r", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def main():
    parser = argparse.ArgumentParser(description="Replay a recorded left-arm target trajectory.")
    parser.add_argument("--config", default=None)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--speed-scale", type=float, default=0.4)
    parser.add_argument("--max-step", type=float, default=0.025)
    args = parser.parse_args()

    rows = load_rows(args.csv)
    if not rows:
        raise RuntimeError("empty csv")

    arm = LeftArm(args.config)
    arm.connect()
    arm.enable_all()
    joint_names = [spec.name for spec in arm.specs]

    targets = arm.home_targets()
    print("settle to configured home")
    arm.settle(targets, timeout=15.0)

    try:
        last_t = float(rows[0]["t"])
        for row in rows:
            phase = row.get("phase", "")
            if phase.startswith("hold"):
                continue

            dt = (float(row["t"]) - last_t) / max(args.speed_scale, 1e-6)
            last_t = float(row["t"])
            time.sleep(min(max(dt, 0.0), 0.08))

            next_targets = {}
            for name in joint_names:
                raw = float(row[f"{name}.target_q"])
                delta = raw - targets[name]
                if delta > args.max_step:
                    raw = targets[name] + args.max_step
                elif delta < -args.max_step:
                    raw = targets[name] - args.max_step
                next_targets[name] = raw

            targets = arm.clamp_targets(next_targets)
            arm.command_targets(targets, moving_name=row.get("active_joint"))

        print("final settle")
        arm.settle(arm.home_targets(), timeout=20.0)
        print("hold at home, Ctrl+C to stop")
        while True:
            arm.command_targets(arm.home_targets())
            time.sleep(arm.period)

    except KeyboardInterrupt:
        print("stop requested")
    finally:
        arm.ramp_down_hold(arm.home_targets())


if __name__ == "__main__":
    main()

