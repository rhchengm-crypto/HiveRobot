import argparse
import csv
import os
import time

from damiao_left_arm import LeftArm


def build_header(joint_names):
    header = ["t", "phase", "active_joint"]
    for name in joint_names:
        header += [f"{name}.q", f"{name}.dq", f"{name}.tau", f"{name}.target_q"]
    return header


def row_from_state(t0, phase, active_joint, joint_names, state, targets):
    row = [time.time() - t0, phase, active_joint]
    for name in joint_names:
        s = state[name]
        row += [s["q"], s["dq"], s["tau"], targets[name]]
    return row


def main():
    parser = argparse.ArgumentParser(description="Collect a safe scripted left-arm demonstration.")
    parser.add_argument("--config", default=None)
    parser.add_argument("--out", default="data/left_arm_scripted_demo.csv")
    parser.add_argument("--amp", type=float, default=0.10)
    parser.add_argument("--freq", type=float, default=0.20)
    parser.add_argument("--move-time", type=float, default=3.0)
    parser.add_argument("--settle-tol", type=float, default=0.015)
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)

    arm = LeftArm(args.config)
    arm.connect()
    arm.enable_all()

    base = arm.home_targets()
    joint_names = [spec.name for spec in arm.specs]
    header = build_header(joint_names)

    print("move to configured home")
    arm.settle(base, tol=args.settle_tol, timeout=15.0)

    t0 = time.time()

    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        try:
            for active_name in joint_names:
                print("collect:", active_name)
                start = time.time()
                last_targets = dict(base)
                last_offset = 0.0

                while time.time() - start < args.move_time:
                    elapsed = time.time() - start
                    targets, last_offset = arm.cosine_sine_target(base, active_name, args.amp, args.freq, elapsed)
                    last_targets = targets
                    arm.command_targets(targets, moving_name=active_name)
                    state = arm.state()
                    writer.writerow(row_from_state(t0, "move", active_name, joint_names, state, targets))
                    time.sleep(arm.period)

                return_time = 1.5
                return_start = time.time()
                while time.time() - return_start < return_time:
                    alpha = min(max((time.time() - return_start) / return_time, 0.0), 1.0)
                    smooth = 0.5 - 0.5 * __import__("math").cos(__import__("math").pi * alpha)
                    targets = dict(base)
                    targets[active_name] = base[active_name] + last_offset * (1.0 - smooth)
                    last_targets = targets
                    arm.command_targets(targets, moving_name=active_name)
                    state = arm.state()
                    writer.writerow(row_from_state(t0, "return", active_name, joint_names, state, targets))
                    time.sleep(arm.period)

                ok, _ = arm.settle(base, tol=args.settle_tol, timeout=12.0)
                state = arm.state()
                writer.writerow(row_from_state(t0, "settle_ok" if ok else "settle_timeout", active_name, joint_names, state, base))

            print("hold at home, Ctrl+C to stop")
            while True:
                arm.command_targets(base)
                state = arm.state()
                writer.writerow(row_from_state(t0, "hold", "all", joint_names, state, base))
                time.sleep(arm.period)

        except KeyboardInterrupt:
            print("stop requested")
        finally:
            arm.ramp_down_hold(base)
            print("saved", args.out)


if __name__ == "__main__":
    main()

