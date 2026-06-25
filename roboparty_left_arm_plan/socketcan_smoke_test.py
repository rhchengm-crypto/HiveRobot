import argparse
import os
import subprocess
import time


def run(cmd):
    print("+", " ".join(cmd))
    subprocess.check_call(cmd)


def main():
    parser = argparse.ArgumentParser(description="Bring up SocketCAN and send safe Damiao status reads.")
    parser.add_argument("--iface", default="can0")
    parser.add_argument("--bitrate", default="1000000")
    parser.add_argument("--ids", nargs="+", default=["0x1E", "0x1F", "0x20", "0x21", "0x22"])
    args = parser.parse_args()

    if os.geteuid() != 0:
        raise SystemExit("run with sudo")

    run(["modprobe", "can"])
    run(["modprobe", "can_raw"])
    run(["modprobe", "mttcan"])
    subprocess.call(["ip", "link", "set", args.iface, "down"])
    run(["ip", "link", "set", args.iface, "type", "can", "bitrate", args.bitrate, "restart-ms", "100"])
    run(["ip", "link", "set", args.iface, "up"])

    for motor_id in args.ids:
        slave = int(motor_id, 0)
        payload = f"{slave:02X}00330700000000"
        run(["cansend", args.iface, f"7FF#{payload}"])
        time.sleep(0.2)

    run(["ip", "-details", "-statistics", "link", "show", args.iface])


if __name__ == "__main__":
    main()

