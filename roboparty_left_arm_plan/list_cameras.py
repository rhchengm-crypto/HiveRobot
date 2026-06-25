import argparse

import cv2


def main():
    parser = argparse.ArgumentParser(description="List OpenCV-readable camera indexes.")
    parser.add_argument("--max-index", type=int, default=8)
    args = parser.parse_args()

    for idx in range(args.max_index + 1):
        cap = cv2.VideoCapture(idx)
        ok = cap.isOpened()
        if ok:
            ret, frame = cap.read()
            shape = frame.shape if ret else None
            print(f"camera {idx}: opened, first_frame={ret}, shape={shape}")
        else:
            print(f"camera {idx}: not opened")
        cap.release()


if __name__ == "__main__":
    main()

