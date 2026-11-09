"""Read-only, bounded RGB-D recording; never imports a motor controller."""
import argparse
import json
from pathlib import Path
import signal
import threading
import time
import urllib.request


def image_array(message, np):
    formats = {'rgb8': ('u1', 3), 'bgr8': ('u1', 3), '16UC1': ('<u2', 1)}
    if message.encoding not in formats or message.is_bigendian:
        raise ValueError('unsupported camera image encoding: '+message.encoding)
    dtype, channels = formats[message.encoding]
    size = np.dtype(dtype).itemsize
    row = np.frombuffer(message.data, dtype=dtype).reshape(message.height, message.step//size)
    return row[:, :message.width*channels].reshape(message.height, message.width, channels).squeeze().copy()


def latest_stage(run):
    for line in reversed(run.get('stdout', '').splitlines()):
        if line.startswith('board tracking audit= '):
            try:
                item = json.loads(line.split('= ', 1)[1])
                return {k: item.get(k) for k in ('stage', 'waypoint_index', 'reference_rad', 'feedback',
                    'feedback_read_started_wall_s', 'feedback_read_finished_wall_s',
                    'feedback_sample_monotonic_s')}
            except ValueError:
                pass
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=300.)
    parser.add_argument('--fps', type=float, default=2.)
    args = parser.parse_args()
    if not 0 < args.seconds <= 600 or not .2 <= args.fps <= 4:
        parser.error('recording is limited to 600 seconds and 0.2..4 fps')
    args.output.mkdir(parents=True, exist_ok=False)
    import cv2
    import numpy as np
    import rospy
    from sensor_msgs.msg import Image, CameraInfo
    rospy.init_node('hive_readonly_rgbd_audit', anonymous=True, disable_signals=True)
    latest = {}
    lock = threading.Lock()
    stopped = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    signal.signal(signal.SIGINT, lambda *_: stopped.set())

    def remember(name):
        def callback(message):
            with lock:
                latest[name] = (message, time.time())
        return callback

    subscribers = [rospy.Subscriber('/ascamera_hp60c/'+suffix, kind, remember(name), queue_size=1)
        for name, suffix, kind in [('rgb', 'rgb0/image', Image),
                                  ('depth', 'depth0/image_raw', Image),
                                  ('info', 'rgb0/camera_info', CameraInfo)]]
    client = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    started = time.monotonic()
    index = 0
    run = {}
    last_poll = 0.
    with (args.output/'frames.jsonl').open('w') as stream:
        while not stopped.is_set() and time.monotonic()-started < args.seconds:
            tick = time.monotonic()
            with lock:
                current = dict(latest)
            if not all(k in current for k in ('rgb', 'depth', 'info')):
                stopped.wait(.05)
                continue
            now = time.time()
            rgb, rgb_received = current['rgb']
            depth, depth_received = current['depth']
            info, _ = current['info']
            # Never silently pair stale frames or repeat a frozen stream.
            if max(now-rgb_received, now-depth_received) > .3:
                stopped.wait(.1)
                continue
            rgb_stamp, depth_stamp = rgb.header.stamp.to_sec(), depth.header.stamp.to_sec()
            if abs(rgb_stamp-depth_stamp) > .06:
                stopped.wait(.01)
                continue
            if tick-last_poll > 1.:
                try:
                    with client.open('http://127.0.0.1:8100/api/v3/dynamics/status', timeout=2) as response:
                        run = json.load(response).get('run') or {}
                except Exception as error:
                    run = {'recording_status_error': str(error)}
                last_poll = tick
            if not (args.output/'camera.json').exists():
                (args.output/'camera.json').write_text(json.dumps({
                    'width': info.width, 'height': info.height, 'K': list(info.K),
                    'D': list(info.D), 'frame': info.header.frame_id,
                    'depth_encoding': depth.encoding, 'depth_frame': depth.header.frame_id,
                    'motion_commands_sent': False, 'depth_scale_independently_verified': False}), encoding='utf8')
            color = image_array(rgb, np)
            if rgb.encoding == 'rgb8':
                color = color[:, :, ::-1]
            name = '%06d' % index
            if not cv2.imwrite(str(args.output/(name+'.jpg')), color, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                raise RuntimeError('camera image write failed')
            np.save(str(args.output/(name+'.npy')), image_array(depth, np), allow_pickle=False)
            stream.write(json.dumps({'index': index, 'wall_time': now,
                'elapsed_s': tick-started, 'rgb_stamp': rgb_stamp, 'depth_stamp': depth_stamp,
                'action': run.get('action'), 'running': run.get('running'),
                'full_log_path': run.get('full_log_path'), 'tracking': latest_stage(run)})+'\n')
            stream.flush()
            index += 1
            stopped.wait(max(0, 1./args.fps-(time.monotonic()-tick)))
    for subscriber in subscribers:
        subscriber.unregister()
    print(json.dumps({'ok': True, 'frames': index, 'output': str(args.output),
                      'motion_commands_sent': False}), flush=True)


if __name__ == '__main__':
    main()
