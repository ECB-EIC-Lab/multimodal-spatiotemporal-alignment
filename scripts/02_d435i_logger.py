import csv
import time
import threading
from pathlib import Path

import pyrealsense2 as rs


# 第一次先10秒
# 成功以后改为120
DURATION = 10.0

OUTPUT = Path(
    r"D:\robot_data_audit\d435i\d435i_10s.csv"
)

rows = []
lock = threading.Lock()


def get_metadata(frame, item):
    try:
        if frame.supports_frame_metadata(item):
            return frame.get_frame_metadata(item)
    except Exception:
        pass

    return ""


def stream_name(frame):

    st = frame.get_profile().stream_type()

    if st == rs.stream.depth:
        return "Depth"

    if st == rs.stream.color:
        return "RGB"

    if st == rs.stream.gyro:
        return "Gyro"

    if st == rs.stream.accel:
        return "Accel"

    return str(st)


def save_one_frame(frame):

    host_perf_ns = time.perf_counter_ns()
    host_epoch_ns = time.time_ns()

    try:
        sdk_timestamp_ms = frame.get_timestamp()
    except Exception:
        sdk_timestamp_ms = ""

    try:
        timestamp_domain = str(
            frame.get_frame_timestamp_domain()
        )
    except Exception:
        timestamp_domain = ""

    try:
        frame_number = frame.get_frame_number()
    except Exception:
        frame_number = ""

    row = [
        stream_name(frame),
        frame_number,
        sdk_timestamp_ms,
        timestamp_domain,
        host_perf_ns,
        host_epoch_ns,

        get_metadata(
            frame,
            rs.frame_metadata_value.sensor_timestamp
        ),

        get_metadata(
            frame,
            rs.frame_metadata_value.frame_timestamp
        ),

        get_metadata(
            frame,
            rs.frame_metadata_value.backend_timestamp
        ),

        get_metadata(
            frame,
            rs.frame_metadata_value.time_of_arrival
        ),

        get_metadata(
            frame,
            rs.frame_metadata_value.actual_fps
        ),
    ]

    with lock:
        rows.append(row)


def callback(frame):

    try:

        if frame.is_frameset():

            frameset = frame.as_frameset()

            for f in frameset:
                save_one_frame(f)

        else:
            save_one_frame(frame)

    except Exception as e:
        print("Callback error:", e)


pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(
    rs.stream.depth,
    640,
    480,
    rs.format.z16,
    30
)

config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)

config.enable_stream(
    rs.stream.gyro,
    rs.format.motion_xyz32f,
    200
)

config.enable_stream(
    rs.stream.accel,
    rs.format.motion_xyz32f,
    200
)


# 尽可能关闭global time，
# 保留设备自身的timestamp domain
ctx = rs.context()

devices = ctx.query_devices()

if len(devices) == 0:
    raise RuntimeError("No RealSense detected")

for sensor in devices[0].query_sensors():

    try:

        if sensor.supports(
            rs.option.global_time_enabled
        ):

            sensor.set_option(
                rs.option.global_time_enabled,
                0
            )

            print(
                "GLOBAL_TIME_ENABLED = 0:",
                sensor.get_info(
                    rs.camera_info.name
                )
            )

    except Exception:
        pass


print("=" * 60)
print("D435i data logger")
print("Depth : 640x480 @ 30 Hz")
print("RGB   : 640x480 @ 30 Hz")
print("Gyro  : 200 Hz")
print("Accel : 200 Hz")
print("Duration:", DURATION, "s")
print("=" * 60)


pipeline.start(
    config,
    callback
)

start = time.perf_counter()


try:

    while True:

        elapsed = (
            time.perf_counter()
            -
            start
        )

        if elapsed >= DURATION:
            break

        print(
            f"\rRunning: "
            f"{elapsed:6.1f}/{DURATION:.0f} s",
            end=""
        )

        time.sleep(0.5)

finally:

    pipeline.stop()


print()
print("Saving CSV...")


with OUTPUT.open(
    "w",
    newline="",
    encoding="utf-8-sig"
) as f:

    writer = csv.writer(f)

    writer.writerow([
        "stream",
        "frame_number",
        "sdk_timestamp_ms",
        "timestamp_domain",
        "host_perf_ns",
        "host_epoch_ns",
        "sensor_timestamp_raw",
        "frame_timestamp_raw",
        "backend_timestamp_raw",
        "time_of_arrival_raw",
        "actual_fps_raw",
    ])

    writer.writerows(rows)


print("Saved:", OUTPUT)
print("Rows:", len(rows))
