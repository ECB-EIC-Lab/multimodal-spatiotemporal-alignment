import csv
import time
import threading
from pathlib import Path
from collections import defaultdict

import pyrealsense2 as rs


# =========================================================
# 实验参数
# =========================================================

WARMUP_SECONDS = 3.0
DURATION_SECONDS = 120.0

OUTPUT = Path(
    r"D:\robot_data_audit\d435i\d435i_final_120s.csv"
)


# =========================================================
# 数据容器
# =========================================================

rows = []
lock = threading.Lock()

# 每一路最后保存的帧号，用于去除重复帧
last_frame_number = {}

recording_started = False


# =========================================================
# 工具函数
# =========================================================

def get_stream_name(frame):

    st = frame.get_profile().stream_type()

    if st == rs.stream.depth:
        return "Depth"

    elif st == rs.stream.color:
        return "RGB"

    elif st == rs.stream.gyro:
        return "Gyro"

    elif st == rs.stream.accel:
        return "Accel"

    return str(st)


def get_metadata(frame, item):

    try:

        if frame.supports_frame_metadata(item):

            return frame.get_frame_metadata(item)

    except Exception:
        pass

    return ""


def save_frame(frame):

    name = get_stream_name(frame)

    try:

        frame_number = int(
            frame.get_frame_number()
        )

    except Exception:

        frame_number = -1


    # =====================================================
    # 去重
    # =====================================================

    if last_frame_number.get(name) == frame_number:
        return

    last_frame_number[name] = frame_number


    # =====================================================
    # Windows主机时间
    # =====================================================

    host_perf_ns = time.perf_counter_ns()
    host_epoch_ns = time.time_ns()


    # =====================================================
    # SDK时间戳
    # =====================================================

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


    # =====================================================
    # RealSense metadata
    # =====================================================

    sensor_timestamp_us = get_metadata(
        frame,
        rs.frame_metadata_value.sensor_timestamp
    )

    frame_timestamp_us = get_metadata(
        frame,
        rs.frame_metadata_value.frame_timestamp
    )

    backend_timestamp_ms = get_metadata(
        frame,
        rs.frame_metadata_value.backend_timestamp
    )

    time_of_arrival_ms = get_metadata(
        frame,
        rs.frame_metadata_value.time_of_arrival
    )

    actual_fps_raw = get_metadata(
        frame,
        rs.frame_metadata_value.actual_fps
    )


    row = [

        name,

        frame_number,

        sdk_timestamp_ms,

        timestamp_domain,

        host_perf_ns,

        host_epoch_ns,

        sensor_timestamp_us,

        frame_timestamp_us,

        backend_timestamp_ms,

        time_of_arrival_ms,

        actual_fps_raw,
    ]


    with lock:

        rows.append(row)


# =========================================================
# Callback
# =========================================================

def callback(frame):

    global recording_started

    if not recording_started:
        return

    try:

        if frame.is_frameset():

            fs = frame.as_frameset()

            for f in fs:
                save_frame(f)

        else:

            save_frame(frame)

    except Exception as e:

        print(
            "Callback error:",
            e
        )


# =========================================================
# Pipeline配置
# =========================================================

pipeline = rs.pipeline()
config = rs.config()


# Depth 640x480 @ 30 Hz
config.enable_stream(
    rs.stream.depth,
    640,
    480,
    rs.format.z16,
    30
)


# RGB 640x480 @ 30 Hz
config.enable_stream(
    rs.stream.color,
    640,
    480,
    rs.format.bgr8,
    30
)


# Gyro 200 Hz
config.enable_stream(
    rs.stream.gyro,
    rs.format.motion_xyz32f,
    200
)


# Accel 200 Hz
config.enable_stream(
    rs.stream.accel,
    rs.format.motion_xyz32f,
    200
)


print("=" * 70)
print("D435i FINAL DATA FLOW TEST")
print("=" * 70)

print("Depth : 640x480 @ 30 Hz")
print("RGB   : 640x480 @ 30 Hz")
print("Gyro  : 200 Hz")
print("Accel : 200 Hz")

print()
print("Warm-up :", WARMUP_SECONDS, "s")
print("Record  :", DURATION_SECONDS, "s")

print("=" * 70)


# =========================================================
# 启动相机
# =========================================================

pipeline.start(
    config,
    callback
)


# =========================================================
# 预热
# =========================================================

print()
print("Warm-up started...")

time.sleep(
    WARMUP_SECONDS
)


# 清空预热期间状态
last_frame_number.clear()

recording_started = True

print()
print("Recording started...")


# =========================================================
# 正式采集
# =========================================================

start = time.perf_counter()


try:

    while True:

        elapsed = (
            time.perf_counter()
            -
            start
        )

        if elapsed >= DURATION_SECONDS:
            break

        print(
            f"\rRecording: "
            f"{elapsed:6.1f}/"
            f"{DURATION_SECONDS:.0f} s",
            end=""
        )

        time.sleep(
            0.5
        )


finally:

    recording_started = False

    pipeline.stop()


print()
print()
print("Saving CSV...")


# =========================================================
# 保存CSV
# =========================================================

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

        "sensor_timestamp_us",

        "frame_timestamp_us",

        "backend_timestamp_ms",

        "time_of_arrival_ms",

        "actual_fps_raw",
    ])

    writer.writerows(
        rows
    )


print(
    "Saved:",
    OUTPUT
)

print(
    "Total rows:",
    len(rows)
)


# =========================================================
# 帧数统计
# =========================================================

counts = defaultdict(int)

for row in rows:

    counts[
        row[0]
    ] += 1


print()
print("=" * 70)
print("COUNTS")
print("=" * 70)


for name in [
    "Depth",
    "RGB",
    "Gyro",
    "Accel"
]:

    print(
        f"{name:6s}:",
        counts[name]
    )


print("=" * 70)
print("Finished")
