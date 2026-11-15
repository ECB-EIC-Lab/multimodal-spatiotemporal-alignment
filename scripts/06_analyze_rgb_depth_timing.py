import numpy as np
import pandas as pd
from pathlib import Path

INPUT = Path(
    r"D:\robot_data_audit\d435i\d435i_final_120s.csv"
)

OUTPUT = Path(
    r"D:\robot_data_audit\d435i\rgb_depth_timing_summary.csv"
)

PAIRS_OUTPUT = Path(
    r"D:\robot_data_audit\d435i\rgb_depth_pairs.csv"
)

df = pd.read_csv(INPUT)

depth = df[df["stream"] == "Depth"].copy()
rgb = df[df["stream"] == "RGB"].copy()

# 数值化
for g in [depth, rgb]:
    g["sdk_timestamp_ms"] = pd.to_numeric(
        g["sdk_timestamp_ms"],
        errors="coerce"
    )

depth = depth.dropna(
    subset=["sdk_timestamp_ms"]
).sort_values("sdk_timestamp_ms")

rgb = rgb.dropna(
    subset=["sdk_timestamp_ms"]
).sort_values("sdk_timestamp_ms")

# 加前缀，防止列名冲突
depth = depth.add_prefix("depth_")
rgb = rgb.add_prefix("rgb_")

# 以SDK global/system时间做最近邻配对
pairs = pd.merge_asof(
    depth,
    rgb,
    left_on="depth_sdk_timestamp_ms",
    right_on="rgb_sdk_timestamp_ms",
    direction="nearest",
    tolerance=20.0
)

pairs = pairs.dropna(
    subset=["rgb_sdk_timestamp_ms"]
).copy()

# SDK时间差，单位转成us
pairs["sdk_dt_us"] = (
    pairs["rgb_sdk_timestamp_ms"]
    -
    pairs["depth_sdk_timestamp_ms"]
) * 1000.0

# SENSOR timestamp
pairs["depth_sensor_us"] = pd.to_numeric(
    pairs["depth_sensor_timestamp_us"],
    errors="coerce"
)

pairs["rgb_sensor_us"] = pd.to_numeric(
    pairs["rgb_sensor_timestamp_us"],
    errors="coerce"
)

pairs["sensor_dt_us"] = (
    pairs["rgb_sensor_us"]
    -
    pairs["depth_sensor_us"]
)

# FRAME timestamp
pairs["depth_frame_us"] = pd.to_numeric(
    pairs["depth_frame_timestamp_us"],
    errors="coerce"
)

pairs["rgb_frame_us"] = pd.to_numeric(
    pairs["rgb_frame_timestamp_us"],
    errors="coerce"
)

pairs["frame_dt_us"] = (
    pairs["rgb_frame_us"]
    -
    pairs["depth_frame_us"]
)

def stats(name, series):

    x = pd.to_numeric(
        series,
        errors="coerce"
    ).dropna().to_numpy(dtype=float)

    if len(x) == 0:
        return {
            "metric": name,
            "N": 0
        }

    return {
        "metric": name,
        "N": len(x),
        "median_us": float(np.median(x)),
        "mean_us": float(np.mean(x)),
        "std_us": float(np.std(x)),
        "p95_abs_us": float(
            np.percentile(np.abs(x), 95)
        ),
        "p99_abs_us": float(
            np.percentile(np.abs(x), 99)
        ),
        "min_us": float(np.min(x)),
        "max_us": float(np.max(x)),
        "max_abs_us": float(
            np.max(np.abs(x))
        )
    }

summary = pd.DataFrame([
    stats(
        "RGB-Depth SDK time",
        pairs["sdk_dt_us"]
    ),
    stats(
        "RGB-Depth SENSOR timestamp",
        pairs["sensor_dt_us"]
    ),
    stats(
        "RGB-Depth FRAME timestamp",
        pairs["frame_dt_us"]
    )
])

pd.set_option("display.max_columns", None)
pd.set_option("display.width", 200)

print()
print("=" * 140)
print(summary.to_string(index=False))
print("=" * 140)

print()
print("Matched pairs:", len(pairs))

summary.to_csv(
    OUTPUT,
    index=False,
    encoding="utf-8-sig"
)

pairs[[
    "depth_frame_number",
    "rgb_frame_number",
    "depth_sdk_timestamp_ms",
    "rgb_sdk_timestamp_ms",
    "sdk_dt_us",
    "sensor_dt_us",
    "frame_dt_us"
]].to_csv(
    PAIRS_OUTPUT,
    index=False,
    encoding="utf-8-sig"
)

print()
print("Saved:", OUTPUT)
print("Saved:", PAIRS_OUTPUT)
