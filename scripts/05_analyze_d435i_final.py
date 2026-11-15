import numpy as np
import pandas as pd
from pathlib import Path


INPUT = Path(
    r"D:\robot_data_audit\d435i\d435i_final_120s.csv"
)

OUTPUT = Path(
    r"D:\robot_data_audit\d435i\d435i_final_summary.csv"
)


df = pd.read_csv(INPUT)

results = []


def numeric_array(series):

    return (
        pd.to_numeric(
            series,
            errors="coerce"
        )
        .dropna()
        .to_numpy(dtype=float)
    )


def analyse(name, g):

    g = g.sort_values(
        "host_perf_ns"
    ).copy()


    # ==================================================
    # 选择设备侧时间戳
    #
    # 视频：优先使用 SENSOR_TIMESTAMP
    # IMU ：使用 FRAME_TIMESTAMP
    # ==================================================

    if name in ["Depth", "RGB"]:

        ts = numeric_array(
            g["sensor_timestamp_us"]
        )

        timestamp_source = (
            "SENSOR_TIMESTAMP"
        )

        if len(ts) < 2:

            ts = numeric_array(
                g["frame_timestamp_us"]
            )

            timestamp_source = (
                "FRAME_TIMESTAMP"
            )

    else:

        ts = numeric_array(
            g["frame_timestamp_us"]
        )

        timestamp_source = (
            "FRAME_TIMESTAMP"
        )


    # ==================================================
    # 设备时间间隔
    # ==================================================

    dt_us = np.diff(ts)

    # 记录异常
    nonpositive_count = int(
        np.sum(dt_us <= 0)
    )

    # 正常分析只使用正时间间隔
    dt_valid = dt_us[
        dt_us > 0
    ]


    if len(dt_valid) > 0:

        median_period_us = float(
            np.median(dt_valid)
        )

        mean_period_us = float(
            np.mean(dt_valid)
        )

        std_period_us = float(
            np.std(dt_valid)
        )

        median_frequency_hz = (
            1_000_000.0
            /
            median_period_us
        )

        # 根据首尾设备时间计算长期平均频率
        device_duration_s = (
            ts[-1] - ts[0]
        ) / 1_000_000.0

        average_frequency_hz = (
            (len(ts) - 1)
            /
            device_duration_s
            if device_duration_s > 0
            else np.nan
        )

        jitter_us = np.abs(
            dt_valid
            -
            median_period_us
        )

        p95_jitter_us = float(
            np.percentile(
                jitter_us,
                95
            )
        )

        p99_jitter_us = float(
            np.percentile(
                jitter_us,
                99
            )
        )

        min_period_us = float(
            np.min(dt_valid)
        )

        max_period_us = float(
            np.max(dt_valid)
        )

    else:

        median_period_us = np.nan
        mean_period_us = np.nan
        std_period_us = np.nan
        median_frequency_hz = np.nan
        average_frequency_hz = np.nan
        device_duration_s = np.nan
        p95_jitter_us = np.nan
        p99_jitter_us = np.nan
        min_period_us = np.nan
        max_period_us = np.nan


    # ==================================================
    # Host接收时间
    # ==================================================

    host_ns = numeric_array(
        g["host_perf_ns"]
    )

    host_dt_ms = (
        np.diff(host_ns)
        /
        1_000_000.0
    )

    host_dt_ms = host_dt_ms[
        host_dt_ms > 0
    ]

    host_median_period_ms = (
        float(
            np.median(host_dt_ms)
        )
        if len(host_dt_ms)
        else np.nan
    )


    # ==================================================
    # Frame Number跳号
    # ==================================================

    nums = numeric_array(
        g["frame_number"]
    )

    frame_number_gaps = 0

    if len(nums) > 1:

        dnum = np.diff(nums)

        frame_number_gaps = int(
            np.sum(
                np.maximum(
                    dnum - 1,
                    0
                )
            )
        )


    # ==================================================
    # metadata可用率
    # ==================================================

    sensor_count = (
        pd.to_numeric(
            g["sensor_timestamp_us"],
            errors="coerce"
        )
        .notna()
        .sum()
    )

    frame_count = (
        pd.to_numeric(
            g["frame_timestamp_us"],
            errors="coerce"
        )
        .notna()
        .sum()
    )


    domains = "|".join(
        sorted(
            g["timestamp_domain"]
            .astype(str)
            .unique()
        )
    )


    return {

        "stream":
            name,

        "samples":
            len(g),

        "device_timestamp_source":
            timestamp_source,

        "device_timestamp_samples":
            len(ts),

        "device_duration_s":
            device_duration_s,

        "median_frequency_hz":
            median_frequency_hz,

        "average_frequency_hz":
            average_frequency_hz,

        "median_period_us":
            median_period_us,

        "mean_period_us":
            mean_period_us,

        "std_period_us":
            std_period_us,

        "p95_jitter_us":
            p95_jitter_us,

        "p99_jitter_us":
            p99_jitter_us,

        "min_period_us":
            min_period_us,

        "max_period_us":
            max_period_us,

        "host_median_period_ms":
            host_median_period_ms,

        "frame_number_gaps":
            frame_number_gaps,

        "nonpositive_dt_count":
            nonpositive_count,

        "sensor_timestamp_count":
            int(sensor_count),

        "frame_timestamp_count":
            int(frame_count),

        "timestamp_domain":
            domains,
    }


for name in [
    "Depth",
    "RGB",
    "Gyro",
    "Accel"
]:

    g = df[
        df["stream"] == name
    ]

    if len(g):

        results.append(
            analyse(
                name,
                g
            )
        )


result = pd.DataFrame(
    results
)


pd.set_option(
    "display.max_columns",
    None
)

pd.set_option(
    "display.width",
    220
)


print()
print("=" * 220)

print(
    result.to_string(
        index=False
    )
)

print("=" * 220)


result.to_csv(
    OUTPUT,
    index=False,
    encoding="utf-8-sig"
)


print()
print(
    "Saved:",
    OUTPUT
)
