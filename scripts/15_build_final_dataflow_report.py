from pathlib import Path

import numpy as np
import pandas as pd


ROOT = Path(r"D:\robot_data_audit")

D435I_SUMMARY = ROOT / "d435i" / "d435i_final_summary.csv"
RGBD_SUMMARY = ROOT / "d435i" / "rgb_depth_timing_summary.csv"

NERO_JOINT = ROOT / "nero" / "nero_joint_angles_static_10s.csv"
NERO_MOTOR = ROOT / "nero" / "nero_motor_states_static_10s.csv"

OUT = ROOT / "final"
OUT.mkdir(parents=True, exist_ok=True)


required = [
    D435I_SUMMARY,
    RGBD_SUMMARY,
    NERO_JOINT,
    NERO_MOTOR,
]

missing = [
    str(p)
    for p in required
    if not p.exists()
]

if missing:

    print("ERROR: Missing files:")

    for p in missing:
        print("  ", p)

    raise SystemExit(1)


# ============================================================
# 1. 读取 D435i
# ============================================================

d435 = pd.read_csv(D435I_SUMMARY)

rgbd = pd.read_csv(RGBD_SUMMARY)


# ============================================================
# 2. NERO JointAngles 分析
# ============================================================

nj = pd.read_csv(NERO_JOINT)

nj_ts = pd.to_numeric(
    nj["sdk_timestamp_s"],
    errors="coerce"
).dropna().to_numpy(dtype=float)

nj_dt = np.diff(nj_ts)

nj_dt = nj_dt[
    nj_dt > 0
]


if len(nj_dt):

    nero_joint_median_period_ms = (
        np.median(nj_dt) * 1000
    )

    nero_joint_median_hz = (
        1.0 / np.median(nj_dt)
    )

    nero_joint_avg_hz = (
        (len(nj_ts) - 1)
        /
        (nj_ts[-1] - nj_ts[0])
    )

    nero_joint_std_ms = (
        np.std(nj_dt) * 1000
    )

    nero_joint_p95_ms = (
        np.percentile(
            np.abs(
                nj_dt
                -
                np.median(nj_dt)
            ),
            95
        )
        *
        1000
    )

else:

    nero_joint_median_period_ms = np.nan
    nero_joint_median_hz = np.nan
    nero_joint_avg_hz = np.nan
    nero_joint_std_ms = np.nan
    nero_joint_p95_ms = np.nan


# ============================================================
# 3. NERO 每关节 Motor State
# ============================================================

nm = pd.read_csv(NERO_MOTOR)

nero_joint_results = []


for joint, g in nm.groupby("joint"):

    g = g.sort_values("host_perf_ns")

    ts = pd.to_numeric(
        g["sdk_timestamp_s"],
        errors="coerce"
    ).dropna().to_numpy(dtype=float)

    dt = np.diff(ts)

    dt = dt[
        dt > 0
    ]


    if len(dt):

        median_period = np.median(dt)

        median_hz = (
            1.0 / median_period
        )

        avg_hz = (
            (len(ts) - 1)
            /
            (ts[-1] - ts[0])
        )

        jitter = np.abs(
            dt - median_period
        )

        p95_jitter_ms = (
            np.percentile(
                jitter,
                95
            )
            *
            1000
        )

        std_period_ms = (
            np.std(dt)
            *
            1000
        )

    else:

        median_period = np.nan
        median_hz = np.nan
        avg_hz = np.nan
        p95_jitter_ms = np.nan
        std_period_ms = np.nan


    nero_joint_results.append({

        "joint":
            int(joint),

        "samples":
            len(g),

        "median_frequency_hz":
            median_hz,

        "average_frequency_hz":
            avg_hz,

        "median_period_ms":
            (
                median_period * 1000
                if np.isfinite(median_period)
                else np.nan
            ),

        "std_period_ms":
            std_period_ms,

        "p95_sdk_timestamp_jitter_ms":
            p95_jitter_ms,

        "position_min_rad":
            pd.to_numeric(
                g["position_rad"],
                errors="coerce"
            ).min(),

        "position_max_rad":
            pd.to_numeric(
                g["position_rad"],
                errors="coerce"
            ).max(),

        "velocity_min_rad_s":
            pd.to_numeric(
                g["velocity_rad_s"],
                errors="coerce"
            ).min(),

        "velocity_max_rad_s":
            pd.to_numeric(
                g["velocity_rad_s"],
                errors="coerce"
            ).max(),

        "current_min_A":
            pd.to_numeric(
                g["current_A"],
                errors="coerce"
            ).min(),

        "current_max_A":
            pd.to_numeric(
                g["current_A"],
                errors="coerce"
            ).max(),

        "torque_min_Nm":
            pd.to_numeric(
                g["torque_Nm"],
                errors="coerce"
            ).min(),

        "torque_max_Nm":
            pd.to_numeric(
                g["torque_Nm"],
                errors="coerce"
            ).max(),
    })


nero_rate = pd.DataFrame(
    nero_joint_results
)

nero_rate.to_csv(
    OUT / "nero_joint_rate_summary.csv",
    index=False,
    encoding="utf-8-sig"
)


# ============================================================
# 4. NERO跨关节 SDK timestamp 错峰
#
# 注意：
# 这里只能描述SDK层错峰，
# 不能解释为物理硬件采样延迟。
# ============================================================

series = {}

for joint in range(1, 8):

    g = nm[
        nm["joint"] == joint
    ].sort_values(
        "host_perf_ns"
    )

    series[joint] = (
        pd.to_numeric(
            g["sdk_timestamp_s"],
            errors="coerce"
        )
        .dropna()
        .to_numpy(dtype=float)
    )


n = min(
    len(series[j])
    for j in range(1, 8)
)

base = series[1][:n]

offset_rows = []


for joint in range(2, 8):

    x = series[joint][:n]

    diff_us = (
        x - base
    ) * 1e6

    offset_rows.append({

        "comparison":
            f"J{joint}-J1",

        "samples":
            len(diff_us),

        "median_offset_us":
            np.median(diff_us),

        "mean_offset_us":
            np.mean(diff_us),

        "std_offset_us":
            np.std(diff_us),

        "min_offset_us":
            np.min(diff_us),

        "max_offset_us":
            np.max(diff_us),

        "note":
            "SDK sequence-aligned offset; NOT hardware sampling delay",
    })


offset_df = pd.DataFrame(
    offset_rows
)

offset_df.to_csv(
    OUT / "nero_cross_joint_sdk_offset.csv",
    index=False,
    encoding="utf-8-sig"
)


# ============================================================
# 5. D435i统一清单
# ============================================================

inventory = []


for _, row in d435.iterrows():

    name = str(
        row["stream"]
    )

    if name in [
        "Depth",
        "RGB"
    ]:

        config = "640x480 @ 30 Hz"

    elif name in [
        "Gyro",
        "Accel"
    ]:

        config = "configured 200 Hz"

    else:

        config = ""


    inventory.append({

        "device":
            "Intel RealSense D435i",

        "stream":
            name,

        "configured_rate":
            config,

        "measured_average_rate_hz":
            row.get(
                "average_frequency_hz",
                np.nan
            ),

        "measured_median_rate_hz":
            row.get(
                "median_frequency_hz",
                np.nan
            ),

        "median_period_ms":
            row.get(
                "median_period_us",
                np.nan
            ) / 1000.0,

        "timestamp_type":
            row.get(
                "device_timestamp_source",
                ""
            ),

        "timestamp_semantics":
            "device-side RealSense metadata",

        "timing_stability":
            (
                f"P95 period jitter = "
                f"{row.get('p95_jitter_us', np.nan):.3f} us"
            ),

        "data_status":
            "VALID",

        "notes":
            (
                "Absolute physical timestamp accuracy "
                "has NOT been externally calibrated."
            ),
    })


# ============================================================
# 6. NERO统一清单
# ============================================================

inventory.append({

    "device":
        "AgileX NERO 7-axis",

    "stream":
        "7-axis joint-angle aggregate",

    "configured_rate":
        "not assumed",

    "measured_average_rate_hz":
        nero_joint_avg_hz,

    "measured_median_rate_hz":
        nero_joint_median_hz,

    "median_period_ms":
        nero_joint_median_period_ms,

    "timestamp_type":
        "pyAgxArm SDK timestamp",

    "timestamp_semantics":
        "SDK-recorded message time; NOT hardware sampling time",

    "timing_stability":
        (
            f"P95 SDK timestamp interval jitter = "
            f"{nero_joint_p95_ms:.3f} ms"
        ),

    "data_status":
        "VALID",

    "notes":
        (
            "Aggregate joint vector is built from asynchronously "
            "updated feedback and must not be assumed simultaneous."
        ),
})


inventory.append({

    "device":
        "AgileX NERO 7-axis",

    "stream":
        "per-joint motor state",

    "configured_rate":
        "not assumed",

    "measured_average_rate_hz":
        nero_rate[
            "average_frequency_hz"
        ].mean(),

    "measured_median_rate_hz":
        nero_rate[
            "median_frequency_hz"
        ].mean(),

    "median_period_ms":
        nero_rate[
            "median_period_ms"
        ].mean(),

    "timestamp_type":
        "pyAgxArm SDK timestamp",

    "timestamp_semantics":
        "SDK-recorded message time; NOT motor hardware sample time",

    "timing_stability":
        (
            f"mean per-joint P95 interval jitter = "
            f"{nero_rate['p95_sdk_timestamp_jitter_ms'].mean():.3f} ms"
        ),

    "data_status":
        "VALID",

    "notes":
        (
            "position available; velocity field available; "
            "current and torque validated after enable; "
            "dynamic velocity validation remains pending."
        ),
})


inventory_df = pd.DataFrame(
    inventory
)

inventory_df.to_csv(
    OUT / "dataflow_inventory.csv",
    index=False,
    encoding="utf-8-sig"
)


# ============================================================
# 7. 提取RGB-Depth结果
# ============================================================

def find_rgbd(text):

    x = rgbd[
        rgbd["metric"].astype(str).str.contains(
            text,
            case=False,
            na=False
        )
    ]

    if len(x) == 0:
        return None

    return x.iloc[0]


sdk_rgbd = find_rgbd(
    "SDK"
)

sensor_rgbd = find_rgbd(
    "SENSOR"
)

frame_rgbd = find_rgbd(
    "FRAME"
)


# ============================================================
# 8. Markdown最终报告
# ============================================================

def fmt(x, digits=3):

    try:

        if pd.isna(x):
            return "N/A"

        return f"{float(x):.{digits}f}"

    except Exception:
        return str(x)


depth = d435[
    d435["stream"] == "Depth"
].iloc[0]

rgb = d435[
    d435["stream"] == "RGB"
].iloc[0]

gyro = d435[
    d435["stream"] == "Gyro"
].iloc[0]

accel = d435[
    d435["stream"] == "Accel"
].iloc[0]


max_cross_joint = (
    offset_df[
        "max_offset_us"
    ].max()
)


report = f"""# D435i + NERO 数据流摸底最终报告 V1.0

## 1. 实验目的

本阶段不进行算法验证，只确认真实硬件系统中：

- 可以读取哪些数据；
- 各数据流的实际更新率；
- 时间戳来自哪里；
- 不同数据流是否真正同步；
- 哪些时间量只能视为软件接收时间；
- 后续 Zynq 硬件时间基真正需要解决什么问题。

---

## 2. D435i 实测结论

实验配置：

- RGB: 640x480 @ 30 Hz
- Depth: 640x480 @ 30 Hz
- Gyro: 200 Hz
- Accel: 200 Hz
- Windows 11 实机 USB 3.x
- 120 s 正式采样
- 启用 RealSense Windows metadata

### 实际采样率

| 数据流 | 长期平均频率 | 中位周期对应频率 | 中位周期 |
|---|---:|---:|---:|
| Depth | {fmt(depth['average_frequency_hz'])} Hz | {fmt(depth['median_frequency_hz'])} Hz | {fmt(depth['median_period_us']/1000)} ms |
| RGB | {fmt(rgb['average_frequency_hz'])} Hz | {fmt(rgb['median_frequency_hz'])} Hz | {fmt(rgb['median_period_us']/1000)} ms |
| Gyro | {fmt(gyro['average_frequency_hz'])} Hz | {fmt(gyro['median_frequency_hz'])} Hz | {fmt(gyro['median_period_us']/1000)} ms |
| Accel | {fmt(accel['average_frequency_hz'])} Hz | {fmt(accel['median_frequency_hz'])} Hz | {fmt(accel['median_period_us']/1000)} ms |

### 时间戳

- Depth/RGB：可以获得 SENSOR_TIMESTAMP 和 FRAME_TIMESTAMP。
- Gyro/Accel：可以获得设备 FRAME_TIMESTAMP。
- SDK global time 与设备 metadata 必须区分。
- timestamp 的微秒表示粒度不能等同于绝对物理时间精度。
- 当前尚未用 FPGA / 示波器对绝对采样时刻进行外部校准。

### RGB 与 Depth 相对时序

"""

if sensor_rgbd is not None:

    report += (
        f"- SENSOR timestamp median RGB-Depth offset: "
        f"{fmt(sensor_rgbd['median_us'])} us\n"
    )

if frame_rgbd is not None:

    report += (
        f"- FRAME timestamp median RGB-Depth offset: "
        f"{fmt(frame_rgbd['median_us'])} us\n"
    )

if sdk_rgbd is not None:

    report += (
        f"- SDK/global timestamp median RGB-Depth offset: "
        f"{fmt(sdk_rgbd['median_us'])} us\n"
    )


report += f"""

结论：

**RGB、Depth、Gyro、Accel 不是“同一时刻同步采样”的单速率系统。**
后续必须按各自真实 timestamp 建立统一时间轴。

---

## 3. NERO 实测结论

硬件/软件条件：

- Firmware: 1.21
- pyAgxArm driver: NeroFW.V121
- Interface: agx_cando
- Channel: 0

### 可读取数据

NERO SDK 已实测可读取：

- 7轴组合关节角；
- 单关节 position；
- velocity；
- current；
- torque；
- SDK timestamp；
- SDK reported hz；
- enable status；
- arm status；
- driver status；
- angle / velocity / acceleration limits。

### 更新率

7轴组合状态长期平均更新率：

**{fmt(nero_joint_avg_hz)} Hz**

每关节 motor-state 长期平均更新率均值：

**{fmt(nero_rate['average_frequency_hz'].mean())} Hz**

因此当前系统可按：

**约 200 Hz 的 NERO 状态反馈**

进行系统级设计。

### 时间戳性质

当前 pyAgxArm 的 timestamp 只能解释为：

**SDK-recorded message timestamp**

不能解释为：

**电机编码器/驱动器硬件真实采样时刻。**

7个关节 SDK timestamp 存在分批更新。

本次 sequence-index 对齐分析中，观测到最大 SDK timestamp 差约：

**{fmt(max_cross_joint/1000)} ms**

这只能证明 SDK 层存在毫秒级错峰，
不能直接解释成真实关节物理采样延迟。

### current / torque

未使能时：

- velocity ≈ 0
- current ≈ 0
- torque ≈ 0

使能静止后：

- current 出现 mA 级非零值；
- torque 出现对应的非零值。

因此 current / torque 字段已经验证为有效数据流。

torque 是基于电流和关节系数计算得到的关节力矩量，
不等价于末端六维 F/T 传感器。

### velocity

Firmware 1.21 提供 velocity 字段。

目前：

- 静止状态 velocity = 0；
- 动态运动状态尚未做最终验证。

因此当前状态标记为：

**字段存在、静态合理、动态待验证。**

---

## 4. 机械臂状态补充发现

当前反馈角：

- J2 ≈ -100.960 deg
- J4 ≈ 125.787 deg
- J7 ≈ 96.955 deg

超过当前软件命令范围：

- J2 min = -100 deg
- J4 max = 123 deg
- J7 max = 90 deg

但原厂状态窗口显示：

- collision: normal
- stall: normal
- communication: normal
- angle status: normal
- error status: normal

因此目前应区分：

**反馈实际位置**

与

**软件允许命令范围**

二者不是同一个概念。

在该问题完全确认前，不建议直接进行自动 move_j 动态测试。

---

## 5. 系统级最终结论

真实系统不是：

`所有数据天然同步 -> 统一送入算法`

而是：

`多设备 + 多采样率 + 多时间域 + 非同步更新`

具体表现为：

### D435i

- 视频约 30 Hz；
- Gyro约 200 Hz；
- Accel实际节拍与Gyro并不完全一致；
- RGB/Depth存在固定相位偏置；
- 有设备侧硬件 metadata。

### NERO

- 状态长期约 200 Hz；
- 7关节反馈并非严格同时更新；
- SDK timestamp 存在毫秒级分批到达特征；
- SDK 当前没有证明存在真正的关节硬件采样 timestamp。

---

## 6. 对 Zynq 研究方案的影响

原方案不应简单表述为：

**“利用10 ns时基使所有传感器实现10 ns同步。”**

更准确的技术目标应该是：

### 1. CAN 帧硬件打戳

利用 Zynq PL 100 MHz 时基，在 CAN SOF 附近记录总线事件时间，
绕开 Windows / USB-CAN / Python SDK 带来的软件接收时间不确定性。

### 2. 跨模态时钟映射

建立：

- D435i device time
- NERO CAN event time
- Zynq global time
- Windows host time

之间的映射关系。

### 3. 异步多率状态重建

将：

- RGB
- Depth
- Gyro
- Accel
- NERO J1-J7

映射到统一融合时刻，
而不是假设它们天然同时采样。

### 4. 延迟分解

区分：

- sensor sampling time
- CAN frame time
- USB arrival time
- SDK timestamp
- algorithm processing time

从而真正研究接触任务中的时序误差来源。

---

## 7. 当前数据流摸底状态

### 已完成

- [x] D435i RGB实际频率
- [x] D435i Depth实际频率
- [x] Gyro实际频率
- [x] Accel实际频率
- [x] D435i device metadata
- [x] RGB-Depth相对时序
- [x] NERO firmware
- [x] NERO position
- [x] NERO velocity字段
- [x] NERO current
- [x] NERO torque
- [x] NERO SDK timestamp
- [x] NERO真实更新率
- [x] NERO跨关节SDK错峰
- [x] NERO driver / communication状态

### 尚未完成

- [ ] NERO velocity动态运动验证
- [ ] Zynq CAN SOF硬件时间戳
- [ ] NERO CAN事件与SDK timestamp延迟比较
- [ ] D435i与Zynq绝对物理时间校准
- [ ] Revo 2 数据流摸底

---

## 8. 当前阶段结论

**D435i + NERO 数据流摸底 V1.0 完成。**

目前已经具备进入下一阶段：

**“Zynq统一时间轴设计与硬件时间戳验证”**

的基础条件。
"""


report_file = (
    OUT /
    "FINAL_DATAFLOW_AUDIT.md"
)

report_file.write_text(
    report,
    encoding="utf-8"
)


# ============================================================
# 9. 输出终端摘要
# ============================================================

print()
print("=" * 90)
print("DATA FLOW AUDIT COMPLETE")
print("=" * 90)

print()
print("D435i:")
print(
    f"Depth = {depth['average_frequency_hz']:.3f} Hz"
)
print(
    f"RGB   = {rgb['average_frequency_hz']:.3f} Hz"
)
print(
    f"Gyro  = {gyro['average_frequency_hz']:.3f} Hz"
)
print(
    f"Accel = {accel['average_frequency_hz']:.3f} Hz"
)

print()
print("NERO:")

print(
    f"JointAngles average = "
    f"{nero_joint_avg_hz:.3f} Hz"
)

print(
    f"Motor-state average = "
    f"{nero_rate['average_frequency_hz'].mean():.3f} Hz"
)

print(
    f"Max observed cross-joint SDK offset = "
    f"{max_cross_joint/1000:.3f} ms"
)

print()
print("Generated:")

for p in [
    OUT / "dataflow_inventory.csv",
    OUT / "nero_joint_rate_summary.csv",
    OUT / "nero_cross_joint_sdk_offset.csv",
    OUT / "FINAL_DATAFLOW_AUDIT.md",
]:

    print(" ", p)

print()
print("=" * 90)
