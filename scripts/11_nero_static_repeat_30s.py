import csv
import time
import numpy as np
from pathlib import Path

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    NeroFW,
)

DURATION = 30.0
WARMUP = 2.0

JOINT_FILE = Path(
    r"D:\robot_data_audit\nero\nero_joint_angles_static_30s_run02.csv"
)

MOTOR_FILE = Path(
    r"D:\robot_data_audit\nero\nero_motor_states_static_30s_run02.csv"
)

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.V121,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)

joint_rows = []
motor_rows = []

last_joint_ts = None

last_motor_ts = {
    i: None
    for i in range(1, 8)
}


def calc_stats(name, timestamps):

    ts = np.asarray(
        timestamps,
        dtype=float
    )

    if len(ts) < 2:

        print(
            f"{name:14s} N={len(ts):5d}  DATA INSUFFICIENT"
        )

        return

    dt = np.diff(ts)

    dt = dt[
        dt > 0
    ]

    if len(dt) == 0:

        print(
            f"{name:14s} no positive timestamp intervals"
        )

        return

    median_dt = float(
        np.median(dt)
    )

    mean_dt = float(
        np.mean(dt)
    )

    std_dt = float(
        np.std(dt)
    )

    hz_median = (
        1.0 / median_dt
    )

    duration = (
        ts[-1] - ts[0]
    )

    hz_average = (
        (len(ts) - 1)
        / duration
        if duration > 0
        else float("nan")
    )

    jitter = np.abs(
        dt - median_dt
    )

    p95 = float(
        np.percentile(
            jitter,
            95
        )
    )

    print(
        f"{name:14s} "
        f"N={len(ts):5d}  "
        f"medianHz={hz_median:9.3f}  "
        f"avgHz={hz_average:9.3f}  "
        f"medianT={median_dt*1000:8.4f} ms  "
        f"stdT={std_dt*1000:8.4f} ms  "
        f"P95jit={p95*1000:8.4f} ms"
    )


try:

    print("=" * 90)
    print("NERO STATIC RATE TEST")
    print("Firmware 1.21 / NeroFW.V121")
    print("READ ONLY - NO ENABLE - NO MOTION")
    print("=" * 90)

    robot.connect()

    print("Connected.")

    time.sleep(WARMUP)

    # ------------------------------------------------------
    # 先检查电机是否已经使能
    # ------------------------------------------------------

    print()
    print("Joint enable status:")

    for i in range(1, 8):

        try:

            state = robot.get_joint_enable_status(i)

            print(
                f"J{i}: {state}"
            )

        except Exception as e:

            print(
                f"J{i}: ERROR {e}"
            )


    print()
    print("Recording 10 seconds...")

    start = time.perf_counter()


    while (
        time.perf_counter()
        -
        start
        <
        DURATION
    ):

        # ==================================================
        # 7轴组合角
        # ==================================================

        ja = robot.get_joint_angles()

        if ja is not None:

            if (
                ja.timestamp
                !=
                last_joint_ts
            ):

                host_ns = (
                    time.perf_counter_ns()
                )

                q = list(
                    ja.msg
                )

                joint_rows.append([
                    host_ns,
                    ja.timestamp,
                    ja.hz,
                    *q
                ])

                last_joint_ts = (
                    ja.timestamp
                )


        # ==================================================
        # 7个motor state
        # ==================================================

        for j in range(1, 8):

            ms = robot.get_motor_states(j)

            if ms is None:
                continue

            if (
                ms.timestamp
                ==
                last_motor_ts[j]
            ):
                continue

            host_ns = (
                time.perf_counter_ns()
            )

            motor_rows.append([
                host_ns,
                j,
                ms.timestamp,
                ms.hz,
                ms.msg.position,
                ms.msg.velocity,
                ms.msg.current,
                ms.msg.torque,
            ])

            last_motor_ts[j] = (
                ms.timestamp
            )


        elapsed = (
            time.perf_counter()
            -
            start
        )

        print(
            f"\r{elapsed:5.1f}/{DURATION:.0f} s",
            end=""
        )

        # 仅是查询周期
        # 不是NERO采样周期
        time.sleep(0.001)


finally:

    robot.disconnect()


print()
print()
print("Saving CSV...")


# ----------------------------------------------------------
# 保存 joint angles
# ----------------------------------------------------------

with JOINT_FILE.open(
    "w",
    newline="",
    encoding="utf-8-sig"
) as f:

    w = csv.writer(f)

    w.writerow([
        "host_perf_ns",
        "sdk_timestamp_s",
        "sdk_reported_hz",
        "q1_rad",
        "q2_rad",
        "q3_rad",
        "q4_rad",
        "q5_rad",
        "q6_rad",
        "q7_rad",
    ])

    w.writerows(
        joint_rows
    )


# ----------------------------------------------------------
# 保存 motor states
# ----------------------------------------------------------

with MOTOR_FILE.open(
    "w",
    newline="",
    encoding="utf-8-sig"
) as f:

    w = csv.writer(f)

    w.writerow([
        "host_perf_ns",
        "joint",
        "sdk_timestamp_s",
        "sdk_reported_hz",
        "position_rad",
        "velocity_rad_s",
        "current_A",
        "torque_Nm",
    ])

    w.writerows(
        motor_rows
    )


# ----------------------------------------------------------
# 时间戳统计
# ----------------------------------------------------------

print()
print("=" * 110)
print("MEASURED UPDATE RATE FROM SDK TIMESTAMP")
print("=" * 110)

calc_stats(
    "JointAngles",
    [
        row[1]
        for row in joint_rows
    ]
)


for j in range(1, 8):

    ts = [
        row[2]
        for row in motor_rows
        if row[1] == j
    ]

    calc_stats(
        f"Motor J{j}",
        ts
    )


print()
print("=" * 110)

print(
    "Joint-angle unique samples:",
    len(joint_rows)
)

print(
    "Motor-state total samples:",
    len(motor_rows)
)

print()
print(
    "Saved:",
    JOINT_FILE
)

print(
    "Saved:",
    MOTOR_FILE
)
