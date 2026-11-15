import pandas as pd
import numpy as np

FILE = r"D:\robot_data_audit\nero\nero_motor_states_static_10s.csv"

df = pd.read_csv(FILE)

# 每个关节自己的timestamp序列
series = {}

for j in range(1, 8):

    g = df[df["joint"] == j].copy()

    g = g.sort_values("host_perf_ns")

    series[j] = g["sdk_timestamp_s"].to_numpy(dtype=float)

print("=" * 80)
print("CROSS-JOINT SDK TIMESTAMP CHECK")
print("=" * 80)

n = min(
    len(series[j])
    for j in range(1, 8)
)

print("Comparable cycles:", n)
print()

base = series[1][:n]

for j in range(2, 8):

    x = series[j][:n]

    diff_us = (
        x - base
    ) * 1e6

    print(
        f"J{j} - J1:",
        f"median = {np.median(diff_us):.3f} us,",
        f"mean = {np.mean(diff_us):.3f} us,",
        f"std = {np.std(diff_us):.3f} us,",
        f"min = {np.min(diff_us):.3f} us,",
        f"max = {np.max(diff_us):.3f} us"
    )

print()
print("First 10 timestamp sets:")

for k in range(min(10, n)):

    values = [
        series[j][k]
        for j in range(1, 8)
    ]

    print(
        k + 1,
        values
    )
