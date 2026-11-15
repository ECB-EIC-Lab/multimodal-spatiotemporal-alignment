import time
import math

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    NeroFW,
)

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.V121,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)

RAD2DEG = 180.0 / math.pi

try:

    print("=" * 90)
    print("NERO JOINT LIMIT CONSISTENCY CHECK")
    print("READ ONLY - NO ENABLE - NO MOTION")
    print("=" * 90)

    robot.connect()

    time.sleep(2.0)

    limits = {}

    print()
    print("Reading firmware limits...")

    for j in range(1, 8):

        r = robot.get_joint_angle_vel_limits(j)

        limits[j] = (
            r.msg.min_angle_limit,
            r.msg.max_angle_limit,
            r.msg.max_joint_spd,
        )

        print(
            f"J{j}: "
            f"min={limits[j][0]: .6f} rad, "
            f"max={limits[j][1]: .6f} rad, "
            f"vmax={limits[j][2]: .6f} rad/s"
        )

    print()
    print("=" * 90)
    print("Monitoring current angles for 5 seconds")
    print("=" * 90)

    worst_low = {j: 0.0 for j in range(1, 8)}
    worst_high = {j: 0.0 for j in range(1, 8)}

    end = time.perf_counter() + 5.0

    sample = 0

    while time.perf_counter() < end:

        ja = robot.get_joint_angles()

        if ja is None:
            time.sleep(0.02)
            continue

        q = list(ja.msg)

        sample += 1

        if sample == 1:

            print()
            print("Current angle snapshot:")

        for j in range(1, 8):

            angle = q[j - 1]

            mn, mx, vmax = limits[j]

            low_margin = angle - mn
            high_margin = mx - angle

            if angle < mn:

                over = mn - angle

                worst_low[j] = max(
                    worst_low[j],
                    over
                )

            if angle > mx:

                over = angle - mx

                worst_high[j] = max(
                    worst_high[j],
                    over
                )

            if sample == 1:

                state = "OK"

                if angle < mn:
                    state = "BELOW LIMIT"

                elif angle > mx:
                    state = "ABOVE LIMIT"

                print(
                    f"J{j}: "
                    f"q={angle: .6f} rad "
                    f"({angle*RAD2DEG: .3f} deg), "
                    f"range=[{mn: .6f}, {mx: .6f}], "
                    f"{state}"
                )

        time.sleep(0.02)

    print()
    print("=" * 90)
    print("5 SECOND CONSISTENCY RESULT")
    print("=" * 90)

    for j in range(1, 8):

        low_deg = (
            worst_low[j] * RAD2DEG
        )

        high_deg = (
            worst_high[j] * RAD2DEG
        )

        if (
            worst_low[j] == 0.0
            and
            worst_high[j] == 0.0
        ):

            print(
                f"J{j}: always inside reported limits"
            )

        else:

            print(
                f"J{j}: OUTSIDE reported limits, "
                f"max below={low_deg:.3f} deg, "
                f"max above={high_deg:.3f} deg"
            )

    print()
    print("Arm status:")

    print(
        robot.get_arm_status()
    )

finally:

    try:
        robot.disconnect()
    except Exception:
        pass

    print()
    print("Disconnected.")
