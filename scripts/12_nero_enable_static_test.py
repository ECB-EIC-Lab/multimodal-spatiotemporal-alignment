import time
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

try:
    print("=" * 70)
    print("NERO ENABLED STATIC TEST")
    print("ENABLE ONLY - NO MOTION COMMAND")
    print("=" * 70)

    robot.connect()
    print("Connected.")

    print("\nEnabling...")

    t0 = time.perf_counter()
    enabled = False

    while time.perf_counter() - t0 < 5.0:
        enabled = bool(robot.enable())

        if enabled:
            break

        time.sleep(0.05)

    print("enable() =", enabled)

    time.sleep(2.0)

    print("\nJoint enable status:")

    for j in range(1, 8):
        print(
            f"J{j}:",
            robot.get_joint_enable_status(j)
        )

    print("\nMotor states after enable:")

    for j in range(1, 8):

        ms = robot.get_motor_states(j)

        print("\nJOINT", j)

        if ms is None:
            print("None")
            continue

        print("position  =", ms.msg.position)
        print("velocity  =", ms.msg.velocity)
        print("current   =", ms.msg.current)
        print("torque    =", ms.msg.torque)
        print("timestamp =", ms.timestamp)
        print("hz        =", ms.hz)

    print("\nHolding enabled state for 10 s...")

    for i in range(10):
        time.sleep(1)

        vals = []

        for j in range(1, 8):

            ms = robot.get_motor_states(j)

            if ms is not None:
                vals.append(
                    (
                        j,
                        ms.msg.current,
                        ms.msg.torque,
                        ms.msg.velocity
                    )
                )

        print(f"\nSecond {i+1}")

        for j, current, torque, velocity in vals:
            print(
                f"J{j}: "
                f"velocity={velocity:.6f}, "
                f"current={current:.6f}, "
                f"torque={torque:.6f}"
            )

finally:

    try:
        robot.disable()
        print("\nRobot disabled.")
    except Exception as e:
        print("\ndisable() warning:", e)

    try:
        robot.disconnect()
    except Exception:
        pass

    print("Disconnected.")
