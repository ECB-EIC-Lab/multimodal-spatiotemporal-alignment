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

    print("=" * 80)
    print("NERO PRE-MOTION READ-ONLY CHECK")
    print("NO ENABLE / NO MOTION COMMAND")
    print("=" * 80)

    robot.connect()
    print("Connected.")

    time.sleep(2)

    print()
    print("=" * 80)
    print("ARM STATUS")
    print("=" * 80)

    try:
        print(robot.get_arm_status())
    except Exception as e:
        print("ERROR:", repr(e))

    print()
    print("=" * 80)
    print("CURRENT JOINT ANGLES")
    print("=" * 80)

    try:
        print(robot.get_joint_angles())
    except Exception as e:
        print("ERROR:", repr(e))

    print()
    print("=" * 80)
    print("JOINT ANGLE / VELOCITY LIMITS")
    print("=" * 80)

    for j in range(1, 8):

        print()
        print("JOINT", j)

        try:
            print(
                robot.get_joint_angle_vel_limits(j)
            )
        except Exception as e:
            print("ERROR:", repr(e))

    print()
    print("=" * 80)
    print("JOINT ACCELERATION LIMITS")
    print("=" * 80)

    for j in range(1, 8):

        print()
        print("JOINT", j)

        try:
            print(
                robot.get_joint_acc_limits(j)
            )
        except Exception as e:
            print("ERROR:", repr(e))

    print()
    print("=" * 80)
    print("DRIVER STATES")
    print("=" * 80)

    for j in range(1, 8):

        print()
        print("JOINT", j)

        try:
            print(
                robot.get_driver_states(j)
            )
        except Exception as e:
            print("ERROR:", repr(e))

finally:

    try:
        robot.disconnect()
    except Exception:
        pass

    print()
    print("Disconnected.")
