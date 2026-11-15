import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    NeroFW,
)

print("=" * 70)
print("NERO READ-ONLY STATE TEST")
print("Firmware: 1.21")
print("Driver: NeroFW.V121")
print("NO MOTION COMMAND")
print("=" * 70)

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.V121,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)

try:

    print()
    print("Connecting...")

    robot.connect()

    print("Connected.")

    # 先等反馈稳定
    time.sleep(2.0)

    print()
    print("=" * 70)
    print("TEST 1: get_joint_angles()")
    print("=" * 70)

    for i in range(10):

        try:

            ja = robot.get_joint_angles()

            print()
            print("Read", i + 1)

            if ja is None:

                print("joint_angles = None")

            else:

                print("msg       =", ja.msg)
                print("timestamp =", ja.timestamp)
                print("hz        =", ja.hz)

        except Exception as e:

            print(
                "get_joint_angles ERROR:",
                repr(e)
            )

        time.sleep(0.1)


    print()
    print("=" * 70)
    print("TEST 2: get_motor_states(1..7)")
    print("=" * 70)


    for joint in range(1, 8):

        print()
        print("-" * 60)
        print("JOINT", joint)
        print("-" * 60)

        try:

            ms = robot.get_motor_states(joint)

            if ms is None:

                print("motor_state = None")

            else:

                print(
                    "position  =",
                    ms.msg.position
                )

                print(
                    "velocity  =",
                    ms.msg.velocity
                )

                print(
                    "current   =",
                    ms.msg.current
                )

                print(
                    "torque    =",
                    ms.msg.torque
                )

                print(
                    "timestamp =",
                    ms.timestamp
                )

                print(
                    "hz        =",
                    ms.hz
                )

        except Exception as e:

            print(
                "get_motor_states ERROR:",
                repr(e)
            )


finally:

    print()
    print("Disconnecting...")

    try:
        robot.disconnect()
    except Exception:
        pass

    print("Disconnected.")
