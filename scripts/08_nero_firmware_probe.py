import time

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    NeroFW,
)

print("=" * 70)
print("NERO FIRMWARE PROBE")
print("No motion command will be sent.")
print("=" * 70)

cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmeware_version=NeroFW.DEFAULT,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)

try:
    print("Connecting...")
    robot.connect()
    print("Connected.")

    print()
    print("Step 1: wait 3 s for firmware WITHOUT enable")

    firmware = None

    deadline = time.time() + 3.0

    while time.time() < deadline:

        firmware = robot.get_firmware()

        if firmware is not None:
            break

        time.sleep(0.1)

    if firmware is not None:

        print()
        print("Firmware received without enable:")
        print(firmware)

    else:

        print()
        print("No firmware yet.")
        print("Step 2: sending ENABLE only.")
        print("This does NOT send a motion command.")

        # 官方detect流程也是通过enable等待机器人建立反馈
        deadline = time.time() + 8.0

        while time.time() < deadline:

            robot.enable()

            firmware = robot.get_firmware()

            if firmware is not None:
                break

            time.sleep(0.1)

        if firmware is None:

            print()
            print("RESULT: FAILED")
            print("Firmware was not received.")

        else:

            print()
            print("RESULT: SUCCESS")
            print("Firmware:")
            print(firmware)

finally:

    try:
        robot.disconnect()
    except Exception:
        pass

    print()
    print("Disconnected.")
