
import time
import math

from pyAgxArm import (
    create_agx_arm_config,
    AgxArmFactory,
    ArmModel,
    NeroFW,
)

# 沿用已经成功运行的 NERO 配置
cfg = create_agx_arm_config(
    robot=ArmModel.NERO,
    firmware_version=NeroFW.V121,
    interface="agx_cando",
    channel="0",
)

robot = AgxArmFactory.create_arm(cfg)

try:
    print("Connecting to NERO...")
    robot.connect()
    time.sleep(1.0)

    print("\n========== CONNECTION ==========")
    print("Connected:", robot.is_connected())
    print("Communication error:", robot.has_comm_error())
    print("Error detail:", robot.get_comm_error())

    # 读取机械臂总体状态
    status = robot.get_arm_status()

    print("\n========== ARM STATUS ==========")
    if status is not None:
        print(status.msg)
    else:
        print("No arm status feedback!")

    # 读取七轴角度
    angles = robot.get_joint_angles()

    print("\n========== JOINT ANGLES ==========")
    if angles is not None:
        for i, q in enumerate(angles.msg, 1):
            print(
                f"J{i}: {q:.6f} rad "
                f"({math.degrees(q):.3f} deg)"
            )
    else:
        print("No joint angle feedback!")

    # 读取各关节使能状态
    print("\n========== ENABLE STATUS ==========")
    print(robot.get_joints_enable_status_list())

    # 读取末端法兰位姿
    flange = robot.get_flange_pose()

    print("\n========== FLANGE POSE ==========")
    if flange is not None:
        print(flange.msg)
    else:
        print("No flange pose feedback!")

    # 读取 J7 实际角度和速度限制
    limit = robot.get_joint_angle_vel_limits(7)

    print("\n========== J7 LIMIT ==========")
    if limit is not None:
        print("Min angle:", limit.msg.min_angle_limit)
        print("Max angle:", limit.msg.max_angle_limit)
        print("Max speed:", limit.msg.max_joint_spd)
    else:
        print("No J7 limit feedback!")

    print("\nPrecheck completed. NO MOTION COMMAND SENT.")

finally:
    robot.disconnect()
    print("\nCommunication disconnected.")
