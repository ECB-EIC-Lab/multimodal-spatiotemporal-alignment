# D435i + NERO 数据流摸底最终报告 V1.0

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
| Depth | 29.976 Hz | 29.976 Hz | 33.360 ms |
| RGB | 29.976 Hz | 29.976 Hz | 33.360 ms |
| Gyro | 199.894 Hz | 199.920 Hz | 5.002 ms |
| Accel | 202.553 Hz | 202.511 Hz | 4.938 ms |

### 时间戳

- Depth/RGB：可以获得 SENSOR_TIMESTAMP 和 FRAME_TIMESTAMP。
- Gyro/Accel：可以获得设备 FRAME_TIMESTAMP。
- SDK global time 与设备 metadata 必须区分。
- timestamp 的微秒表示粒度不能等同于绝对物理时间精度。
- 当前尚未用 FPGA / 示波器对绝对采样时刻进行外部校准。

### RGB 与 Depth 相对时序

- SENSOR timestamp median RGB-Depth offset: 591.000 us
- FRAME timestamp median RGB-Depth offset: 18.000 us
- SDK/global timestamp median RGB-Depth offset: 18.066 us


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

**200.026 Hz**

每关节 motor-state 长期平均更新率均值：

**200.014 Hz**

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

**5.033 ms**

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
