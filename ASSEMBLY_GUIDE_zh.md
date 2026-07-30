# Zeroth-01 Round v3 装配指南

## 前提

本包保留 canonical Zeroth-01 17-link / 16DoF 承载机构。白色外壳、屏幕头、脚底和蓝色舵机审图件不能替代缺失的承载支架、轴承、紧固件或线束设计。

SolidWorks 使用两个入口：

- `ZEROTH01_ROUND_V3_WHITE_EXTERIOR.SLDASM`：检查正常白色外观。
- `OPEN_FIRST_ZEROTH01_ROUND_V3_WHITE_EVA_16_BLUE_SERVOS_XRAY.SLDASM`：检查 16 个蓝色舵机位置，特别是两侧肩部各两个关节。

`ZEROTH01_STS3250_C001_BLUE_DIAGNOSTIC.SLDPRT` 只用于审图；不要打印后夹入原机构。

## 装配层级

1. 承载层：原 Zeroth-01 骨架、原关节支架/输出接口、轴承和紧固件。
2. 电子层：电池+BMS、主控/稳压托盘、IMU、Waveshare 4.3 屏、相机、ToF、线束和足底压力传感器。
3. 外观层：前后胸壳、前后骨盆壳、带小耳朵的前后蛋形头壳、黑色屏幕面板、左右上臂套、左右前臂套、左右固定 Q 版手掌和左右加厚脚底。

顺序必须是承载层单独扫掠 → 电子层和线束扫掠 → 外观逐片试装；外壳不得承担关节力矩。

## S01–S16

| ID | canonical joint |
|---|---|
| S01 | right_shoulder_pitch |
| S02 | left_shoulder_pitch |
| S03 | right_shoulder_yaw |
| S04 | right_hip_pitch |
| S05 | left_hip_pitch |
| S06 | left_shoulder_yaw |
| S07 | right_hip_yaw |
| S08 | left_hip_yaw |
| S09 | right_elbow_yaw |
| S10 | left_elbow_yaw |
| S11 | right_hip_roll |
| S12 | left_hip_roll |
| S13 | right_knee_pitch |
| S14 | left_knee_pitch |
| S15 | right_ankle_pitch |
| S16 | left_ankle_pitch |

16 个 SolidWorks 实例全部为蓝色。精确轴线、原点和守护限位见 `reports/joint_servo_frames.csv`；总线 ID、零位 count 和方向必须实机点动标定。

## 电子件安装区

坐标为 Torso 坐标系：前方 `-Y`，上方 `+Z`，机器人左侧 `+X`。

| 部件 | 中心位置 m | 包络 m | 状态 |
|---|---|---|---|
| Waveshare 4.3 DSI/QLED | `[0, -0.025, 0.105]` | `[0.1055, 0.008, 0.0672]` | 供应商型号已选，夹具待试装 |
| Camera Module 3 Wide | `[0, -0.030, 0.151]` | `[0.025, 0.0114, 0.023862]` | 供应商包络 |
| VL53L5CX ToF 载板 | `[0.026, -0.034, 0.151]` | `[0.012, 0.003, 0.010]` | 载板/排线待冻结 |
| IMU | `[0, 0.008, 0.010]` | `[0.032, 0.025, 0.008]` | RL 假设 |
| 主控/稳压托盘 | `[0, 0.022, 0.015]` | `[0.105, 0.020, 0.070]` | RL 假设 |
| 3S2P 电池+BMS | `[0, 0, -0.044]` | `[0.075, 0.038, 0.038]` | RL 假设 |

完整质量范围和光学帧见 `generated/config/round_v1_electronics_sensor_layout.json`。

## 推荐顺序

1. 原骨架上吊架，未装电子/外壳时手动走完守护范围。
2. 扫描 16 个总线 ID；一次只使能一个关节，低扭矩确认正方向、零位、硬限位和空载电流。
3. 低位固定电池/BMS，保证保险丝、急停和总电源可达。
4. 安装主控/稳压托盘与刚性 IMU 座；确认散热和连接器拔插空间。
5. 先做 4.3 屏夹具试装券，再将屏、摄像头、ToF 安装到前头壳内，检查排线弯曲半径和视场。
6. 布线后执行全关节慢速扫掠；线束不得跨输出盘、夹点或锐边。
7. 先用假件试装左右上臂套、前臂套与固定 Q 版手掌，确认无夹线、无关节扫掠侵入；手掌不得用于承载或抓取。
8. 安装加厚脚底和压力位，确认压力载荷不被硬壳旁路。
9. 逐片安装胸、骨盆、头壳；检查动态间隙和维修可达性。
10. 吊架上重复零位、站立位、单关节与双腿低速测试。

## 通电/行走门禁

- 所有关节在守护范围无硬碰和夹线。
- 保险丝、急停、BMS、极性、绝缘完成。
- 16 个 confirmed ID、zero、direction 无空缺/重复。
- 命令限位与 URDF 一致，初始连续扭矩不超过 `1.2552512 N·m`。
- 首轮使用吊架、低电流限制和可物理断电的操作员。

任一项不满足时，不进入落地步行。
