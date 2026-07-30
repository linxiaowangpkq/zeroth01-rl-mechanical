# Zeroth-01 Round V3 RL 机械交接

## 唯一机械基线

RL 训练只加载：

- `generated/urdf/zeroth01_rl_round_v1.urdf`
- `generated/mujoco/zeroth01_rl_round_v1.xml`

不要从 SolidWorks 蓝色舵机诊断件重建动力学，也不要加载已否决的 replacement-servo/cage/hub 实验。S01–S16 是 canonical joint 的视觉身份与受控外包络，不是额外刚体。

## 拓扑和质量

- 自由基座：`nq=23`、`nv=22`。
- URDF：26 links、25 joints，其中 16 个运动关节。
- MuJoCo：26 bodies、16 hinge joints、16 actuators。
- 8 sensors，另有固定 `head_camera`。
- 名义总质量：`4.997342616724 kg`，URDF 与 MJCF 一致。

质量来源：

- 原始聚合 link：`3.0954718282 kg`；
- V3 外壳/脚底/手臂套/Q 版手掌打印层：`1.234870788525 kg`；
- 电子器件：`0.667 kg`。

原始 link 已聚合内部舵机/结构质量，不得再次叠加 `16 × 74.5 g`。蓝色 SolidWorks 舵机诊断件的质量为零且不进入仿真。

左右 Q 版手掌是固定在 hand link 上的非灵巧外观/碰撞体，不增加 actuator 或 joint；策略不得假设独立手指自由度。

## 传感器

MJCF 已定义：

1. `base_orientation`
2. `base_angular_velocity`
3. `base_linear_acceleration`
4. `left_front_pressure_touch`
5. `left_rear_pressure_touch`
6. `right_front_pressure_touch`
7. `right_rear_pressure_touch`
8. `tof_center_range`

另有固定 `head_camera`。相机内参、畸变、曝光、延迟以及相机到 Torso 外参仍需实机标定。

## 执行器初始模型

模型：FEETECH STS3250，12 V，4096 counts/rev，名义中位 2048。

| 配置 | 扭矩上限 | 用途 |
|---|---:|---|
| conservative thermal start | `1.2552512 N·m` | 首轮 RL / 台架连续参考 |
| manufacturer rated | `1.569064 N·m` | 厂商额定点评估 |
| legacy official sim | `2.0 N·m` | 仿真峰值；非连续硬件额定 |
| manufacturer stall | `4.903325 N·m` | 堵转边界；禁止作连续动作上限 |

初始速度上限 `5 rad/s`。仿真 damping、frictionloss、armature 是基线参数，不是本机系统辨识结果。

完整逐关节参数见：

- `generated/config/zeroth01_actuator_metadata.json`
- `generated/config/sts3250_round_v1_rl_profiles.json`
- `reports/joint_servo_frames.csv`

## 训练建议

1. 从 `1.2552512 N·m` 连续扭矩上限开始，保持 URDF 守护限位。
2. 使用动作平滑、关节功率、足底滑移、足底冲击、姿态和温度代理惩罚。
3. 对 link mass、damping、armature、frictionloss 和 `±2°` 零位偏差做域随机化。
4. 记录逐关节扭矩/速度 p50、p90、p95、p99，RMS/峰值电流代理和机械功率。
5. 将足底四点接触和 ToF 纳入 observation ablation，避免策略只依赖理想 base state。
6. 先训练站立和原地踏步，再做低速前进；实物前必须完成 sim2sim 和吊架策略回放。

## STS3250 可行性判据

`reports/sts3250_round_v1_feasibility.json` 的 100,000 个守护姿态静态重力采样最坏值为 `0.339869 N·m`（`right_hip_pitch`），静态额定扭矩门禁通过。

该结果不包含惯性、足底冲击、跟踪误差、供电压降或温升，所以 walking gate 仍为 `UNVERIFIED`。训练完成后必须把策略轨迹回放到台架并测量：

- 每关节连续/峰值扭矩和速度分布；
- RMS 电流、峰值电流和母线压降；
- 线圈/壳体温升与热稳态；
- 背隙、死区、跟踪误差和通信延迟；
- 足底冲击、滑移及跌倒恢复峰值。

只有 p99 轨迹在额定、热和供电边界内且留有余量，才能判定 STS3250 对相应关节可行。

## 验证证据

`reports/mujoco_round_v1_gate.json`：

- 16 joints × 101 轴向样本；
- 100,000 随机姿态；
- 65,536 边界组合；
- 零位、站立位和上述样本均无自碰撞样本；
- 1,000 步有限动态响应通过。

`reports/solidworks_round_v1_kinematic_sweep.csv` 与 `reports/solidworks_round_v1_transmission_semantics.csv`：

- 16 个关节 × lower/zero/upper，共 48 行；
- Transform2 readback 和 parent/output 传动语义全部 PASS。

`reports/rl_package_portability_gate.json`：

- URDF/MJCF mesh 引用存在；
- 相对路径、大小写和质量合同一致。

这些证据覆盖离散网格运动学/动力学，不覆盖线束、紧固件、打印公差或结构变形。

## sim-to-real 前必须回填

在 `generated/config/zeroth01_hardware_calibration_template.csv` 中为每个关节填写：

- confirmed bus ID；
- zero count / offset；
- URDF-to-servo direction；
- 实测软/硬限位；
- backlash/deadband；
- no-load current。

并更新实际电池、主控、IMU、线束和外壳的质量、质心与惯量。未完成前，`hardware_deployment_ready` 必须保持 `false`。
