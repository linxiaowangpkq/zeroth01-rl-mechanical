# Zeroth-01 最小可靠圆润版 RL 机械交接

## 唯一机械基线

RL 训练只加载：

- `generated/urdf/zeroth01_rl_round_v1.urdf`
- `generated/mujoco/zeroth01_rl_round_v1.xml`

不要从 SolidWorks 彩色审阅标识重建动力学，也不要加载被否决的 replacement-servo/cage/hub 实验模型。S01–S16 只是 canonical joint 的视觉 ID。

## 拓扑和质量

- 自由基座：`nq=23`、`nv=22`。
- 26 bodies（含 world）。
- 17 joints，其中 16 个 hinge。
- 16 actuators。
- 8 sensors。
- 名义总质量：`4.586857125474 kg`，与 URDF 一致。

质量来源：

- 原始聚合 link：`3.0954718282 kg`。
- 圆润壳体/脚底：`0.9423852973 kg`。
- 电子件：`0.549 kg`。

原始 link 已聚合内部舵机/结构质量，不得再次叠加 16×74.5 g。

## 传感器

MJCF 中已定义：

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
| conservative thermal start | `1.2552512 N·m` | 首轮 RL/台架连续参考 |
| manufacturer rated | `1.569064 N·m` | 厂商额定点评估 |
| legacy official sim | `2.0 N·m` | 仿真峰值；非连续硬件额定 |
| manufacturer stall | `4.903325 N·m` | 堵转边界；禁止作连续动作上限 |

初始速度上限 `5 rad/s`。仿真 damping、frictionloss、armature 是官方基线参数，不是本机系统辨识结果。

完整逐关节参数见：

- `generated/config/zeroth01_actuator_metadata.json`
- `generated/config/sts3250_round_v1_rl_profiles.json`
- `reports/joint_servo_frames.csv`

## 训练建议

1. 从 `1.2552512 N·m` 连续扭矩上限开始，保持 URDF 守护限位。
2. 使用动作平滑、关节功率、足底滑移、足底冲击、姿态和温度代理惩罚。
3. 对 link mass、damping、armature、frictionloss 和 ±2° 零位偏差做域随机化；范围见 actuator metadata。
4. 记录逐关节扭矩/速度 p50、p90、p95、p99，RMS/峰值电流代理和机械功率。
5. 把足底四点接触和 ToF 纳入 observation ablation，避免策略只依赖理想 base state。
6. 先做站立和原地踏步，再做低速前进；真实硬件前必须通过 sim2sim 和吊架策略回放。

## STS3250 可行性判据

当前 `reports/sts3250_round_v1_feasibility.json` 已用 100,000 个守护姿态做准静态重力采样；最坏值为 `0.339869 N·m`（right_hip_pitch），静态额定扭矩门禁通过。该结果不包含惯性、足底冲击、跟踪误差、供电压降或温升，所以 walking gate 仍为 `UNVERIFIED`。

训练不能单独证明舵机可行。需要将策略轨迹回放到台架并测量：

- 每关节连续/峰值扭矩和速度分布。
- RMS 电流、峰值电流、母线压降。
- 线圈/壳体温升与热稳态。
- 背隙、死区、跟踪误差和通信延迟。
- 脚底冲击、滑移和跌倒恢复峰值。

只有 p99 轨迹在额定/热/供电边界内且留有余量，才能判定 STS3250 对该关节可行。

## 验证证据

`reports/mujoco_round_v1_gate.json`：

- 16 joints × 101 轴向样本。
- 100,000 随机姿态。
- 65,536 边界组合。
- 零位、站立位和上述采样均无自碰撞样本。
- 1,000 步有限动态响应通过。

`reports/rl_package_portability_gate.json`：

- URDF 27 个 mesh 引用。
- MJCF 27 个 mesh 引用。
- 相对路径和大小写一致。

证据范围是离散网格运动学/动力学，不覆盖线束、紧固件、打印公差或结构变形。

## sim-to-real 前必须回填

`generated/config/zeroth01_hardware_calibration_template.csv` 中每个关节的：

- confirmed bus ID
- zero count / offset
- URDF-to-servo direction
- 实测软/硬限位
- backlash/deadband
- no-load current

并更新实际电池、主控、IMU、线束和外壳的质量/质心/惯量。未完成前，`hardware_deployment_ready` 必须保持 false。
