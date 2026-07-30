# Zeroth-01 round-v1：RL 机械交接

## 训练入口

- URDF：`generated/urdf/zeroth01_rl_round_v1.urdf`
- 原生 MuJoCo：`generated/mujoco/zeroth01_rl_round_v1.xml`
- 执行器元数据：`generated/config/zeroth01_actuator_metadata.json`
- STS3250 扭矩档位：`generated/config/sts3250_round_v1_rl_profiles.json`
- 圆润件质量/惯量：`generated/config/round_v1_mass_properties.json`
- 摄像机/IMU/计算板/电池和足底传感器：`generated/config/round_v1_electronics_sensor_layout.json`
- 碰撞策略：`generated/config/zeroth01_collision_policy.json`
- 实机标定模板：`generated/config/zeroth01_hardware_calibration_template.csv`

## 冻结的机械事实

- 上游运动树为 18 links、17 joints；round-v1 再加入 4 个有质量电子模块和 1 个无质量相机光学 frame，最终为 23 links、22 joints、16 个转动关节。
- 含电子舱名义总质量：`4.750138802624 kg`。
- 新增外观/鞋底名义质量：`1.087666974428 kg`。
- 电子舱名义质量：`0.567 kg`；均为 RL/布置假设，待实物选型称重。
- 坐标/单位：URDF/MJCF 使用 m、kg、s、rad；打印 STEP/STL 使用 mm。
- 鞋底比基线向下增加 8 mm，MuJoCo 初始浮动基座高度相应由 0.320 m 调为 0.328 m。
- 16 个执行器按 FEETECH STS3250 建模；真实 STEP 和 SolidWorks 零件均已交付。
- 相机位于面罩后、IMU 位于躯干中心、计算板位于后胸、电池位于胸腔下部；MuJoCo 包含固定相机、IMU 和 4 个足底压力触点。
- 不能再把 `16 × 74.5 g` 加到 URDF：基线 link 惯量来自聚合装配，已包含源装配质量；本次只叠加新打印件质量。

## STS3250 参数与训练边界

| 参数 | 值 | 使用方式 |
|---|---:|---|
| 额定电压 | 12 V | 名义供电 |
| 厂商额定扭矩 | 1.569064 N·m | 额定评估上限 |
| 建议初始连续训练上限 | 1.2552512 N·m | 额定扭矩的 80%，工程起点，非实测热限 |
| 现有上游仿真上限 | 2.0 N·m | 仅仿真参数，是额定扭矩的 127.46% |
| 厂商堵转扭矩 | 4.903325 N·m | 绝不能作为连续 RL 上限 |
| 空载速度 | 7.873666 rad/s | 厂商 12 V 名义值 |
| URDF 速度上限 | 5.0 rad/s | 训练限制 |
| 编码器 | 4096 count/rev | 零位名义 2048 count |

推荐先以 `conservative_thermal_start` 档训练，再以 `manufacturer_rated_evaluation` 档评估。不得因为 2.0 N·m 的上游 MuJoCo 参数能够行走，就推断真机可持续输出 2.0 N·m。

## 已通过的仿真门禁

- MuJoCo 3.11 加载、拓扑和质量导入：PASS。
- 中立位和官方站立位自碰撞：PASS。
- 16 个关节各 101 点轴扫掠、运动响应和碰撞：PASS。
- 100,000 个确定性随机安全盒姿态：0 自碰撞。
- 65,536 个安全盒角点：0 自碰撞。
- 11 个打印 STL：单组件、闭合、流形、绕序一致，PASS。
- 3 个舵机接口试装 STL：单组件、闭合、流形、绕序一致，PASS。
- SolidWorks 48 个姿态的父壳体固定/子输出随动传动门禁：PASS。
- 100,000 个安全盒准静态重力样本：最大关节重力矩 `0.339869 N·m`，低于额定扭矩，静态门禁 PASS。

这不等于 STS3250 已通过步行。当前仍缺训练后轨迹的扭矩-速度散点、RMS 电流、温升、母线压降、足部冲击、回差和跟踪误差。

## 舵机位置与可信度

- 轴心、轴线和关节名：`reports/joint_servo_frames.csv`。
- 16 个 STEP 实例的轴线共线门禁：`reports/round_v1_servo_axis_alignment.csv`。
- 壳体绕输出轴的 phase：`generated/config/zeroth01_sts3250_mount_phase.json`。

轴心/轴线来自 URDF，数值可用于仿真。壳体 phase 是用聚合表面拟合得到的候选值，16/16 的置信度仍为 `LOW`；真机布线、螺钉和壳体朝向必须根据支架 B-Rep 或实物重新锁定。

## 训练后必须输出

1. 每关节扭矩的 p50/p95/p99/max、RMS 和持续超限时间。
2. 每关节速度的 p50/p95/p99/max。
3. 扭矩-速度散点与实测 STS3250 包络的对比。
4. 足底接触冲量、滑移、离地高度和落脚速度。
5. 12 V 母线电压/电流、舵机温度和跟踪误差。
6. 在 1.2552512 N·m 和 1.569064 N·m 两个上限下的成功率与策略差异。

## 硬件阻塞项

- 实际 bus ID、零位 offset、URDF 到舵机方向符号；
- 实测硬限位、回差、死区、延迟和带载扭矩-速度曲线；
- 打印件实际质量/质心、线束及最终相机/IMU/电池/控制板质量和位置；
- 鞋底绑带拉脱、冲击、疲劳和地面摩擦；
- 承力舵机支架、舵盘、反侧轴承、金属紧固件和线束的完整生产 CAD。

在这些项完成前，只能声明 `simulation_ready=true`，不能声明 `hardware_walking_ready=true`。
