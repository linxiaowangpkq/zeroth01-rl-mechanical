# Zeroth-01 最小可靠圆润版机械与 RL 交付

## 结论

本版本采用已经装配并通过运动/碰撞检查的 Zeroth-01 17-link 机构作为唯一机械基线，只做四类低风险改动：

1. 头部改为三轴椭球外壳、复合曲面面罩，并布置 Waveshare 双圆屏、广角相机和 ToF。
2. 胸部、骨盆外壳的锋利边缘改为圆角。
3. 左右脚底加厚。
4. 用稳定颜色标识 16 个原始关节位置及电池、主控、IMU、相机、ToF、屏幕和足底压力位。

没有向原机构内强行加入新的 STS3250 实体、舵机笼、齿轮或输出盘，也没有改变 16 个关节的父子关系、轴线或零位。SolidWorks 中的 S01–S16 彩色圆片是**非物理标注**，不是舵机、齿轮、碰撞体或质量体。

## 权威产物

| 用途 | 文件 |
|---|---|
| SolidWorks 总装 | `generated/solidworks/round_v1/OPEN_FIRST_ZEROTH01_ROUND_V2_MINIMAL_COSMETIC.SLDASM` |
| 便携 SolidWorks 总装 | `generated/solidworks/portable_flat_round_v2/OPEN_FIRST_ZEROTH01_ROUND_V2_MINIMAL_COSMETIC.SLDASM` |
| RL/ROS 机械描述 | `generated/urdf/zeroth01_rl_round_v1.urdf` |
| MuJoCo 训练模型 | `generated/mujoco/zeroth01_rl_round_v1.xml` |
| 执行器元数据 | `generated/config/zeroth01_actuator_metadata.json` |
| 质量/惯量 | `generated/config/round_v1_mass_properties.json`、`reports/link_inertial_audit.csv`、`reports/round_v1_link_inertial_overlay.csv` |
| 电子舱与传感器 | `generated/config/round_v1_electronics_sensor_layout.json` |
| 舵机位置/轴线 | `reports/joint_servo_frames.csv` |
| 硬件标定模板 | `generated/config/zeroth01_hardware_calibration_template.csv` |
| 碰撞策略 | `generated/config/zeroth01_collision_policy.json` |
| 运动 GIF | `snapshots/solidworks/round_v1/zeroth01_round_v1_solidworks_motion.gif` |
| 验证报告 | `reports/solidworks_round_v1_gate.json`、`reports/mujoco_round_v1_gate.json` |

不要使用文件名中带有 installation audit、replacement servo、cage 或 hub 的实验产物作为当前机械基线；它们是已否决方案的分析证据。

## 视觉证据

![S01-S16 关节身份与颜色](snapshots/solidworks/round_v1/zeroth01_round_v2_joint_identity_front.png)

![电子件位置与颜色](snapshots/solidworks/round_v1/zeroth01_round_v2_electronics_annotated_front.png)

![SolidWorks 12 帧运动验证](snapshots/solidworks/round_v1/zeroth01_round_v1_solidworks_motion.gif)

## 当前模型数字

- 17 个运动树关节，其中 16 个转动关节。
- MuJoCo：26 bodies（含 world）、17 joints、16 actuators、8 sensors。
- 名义总质量：`4.586857125474 kg`。
  - 原始 Zeroth-01 聚合 link 质量：`3.0954718282 kg`。
  - PETG 名义外壳/脚底叠加质量：`0.9423852973 kg`。
  - 电子件名义质量：`0.549 kg`。
- 外壳、电子件和线束的最终实测质量尚未回填，因此这些惯量适合 RL 初始训练和域随机化，不是量产质检数据。

原始 link 惯量已经聚合了机构内部结构；不得再把 16 个 `74.5 g` 舵机质量重复叠加到 URDF。

## 头部方案

- 显示：Waveshare 0.71inch DualEye LCD Module，双 160×160 圆形 IPS，供应商 STEP 已归档。
- 相机：Raspberry Pi Camera Module 3 Wide，IMX708 自动对焦，120° 对角视场；精确供应商 STEP 已归档。由于该模型含 631 个实体并会使 SolidWorks 自动导入长时间无响应，工作总装使用官方实测包络，精确 STEP 保留作接口复核。
- 距离传感器：ST VL53L5CX，8×8 多区 ToF，65°，最高 4 m/60 Hz；当前 12×10 mm 载板包络是假设，PCB 和排线出口尚未冻结。
- 外观：屏幕本体仍是平面器件，安装在椭球头壳和复合曲面面罩之后；没有声称它是真正可弯曲显示屏。

供应商资料：

- https://www.waveshare.com/wiki/0.71inch_DualEye_LCD_Module
- https://www.raspberrypi.com/products/camera-module-3/
- https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html

## STS3250 说明

当前官方 STS3250-C001 外形证据为约 `45.22 × 24.72 × 35 mm`、25T/OD5.9 输出、M3 输出螺纹、质量约 `74.5 g`。仓库中历史 STEP 的标题是 `ST-3235M-20211119-A_ASM`，包围盒约 `45.220049 × 37.400057 × 24.720050 mm`，且轴线为 `+Y`；它不是已确认的 C001 模型，已隔离，未装入最终总装。

因此本交付确认的是“原 Zeroth-01 已装配机构的关节位置、轴线和运动范围”，不是重新证明某个未知支架能装入一套新画的 C001 舵机。采购前仍需用实物或供应商 C001 原生 CAD 做接口量测。

## 已通过的验证

- SolidWorks：51 个组件 = 17 个原始 link + 18 个圆润/电子叠加件 + 16 个非物理颜色标记；替代 STS3250、笼体、输出 hub 数均为 0。
- 16 关节 × 101 轴向采样、100,000 随机姿态、65,536 边界组合：无自碰撞样本。
- 站立位、零位、动态响应、质量一致性：通过。
- URDF/MJCF 共 27 个 mesh 引用，大小写和相对路径：通过。
- 11 个打印 STL：水密、绕向一致、无非流形边，STEP/STL 体积误差不超过 0.5%。
- 12 帧 SolidWorks 运动 GIF：通过。
- 100,000 个守护姿态的准静态重力采样：最大 `0.339869 N·m`（right_hip_pitch），低于 `1.569064 N·m` 厂商额定点；这只证明静态重力裕量，不证明动态步行、热稳态或冲击可行。

这些是离散网格/解析代理的运动证据，不是连续碰撞证明，也不覆盖线束、紧固件、公差、柔性外壳变形和真实跌落冲击。

## 可用性边界

- **RL 仿真：可用。** URDF/MJCF、质量惯量、执行器初始限制、传感器帧和碰撞策略已齐。
- **外壳试打/装配空间验证：可用。** 11 个 STL 的网格门禁已通过。
- **打印后直接拼装并行走：不可宣称。** 原机构的承载支架、轴承、舵盘、螺钉、热熔螺母、线束、BMS、主控和真实安装公差仍需冻结。
- **真实 STS3250 可行性：待台架验证。** 必须完成每台舵机 ID、方向、零位、背隙、电流、温升和扭矩-速度标定。

详细边界见 `PRINT_AND_ASSEMBLY_READINESS_zh.md` 和 `ASSEMBLY_GUIDE_zh.md`。

## 最短使用路径

1. RL 会话先读 `one-seq.md`。
2. 训练以 URDF 与 MJCF 为唯一机械基线，不加载 S01–S16 彩色标注。
3. 初始连续扭矩参考使用 `1.2552512 N·m`，不要用堵转扭矩作连续动作上限。
4. 将真实样机称重、舵机标定和系统辨识结果回填到 `generated/config/` 后再做 sim-to-real。
