# Zeroth-01 Physical Mount V2 Minimal（RL 机械基线）

本版本以已经通过干涉检查的 Physical Mount V1 为机械主链，只做最小、可逆改动：替换头部、增加浅圆角胸板、删除旧爪并替换为小型 Q 手、增加 9 mm 可更换脚底。**16 个关节、安装骨架、关节轴与舵机归属均未重排。**

## 当前结论

| 项目 | 结果 | 证据 |
|---|---:|---|
| 外挂脖子 | **无独立脖子组件** | SolidWorks `external_neck_component_count=0` |
| 原骨架头柱最大外露 | **1.404 mm** | 自动门限 ≤5 mm，PASS |
| 头/胸装配缝 | 0.60 mm、无相交 | STEP 实体布尔门，PASS |
| 胸板前后深度 | 8.775 mm | 浅圆角面板，不再是厚方盒 |
| SolidWorks 装配 | 51/51 组件 | 20 承力件 + 16 独立蓝色舵机 + 15 外观/电子件组件 |
| 旧爪 | 0 | 两个原前臂固定爪已切除，FINGER link 改为 Q 手 |
| Q 手外包络 | 45.0 × 40.55 × 38.0 mm | 左右镜像，双耳 M3 横穿安装 |
| 脚底 | 9 mm | 左右可更换接触底，TPU/PETG 首件 |
| URDF | 16 个 revolute joint | 总标称质量 5.216675 kg，逐 link 质量/质心/惯量齐全 |
| MuJoCo | PASS | 中位、16×61 单关节采样、73 个协同姿态均无非相邻刚体碰撞 |
| 整机直接打印 | **HOLD** | 先做一颗 STS3250 量规和一套头/腕/脚底首件 |

## 唯一推荐入口

- RL URDF：`generated/urdf/physical_mount_v2_minimal/zeroth01_physical_mount_v2_minimal.urdf`
- RL 总交接：`generated/config/physical_mount_v2_minimal_rl_handoff.json`
- 16 舵机数据：`generated/config/physical_mount_v1_actuators.json`
- 舵机标定模板：`generated/config/physical_mount_v1_hardware_calibration_template.csv`
- 打印件质量/质心/惯量：`generated/config/physical_mount_v2_minimal_mass_properties.json`
- SolidWorks 便携透视装配：`generated/solidworks/physical_mount_v2_minimal/portable_flat/OPEN_FIRST_ZEROTH01_PHYSICAL_MOUNT_V2_MINIMAL_16_BLUE_SERVOS_XRAY.SLDASM`
- SolidWorks 便携正常外观：`generated/solidworks/physical_mount_v2_minimal/portable_flat/ZEROTH01_PHYSICAL_MOUNT_V2_MINIMAL_WHITE_NORMAL.SLDASM`
- STEP/STL：`generated/cad/physical_mount_v2_minimal/parts/`
- 一键发布校验：`python scripts/validate_minimal_v2_release.py`

Physical Mount V1 被保留为来源机制和 STS3250 首件接口依据，但不再是推荐训练入口。

## 坐标、电子件与传感器位置

URDF 使用米/千克/弧度；机器人左侧为 `+X`、前方为 `-Y`、上方为 `+Z`。以下均相对 `Z_BOT2_MASTER_BODY_SKELETON`：

| 部件 | SolidWorks 颜色 | 中心 XYZ (m) | 标称质量 | 状态 |
|---|---|---:|---:|---|
| Waveshare 4.3" DSI QLED 显示 | 青色 | `[0, -0.043, 0.070]` | 0.129 kg | 已选产品，夹具需打印试配 |
| Raspberry Pi Camera Module 3 Wide | 绿色 | `[0, -0.033, 0.105]` | 0.004 kg | 产品尺寸/STEP 依据 |
| VL53L5CX ToF 载板包络 | 紫色 | `[0.029, -0.038, 0.108]` | 0.002 kg | 传感器已选，载板待冻结 |
| SBC/载板/稳压器包络 | 橙色 | `[0, 0.042, -0.002]` | 0.180 kg | RL 假设，型号待冻结 |
| 3S2P 电池+BMS 包络 | 品红 | `[0, 0.028, -0.052]` | 0.340 kg | RL 假设，电芯/BMS 待冻结 |
| 躯干 IMU | 亮绿 | `[0, 0.012, 0.018]` | 0.012 kg | RL 假设，型号/方向待冻结 |

相机与 ToF 的 REP-103 光学坐标、四个脚底接触 frame 已写进 URDF 和 handoff JSON。正常 SolidWorks 装配隐藏内部电子件；透视装配显示全部不同颜色包络。

## 16 个蓝色舵机究竟是什么

SolidWorks 中的 `S01`–`S16` 是从原 Zeroth 装配中按原刚体归属提取的 **STS3215-family 安装实体**，不是随意摆放的蓝色盒子：

- 每颗舵机是独立 `.SLDPRT`；
- 每颗跟随其真实 owning link 和 16DoF 正向运动学；
- 左右看似不完全同向来自镜像机构和关节坐标，不允许为了画面对称而旋转；
- 装配树名称同时包含 `Sxx` 与关节名，可在 SolidWorks 中单独隐藏、剖视或测量。
- 发布仓库采用 SolidWorks Pack and Go 生成的 `portable_flat` 同目录包；两套 `.SLDASM` 与 51 个 `.SLDPRT` 放在同一目录，克隆后不依赖原开发机绝对路径。

目标执行器仍是 FEETECH `STS3250-C001`：12 V、74.5 g、额定持续扭矩 1.569 N·m、堵转扭矩 4.903 N·m、URDF 速度上限 3.0 rad/s。训练不得把堵转扭矩当持续输出。

STS3250 和源 STS3215-family 的名义壳体家族接近，但官方 STS3250 的 4×M2、25T 舵盘、后轴与实物公差尚未完成首件签核，所以蓝色件必须理解为“安装位置真值”，不是供应商精确 B-Rep。首件量规仍位于：

- `generated/cad/physical_mount_v1/sts3250_interface/FEETECH_STS3250_C001_DIMENSION_REFERENCE.step`
- `generated/cad/physical_mount_v1/sts3250_interface/STS3250_4XM2_FIRST_ARTICLE_FACE_GAUGE.step`
- `generated/print/physical_mount_v1/first_article/STS3250_4XM2_FIRST_ARTICLE_FACE_GAUGE.stl`

## 3D 打印和装配边界

17 组 v2 STEP/STL 均通过拓扑检查；其中头前壳、头后壳、胸板、相机支架、左右 Q 手和左右脚底为打印件。显示器、相机、ToF、计算板、电池、IMU 与 UI/window 几何是受控安装包络，不应直接当成打印件。

推荐顺序：

1. 打印 STS3250 4×M2 首件量规，只拿一颗实物验证孔、舵盘和后轴；
2. 打印一套头前/后壳、相机支架，实装显示器/相机/ToF 并检查 FFC/CSI 出线；
3. 打印一只 Q 手和一块脚底，验证双耳 M3、腕部间隙、落地面与紧固件；
4. 通过后才打印左右成套件；
5. 装 16 颗舵机时逐颗做总线扫描、方向点动和机械零位，填写 calibration CSV；
6. 走完整受限关节运动，确认线束弯曲半径，再称量各总成并更新 URDF 惯量。

因此结论不是“现在整机直接打印即可拼装”，而是“CAD/URDF 已可训练和做分件首件，整机打印仍由 STS3250 与头/腕/脚首件门控制”。

## 仿真证据与边界

- `reports/physical_mount_v2_minimal/geometry_gate.json`：头柱外露 1.404 mm、0.60 mm 头胸缝、Q 手与脚底尺寸、STEP/STL 拓扑；
- `reports/physical_mount_v2_minimal/dynamic_collision_gate.json`：MuJoCo 3.11.0 中位、单关节和协同动作；
- `reports/physical_mount_v2_minimal/solidworks_gate.json`：51 组件、16 舵机、0 旧爪、0 外挂脖子、视图和 GIF；
- `reports/physical_mount_v2_minimal/release_gate.json`：可移植 URDF、质量惯量、CAD 数量和交付完整性总门。

SolidWorks GIF 是 CLI 正向运动学驱动原生组件 transform 的证据；碰撞结论来自同一 URDF/几何的 MuJoCo 采样门。它不是 SolidWorks Motion 的连续接触求解，也不证明全 16 维关节极限笛卡尔积无自碰撞。RL 必须保留自碰撞惩罚/终止。

## RL 使用方式

先运行：

```bash
python scripts/validate_minimal_v2_release.py
```

然后以 v2-minimal URDF 为唯一机械真值，读取 handoff 中的关节、逐 link 惯量、传感器 frame、接触点、额定扭矩和域随机化范围。当前可做刚体 PPO 步行与 STS3250 额定扭矩可行性研究；sim-to-real 仍需实际总线/方向/零位、首件装配、线束、整机质量/质心/惯量、相机/ToF/IMU 标定和热/电压裕量。
