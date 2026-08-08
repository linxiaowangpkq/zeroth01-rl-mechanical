# Zeroth-01 v4 真实连接版 18DoF

这是本仓库唯一推荐给 RL 的机械基线。它保留原始 Zeroth-01 的承力躯干、肩、手臂、髋、大腿、关节轴和原始脚，只做必要修改：头罩上下左右各扩 5 mm 并加小圆角、M5Stack UnitV2 摄像头/麦克风可拆安装、无脖子直连、双踝 roll、下腿直段缩短 18 mm，以及 18 个 STS3250 的真实机壳侧和输出侧紧固件链。

已从安装清单和便携 SolidWorks 目录删除：方块手掌、外挂后勤舱、额外黑色 7 mm 鞋底、远置踝实验件、髋部虚构整圆垫片。手臂终端回到原始轻量腕端；脚底黑边不是当前零件。计算板、电池和 IMU 最终必须放进躯干，但精确选型、托盘和线束在首件冻结前仍为 HOLD。

## 先打开

- 正常 SolidWorks 装配：`generated/solidworks/physical_mount_v4_original_minimal/portable_flat/OPEN_FIRST_ZEROTH01_V4_ORIGINAL_MINIMAL_WHITE_18_BLUE_STS3250.SLDASM`
- 透视 SolidWorks 装配：`generated/solidworks/physical_mount_v4_original_minimal/portable_flat/OPTIONAL_XRAY_ZEROTH01_V4_ORIGINAL_MINIMAL_INTERNAL_LAYOUT.SLDASM`
- 完整 STEP：`generated/cad/physical_mount_v4_original_minimal/ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY.step`
- URDF：`generated/urdf/physical_mount_v4_original_minimal/zeroth01_physical_mount_v4_original_minimal_18dof.urdf`
- MuJoCo/MJX：`generated/mujoco/physical_mount_v4_original_minimal/zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml`
- 舵机轴、位置、限位：`generated/config/physical_mount_v4_original_minimal_actuator_layout.json`
- 质量、惯量、传感器、接触与随机化：`generated/config/physical_mount_v4_original_minimal_rl_handoff.json`
- 总门禁：`reports/v4_original_minimal/release_gate.json`

## 已通过的数字门禁

| 项目 | 结果 |
|---|---:|
| 自由度 | 18 个 revolute joints |
| 舵机 | 18 × FEETECH STS3250-C001 精确采购 STEP |
| SolidWorks 组件 | 75 / 75 |
| SolidWorks 总高 | 489.989 mm，PASS ≤ 500 mm |
| URDF/MJCF 标称质量 | 2.850 kg，PASS ≤ 3 kg |
| 未批准实体干涉 | 0，PASS |
| 螺纹接合 | 8 处：左右髋各 4 根 M3 进入 PCD14 内螺纹；单处 ≤ 1.25 mm³ |
| MuJoCo 编译 | nq 25 / nv 24 / nu 18，PASS |
| 64 帧协调运动扫掠 | 非地面穿透 0，PASS |
| STS3250 准静态重力峰值 | 0.474776 N·m，低于 1.255251 N·m 连续设计值 |

每个关节均有：蓝色精确 STS3250、机壳侧 4×M2 固定接口、输出侧 2.05 mm PCD14 桥、按原载荷路径归属的子侧连接。髋 yaw 为消除机身碰撞，舵机轴向外移 4 mm，并用 4 根 M2 机壳螺钉和 4 根 M3 输出拉杆闭合力矩链；没有整块垫片或背板。左右踝使用镜像直连支架，附加黑色鞋底为 0 件。

当前仅放行数字 CAD/URDF/MJCF 给 RL。实物首件仍是 **HOLD**：必须验证购买到的舵机尺寸/孔位、打印强度、螺钉与工具可达、躯干内电子托盘、线束弯曲半径、总线 ID/零位/方向、实测分 link 质量惯量、电流、温升和动态策略轨迹。
