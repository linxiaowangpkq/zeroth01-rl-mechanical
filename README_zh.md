# Zeroth-01 v5 原始 16DoF 实体连接 / RL 基线

v5 回到原始 Zeroth-01 的 16 个关节和矮胖比例，不增加踝 roll、不缩短小腿、不安装夹爪或大手掌。仅保留这些小改动：头罩上下左右各扩 5 mm 并加圆角、M5Stack UnitV2 摄像头/麦克风可拆安装、头部无脖子直连、两个小型固定腕端、9 mm 白色下宽上窄可更换脚底，以及真实 FEETECH STS3250 的机壳侧/输出侧传动接口。

## RL 唯一入口

- URDF：`generated/urdf/physical_mount_v5_original_16dof_solidworks_motion/zeroth01_physical_mount_v5_original_16dof_solidworks_motion.urdf`
- MuJoCo/MJX：`generated/mujoco/physical_mount_v5_original_16dof_solidworks_motion/zeroth01_physical_mount_v5_original_16dof_solidworks_motion_mjx.xml`
- 舵机轴、限位、壳体/输出归属：`generated/config/physical_mount_v5_original_16dof_solidworks_motion_actuator_layout.json`
- 硬件标定模板：`generated/config/physical_mount_v5_original_16dof_solidworks_motion_hardware_calibration.csv`
- RL 交接：`generated/config/physical_mount_v5_original_16dof_solidworks_motion_rl_handoff.json`
- 交付文件 SHA-256 清单：`generated/config/physical_mount_v5_original_16dof_solidworks_motion_delivery_manifest.json`
- 总放行边界：`reports/v5_original_16dof_solidworks_motion/release_gate.json`
- 85 组件 CAD 清单：`generated/cad/physical_mount_v5_original_16dof_solidworks_motion/ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json`
- 85 组件彩色中立位审阅 GLB：`generated/cad/physical_mount_v5_original_16dof_solidworks_motion/ZEROTH01_V5_ORIGINAL_16DOF_NEUTRAL_REVIEW.glb`
- 17 个刚体 link STEP：`generated/cad/physical_mount_v5_original_16dof_solidworks_motion/motion_link_parts/`
- 问题闭环：`bug.md`

## 已通过的当前数字门禁

| 项目 | 当前 v5 结果 |
|---|---:|
| 拓扑 | 16 个 revolute joints；无踝 roll |
| 舵机 | 16 × FEETECH STS3250-C001 采购精确 STEP |
| 装配组件 | 85 |
| 刚体运动 link | 17；全部正体积、OCCT B-Rep 有效 |
| 彩色审阅 GLB | 85 组件、347 solids、包络与 B-Rep 报告一致 |
| 中立位精确实体干涉 | 0 / 3570 对；0 Boolean 错误 |
| CAD 总包络高度 | 490.489 mm，PASS ≤ 500 mm |
| MJCF 站立运动学高度 | 416.448 mm |
| URDF/MJCF 标称质量 | 2.745759 kg，PASS ≤ 3 kg |
| MuJoCo 3.3.7 | `nq=23`, `nv=22`, `nu=16`，编译 PASS |
| 髋 yaw 固定座全限位扫掠 | 左右各 17 姿态，0 干涉 |

髋 yaw 的力矩链不是悬空外观件：正体积躯干内融合两组窄肋/螺柱，每侧两根 M2×8 固定 STS3250 后盖，输出端通过 PCD14 四孔桥和四个 M3 套筒连接 U 形髋架；左右 U 形髋架仅切除约 55.3 mm³，并在原限位内验证固定座/螺钉与运动件为 0 干涉。其他关节保留原 Zeroth 承力槽位，采购精确舵机壳体归属静止侧，PCD14 桥归属运动侧。

## SOLIDWORKS 真实 Motion 状态

当前 85 组件清单的离线精确 B-Rep 门禁已 PASS，但原生 SOLIDWORKS 静态重建和 16 关节逐个上下限 Motion 尚未放行。原因是工作站上既有 `SLDWORKS.exe` 进程无响应；旧 `solidworks_gate.json` 对应旧 manifest，必须视为 **STALE**。在当前 manifest 完成 32/32 原生限位运行并生成真实帧之前，不提供 Motion GIF，也不把 `solidworks_all_joints_isolated_motion` 标为 PASS。

## 实物边界

- 计算板、电池/BMS、IMU 和线束预留均在躯干内部；橙/紫/绿只代表受控包络，不是外露背板。
- 黑色脚底数量为 0；白色脚底上窄下宽、厚 9 mm、四个 M3 通孔，可先打印左右试片。
- 手部是两 M3 固定的小圆角支撑，不是夹爪、球手或大方块。
- 标称质量来自工程质量/惯量账本，真实首件必须逐 link 称量并做 SysID 后覆盖。
- 总线 ID、机械零位、方向、背隙、延迟、摩擦、电流和温升均未实测，不得从候选值臆造。

因此，v5 可作为数字 RL 起点；整套直接打印、装配和上电仍是 **physical first article HOLD**。先按 `ASSEMBLY_GUIDE_zh.md` 做 STS3250 槽位/输出桥、腕端和脚底试片，再冻结材料、螺钉长度、电子器件与线束。
