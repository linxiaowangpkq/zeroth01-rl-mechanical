# Zeroth-01 v4 原版最小改动 18DoF

这是仓库当前唯一推荐给 RL 的机械基线。它回到原始 Zeroth-01 的承力骨架、肩宽、髋距、四肢接口和矮胖比例，只保留为真实装配与 18DoF 训练必需的改动：原头罩上下左右名义各扩 5 mm 并做 5 mm 小圆角、无脖子直连、双踝 roll、7 mm 鞋底、下腿直段缩短 18 mm、紧凑固定手掌，以及可拆白色后勤舱。

ToddlerBot、KHR-3HV、TonyPi、Open Duck Mini 只用于借鉴分件维护、双支撑、线束和 MJX/SysID 流程；没有替换 Zeroth-01 的比例或关节拓扑。

## 先打开

- SolidWorks 普通装配：`generated/solidworks/physical_mount_v4_original_minimal/portable_flat/OPEN_FIRST_ZEROTH01_V4_ORIGINAL_MINIMAL_WHITE_18_BLUE_STS3250.SLDASM`
- SolidWorks 内部透视：`generated/solidworks/physical_mount_v4_original_minimal/portable_flat/OPTIONAL_XRAY_ZEROTH01_V4_ORIGINAL_MINIMAL_INTERNAL_LAYOUT.SLDASM`
- 中性交换 STEP：`generated/cad/physical_mount_v4_original_minimal/ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY.step`

普通装配有 57 个独立组件，其中 S01-S18 是 18 个独立蓝色 STS3250。透视装配额外显示橙色计算模块、紫色电池、绿色 IMU 和头部 UnitV2；它们在普通视图中由白色外壳遮蔽，而不是外挂在胸前。

## RL 唯一输入

- URDF：`generated/urdf/physical_mount_v4_original_minimal/zeroth01_physical_mount_v4_original_minimal_18dof.urdf`
- MuJoCo/MJX：`generated/mujoco/physical_mount_v4_original_minimal/zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml`
- 舵机轴、位置、限位：`generated/config/physical_mount_v4_original_minimal_actuator_layout.json`
- 质量、传感器、接触点、随机化：`generated/config/physical_mount_v4_original_minimal_rl_handoff.json`
- 总门禁：`reports/v4_original_minimal/release_gate.json`
- 文件哈希：`RELEASE_MANIFEST.json`

## 已通过的数字门

| 项目 | 结果 |
|---|---:|
| 自由度 | 18 revolute joints |
| 舵机 | 18 × FEETECH STS3250-C001 独立零件 |
| SolidWorks 组件 | 57 / 57 |
| SolidWorks 总高 | 498.959 mm，PASS ≤ 500 mm |
| URDF/MJCF 标称质量 | 2.850 kg，PASS ≤ 3 kg |
| 跨组件实体干涉 | 0，PASS |
| MuJoCo 运行时编译 | nq 25 / nv 24 / nu 18，PASS |
| 64 帧协调运动碰撞扫描 | 非地面穿透 0，PASS |
| STS3250 准静态重力峰值 | 0.534496 N·m，低于 1.255251 N·m 连续设计值 |

头罩外包络左右和顶部严格各 +5 mm；隐藏下角为两只肩舵机开了 0.8 mm B-Rep 避让槽，局部实心增量约 +3.9 mm。若把那里也做成完整 +5 mm，SolidWorks 会重新报告真实干涉。

## 头部与电子件

头部选用可采购的 M5Stack UnitV2（48 × 18.5 × 24 mm、18 g、GC2145 摄像头和麦克风）作为可拆交互模块，装入两片式打印头罩；头罩用隐藏 M3 螺母板直接连接原躯干，没有长脖子。后勤舱是可逆 M3 安装件，因为原始躯干来源是实心多面体而不是可可靠掏空的参数化薄壳。

受控内部包络：计算模块 70 × 12 × 32 mm、电池 75 × 22 × 34 mm、IMU 32 × 8 × 25 mm。具体计算板、电池容量、BMS、稳压和急停在实物电气设计冻结前仍是 HOLD。

## 真实放行边界

当前状态是“数字 CAD/URDF/MJCF 可用于 RL，物理首件 HOLD”。以下数据不能由 CAD 猜测：18 个总线 ID、机械零位、方向符号、实物分 link 质量/惯量、打印方向与强度、线束弯曲半径、电流/温升、冲击和跑步负载。请按 `ASSEMBLY_GUIDE_zh.md` 完成首件、单舵机台架和悬吊低速测试后再落地策略。

参考来源：

- ToddlerBot: https://github.com/hshi74/toddlerbot
- Open Duck Mini: https://github.com/apirrone/Open_Duck_Mini
- KHR-3HV: https://kondo-robot.com/faq/khr-3hv-erection-diagram
- TonyPi: https://docs.hiwonder.com/projects/TonyPi/en/latest/
- M5Stack UnitV2: https://docs.m5stack.com/en/unit/unitv2
