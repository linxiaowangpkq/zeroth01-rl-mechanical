# Zeroth-01 机械模型与 RL 交付

本目录把 Zeroth-01 的公开机械资产整理成一套可复现的 16DoF 机器人描述，并增加了更高曲率的头/胸/骨盆外壳、8 mm 加厚鞋底、内部躯干主脊、电子舱包络和真实 STS3250 传动接口。当前结论：

- `round_v1_rl_simulation_ready = true`：圆润版 URDF、MuJoCo MJCF、名义质量/质心/惯量、关节轴、限位、碰撞策略和执行器元数据已齐全。
- SolidWorks CLI FK 复核通过：81 个组件包含 17 个上游 link、16 个外观/内部/电子件、16 个显式 STS3250、16 个父侧舵机笼和 16 个子侧输出转接组件；16 个转动关节、48 个 lower/zero/upper 姿态全部通过。
- 传动语义已修复：舵机壳体和笼体固定在父 link，前/后输出转接盘跟随子 link，48/48 姿态的轴心和转角误差门禁通过。
- MuJoCo 完整门禁通过：101 点/关节、100,000 个确定性随机姿态和 65,536 个安全盒角点均为零自碰撞。
- Linux 大小写敏感路径门禁通过：URDF/MJCF 的 27 个网格引用均与磁盘文件名精确匹配。
- `round_v1_nominal_mass = 4.750139 kg`：由上游聚合质量 `3.095472 kg`、安装外观/鞋底名义质量 `1.087667 kg` 和电子舱假设质量 `0.567 kg` 组成；实机必须称重后重生成惯量。
- `hardware_deployment_ready = false`：每台实机的舵机 ID、正方向、零偏、硬限位、回差和系统辨识尚未实测，不能直接下发到真机。
- `complete_print_and_walk_kit_ready = false`：公开的 17 个 link STL 是非流形聚合可视网格，不是生产 BOM；本次新增 11 个最终外观/鞋底 STL 和 3 个接口试装 STL，承力笼/转接盘仍须用 STEP 在实物舵盘、轴承、紧固件和公差确认后 CNC。

## 直接使用的文件

| 用途 | 文件 |
|---|---|
| 圆润版 RL 首选 URDF | `generated/urdf/zeroth01_rl_round_v1.urdf` |
| 圆润版 MuJoCo 首选模型 | `generated/mujoco/zeroth01_rl_round_v1.xml` |
| 原始质量基线 URDF | `generated/urdf/zeroth01_rl_ready.urdf` |
| 关节/电机/PD/随机化元数据 | `generated/config/zeroth01_actuator_metadata.json` |
| STS3250 RL 扭矩档位 | `generated/config/sts3250_round_v1_rl_profiles.json` |
| 舵机轴位置和方向 | `reports/joint_servo_frames.csv` |
| 16 个 STS3250 放置/轴线核验 | `reports/round_v1_servo_axis_alignment.csv` |
| 每个 link 的质量、质心和惯量 | `reports/link_inertial_audit.csv` |
| 圆润打印件和合并后惯量 | `generated/config/round_v1_mass_properties.json` |
| 摄像机/IMU/计算板/电池位置和惯量 | `generated/config/round_v1_electronics_sensor_layout.json` |
| 碰撞白名单和安全启动关节盒 | `generated/config/zeroth01_collision_policy.json` |
| 实机标定模板 | `generated/config/zeroth01_hardware_calibration_template.csv` |
| 真实 STS3250 原始 STEP | `source_assets/vendor/sts3250/FEETECH_STS3250.step` |
| 便携 SolidWorks 零件与装配体 | `generated/solidworks/portable_flat_round_v1/` |
| 圆润版 SolidWorks 联动动画 | `snapshots/solidworks/round_v1/zeroth01_round_v1_solidworks_motion.gif` |
| 22 个 CAD STEP | `generated/cad/round_v1/parts/` |
| 11 个最终外观/鞋底 STL | `generated/print/round_v1/final/` |
| 3 个接口试装 STL | `generated/print/round_v1/fit_check_non_load_bearing/final/` |
| 传动接口门禁 | `reports/round_v1_integrated_interface_gate.json` |
| 完整装配指南 | `ASSEMBLY_GUIDE_zh.md` |
| 打印/装配真实性说明 | `PRINT_AND_ASSEMBLY_READINESS_zh.md` |
| 圆润版 RL 交接说明 | `RL_ROUND_V1_HANDOFF_zh.md` |

`zeroth01_rl_reference.urdf` 仅用于追溯官方几何兼容模型；`zeroth01_rl_audited.urdf` 用于逐轴审计；新训练默认使用 `zeroth01_rl_round_v1.urdf`，若不安装圆润外壳/鞋底才退回 `zeroth01_rl_ready.urdf`。

## 已通过的门禁

| 检查 | 结果 |
|---|---|
| 23 links、22 joints、16 moving joints 的连通树 | PASS |
| 上游基线总质量 | 3.0954718282 kg |
| round-v1 外观/结构名义质量 | 4.183138802628 kg |
| 含电子舱 RL 名义总质量 | 4.750138802624 kg |
| 所有惯量正定并满足三角不等式 | PASS |
| MuJoCo 原生 MJCF 中立位/官方站立位 | 无非白名单碰撞 |
| 16 个关节运动、动力学和轴向碰撞门禁 | PASS |
| 100,000 个确定性随机姿态 | 0 个非白名单碰撞 |
| 65,536 个安全盒角点 | 0 个非白名单碰撞 |
| SolidWorks 48 个关节姿态 | 48/48 OK |
| 父侧壳体固定、子侧输出随动 | PASS |
| SolidWorks 最终组件位姿误差 | 最大 5.82e-17 m |

安全启动盒是围绕官方站立姿态搜索得到的 30% 统一范围，并不是完整物理关节范围。它是离散采样证据，不是连续碰撞数学证明。

## 关键版本锁定

公开 Drive 中的 17 个 STL 与 `zeroth-sim` 提交
`33b0553bd085ff6360495497a8e86afaa801785d` 几何兼容。
提交 `43c5baa1287db078bef638308ef077445704be1d` 修改了 joint frame、惯量和 mesh 名称，但公开仓库没有同时提供对应的新 STL。把旧 STL 与新 frame 混用会造成脚部/组件分离，因此本交付明确锁定前者。

完整提交和 SHA-256 记录在 `reports/source_lock.json` 与
`reports/source_asset_manifest.csv`。

## 不能从当前公开资料推导的内容

- 精确实机总线 ID 和线束拓扑；
- URDF 正方向到每台舵机正方向的符号；
- 每台舵机的零偏、硬限位、回差、死区和温升降额；
- 电流到输出转矩的实测曲线；
- 线束、螺钉、打印公差、柔性外壳和变形后的连续间隙；
- 实际相机、IMU、计算板、电池/BMS/稳压器的型号、重量和标定值；
- 采购 25T 舵盘附件孔距、对侧轴承、紧固扭矩和承力件 RFQ；
- 可制造的原生 B-Rep 特征树。

公开 STL 是开放/绕向不完全一致的三角面网格。它足够用于运动学、渲染和受控碰撞审计，但不是 CNC、打印公差或装配放行依据。

## 可复现

在本目录使用 Python 3.12 环境：

```powershell
.\.venv312\Scripts\python.exe .\scripts\audit_source_assets.py
.\.venv312\Scripts\python.exe .\scripts\gen_zeroth01_urdf.py
.\.venv312\Scripts\python.exe .\scripts\audit_mesh_frames.py
.\.venv312\Scripts\python.exe .\scripts\gen_zeroth01_rl_audited.py
.\.venv312\Scripts\python.exe .\scripts\search_global_collision_safe_box.py
.\.venv312\Scripts\python.exe .\scripts\gen_zeroth01_rl_ready.py
.\.venv312\Scripts\python.exe .\scripts\build_rl_mechanical_package.py
.\.venv312\Scripts\python.exe .\scripts\gen_zeroth01_rl_mjcf.py
.\.venv312\Scripts\python.exe .\scripts\validate_rl_ready_mujoco.py
.\.venv\python311_embed\python.exe .\scripts\validate_rl_package_portability.py --urdf .\generated\urdf\zeroth01_rl_round_v1.urdf --mjcf .\generated\mujoco\zeroth01_rl_round_v1.xml
```

SolidWorks 全量检查使用 CLI/COM：

```powershell
.\.venv312\Scripts\python.exe .\scripts\create_solidworks_kinematic_review.py --frame-count 13
```

本机实测约 12 分钟。应在后台运行并轮询
`reports/solidworks_trace.log`；20 分钟没有完成时终止并诊断，不应无限等待。
