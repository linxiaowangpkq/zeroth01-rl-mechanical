# Zeroth-01 v3 RL-Fixed 机械与训练交付

这是仓库唯一推荐的 RL 机械基线：保留 Zeroth-01 v2 已连接的承力骨架和原 16 个关节，新增左右踝横滚形成 18DoF。旧自制头、长脖子、胸前外挂板、爪子、K151 草案、9 mm 鞋底和 50 mm 踝部草案均不是发布入口。

## 先打开哪个文件

SolidWorks 普通装配：

`generated/solidworks/physical_mount_v3_rl_fixed/portable_flat/OPEN_FIRST_ZEROTH01_V3_RL_FIXED_CONNECTED_WHITE_18_BLUE_STS3250.SLDASM`

透视装配：

`generated/solidworks/physical_mount_v3_rl_fixed/portable_flat/OPTIONAL_XRAY_ZEROTH01_V3_RL_FIXED_18_BLUE_STS3250.SLDASM`

## RL 训练唯一输入

- URDF：`generated/urdf/physical_mount_v3_rl_fixed/zeroth01_physical_mount_v3_rl_fixed_18dof.urdf`
- MuJoCo/MJX：`generated/mujoco/physical_mount_v3_rl_fixed/zeroth01_physical_mount_v3_rl_fixed_18dof_mjx.xml`
- S01–S18 舵机轴心、轴向、归属 link：`generated/config/physical_mount_v3_rl_fixed_actuator_layout.json`
- 质量、传感器、随机化范围和门禁：`generated/config/physical_mount_v3_rl_fixed_rl_handoff.json`
- 全部门禁：`reports/physical_mount_v3_rl_fixed/release_gates.json`
- 准静态扭矩：`reports/physical_mount_v3_rl_fixed/sts3250_quasistatic_torque_gate.json`

## 已验证数据

| 项目 | 结果 |
|---|---:|
| 自由度 | 18 个 revolute joints |
| 舵机 | 18× FEETECH STS3250-C001，SolidWorks 中为独立蓝色零件 |
| 标称 URDF/MJCF 质量 | 2.969171828 kg（≤3 kg） |
| SolidWorks 保守高度 | 499.236652 mm（≤500 mm） |
| 诊断 STEP 精确高度 | 494.817994 mm |
| SolidWorks 原生装配 | 51/51 组件 |
| 跨组件物理干涉 | 0 |
| 64 帧 MuJoCo 动作 | PASS，非相邻穿透 0 |

SolidWorks 仍会列出同一个旧躯干 SLDPRT 内部的 5 个多实体联合重叠，它们属于同一制造组件，不是安装件互相穿透；屏幕玻璃、表情点和摄像孔还有 3 个只显示、不制造、不计质量的参考层重叠。

## 头部与内部电子件

头部改为可直接采购的 M5Stack CoreS3 K128 主机，官方主机外形 `54 × 54 × 15.5 mm`，保守把官方整套 `72.7 g` 全部计入该 link。它包含 2 英寸 320×240 触摸屏、GC0308 摄像头、双麦克风、1 W 扬声器、环境/接近传感器、BMI270 IMU、BMM150 磁力计、Wi‑Fi 和电池。

CoreS3 固定嵌入上胸腔，世界坐标中心为 `(38.75, 0, 18) mm`，Z 范围 `-9..45 mm`，没有机械 pan/tilt，也没有脖子。隐藏的 2 mm 6061 U 型支架中心为 `(30, 0, 18) mm`；躯干已加工对应安装槽、4×M3 通孔和 0.20 mm 装配间隙。左右髋 yaw 舵机也有 0.30 mm 壳体避让腔。

普通装配只显示白色骨架/外件、蓝色舵机、黑色鞋底与屏幕；橙色计算模块、紫色电池、绿色躯干 IMU 只在透视装配中显示，实际安装在胸腔内。

## STS3250 结论

几何、轴线、安装位置、FK、地面接触和样本动作通过，但不能宣称“动态步行已通过”。64 帧协调姿态的 MuJoCo 逆动力学峰值为左踝 pitch `1.414238 N·m`，是连续设计值 `1.255251 N·m` 的 112.67%，但为额定值 `1.569064 N·m` 的 90.13%。因此：

- 可以继续做约束 RL 训练；
- reward/termination 必须加入峰值/RMS 扭矩、电流、温升、足底接触和跌倒约束；
- 真机步行必须先做单舵机首件、台架热/电流测试和慢速吊架测试。

## 仍未被数字模型证明的事项

- STS3250-C001 实物 4×M2、25T、后支撑与当前支架首件配合；
- CoreS3 U 型支架、躯干螺母板、USB-C 和线束弯曲半径；
- 18 个总线 ID、机械零位、方向符号；
- 实物逐 link 质量、质心、惯量；
- 打印材料、层向、嵌件、螺钉长度和承载件加工工艺。

因此本仓库是“数字 RL 可用、实物首件 HOLD”，不是“下载后整机一次打印即可装好”的工厂签署。
