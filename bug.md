# v5 原始 16DoF 机械问题闭环

## 已修复并有数字证据

- 回滚错误的 18DoF/踝 roll 方案：当前仅 16 个原始 Zeroth-01 关节，未添加穿模踝舵机，也未缩短小腿。
- 删除夹爪、大手掌、球形手和方块腕端；左右仅保留镜像的两 M3 小型固定支撑。
- 删除黑色增高鞋底；替换为 9 mm 白色脚底，上端不超过原脚轮廓、接地端每向外扩 4 mm，即下宽上窄。
- 删除外露背板/后勤舱；计算板、电池/BMS、IMU 和线束包络全部归属躯干内部。
- 头部只在接受过的原 Zeroth 简单脸基础上，上下左右各扩 5 mm、加小圆角，并以隐藏 M3 螺母板无脖子直连；UnitV2 摄像头/麦克风为可拆件。
- 将 17 个原始承力 carrier 转换为正体积运动 link；原躯干的 26 个闭合区域保持不动，只参数化替换两个破损头部定位柱。当前 motion-link STEP 全部为有效正体积 B-Rep。
- 16 个蓝色舵机均采用 FEETECH STS3250-C001 采购精确 STEP，不再使用尺寸随意的方盒。
- 明确每个关节的壳体侧与输出侧归属：静止侧含采购舵机与轴向承力环，运动侧含 PCD14 四孔输出桥。
- 重做左右髋 yaw 安装：正体积躯干融合两组窄肋/螺柱，每侧 2×M2×8 固定后壳；输出侧四个 M3 套筒连接 U 形髋架；不再使用悬空桥板、整圆垫片或虚构背板。
- 左右 U 形髋架按原关节全限位做 0.30 mm 扫掠避让；右侧切除 55.316 mm³、左侧 55.373 mm³，17 个验证姿态最大剩余交叠均为 0，轴线和中立位不变。
- 当前 85 组件在中立位完成 3570/3570 对精确 B-Rep 检查：正体积干涉 0、Boolean 错误 0。
- URDF：16 关节、2.745759 kg、惯量/网格/限位门禁 PASS；MJCF 在 MuJoCo 3.3.7 编译 PASS，`nq=23`, `nv=22`, `nu=16`。

## 尚未关闭，禁止伪造 PASS

- 当前 SOLIDWORKS 进程无响应；新的 85 组件 assembly 尚未完成原生静态重建和 16 关节 × 上下限的 32 次真实 Motion。旧 native gate 的 manifest 哈希已过期，不能当作 v5 证据。
- 必须采购至少一只真实 STS3250，验证外壳、后盖 M2、输出 25T/PCD14、螺钉长度和工具空间。
- 必须打印髋 yaw 座/输出桥、腕端和脚底试片，验证间隙、层向、嵌件拉拔和重复拆装。
- 必须冻结计算板、电池/BMS、IMU、保险丝、急停、稳压器和线束后，才能对内部托架签字。
- 必须实测总线 ID、机械零位、方向、背隙、延迟、扭矩增益、电流、温升、电压降、分 link 质量/COM/全惯量，并通过 SysID 更新 RL 模型。
- 必须带真实线束完成无电全行程、悬吊单关节、镜像关节、限位慢扫和动态策略测试。

当前结论：`digital_rl_baseline = PASS`；`native_solidworks_motion = HOLD`；`physical_first_article = HOLD`。

## 2026-08-09 独立 RL 运行时审计：新增未关闭问题

审计基线：分支 `codex/zeroth01-v5-16dof-solidworks-motion`，commit
`626d716bffb6688138e545e1b6bff5ae7d5f197b`；MuJoCo 3.3.7、JAX
0.7.2、RTX 4070 Laptop（CUDA）。原始 MJCF 可编译为 `nq=23`,
`nv=22`, `nu=16`，质量为 `2.745758514949 kg`；`official_standing`
在 `1.2552512 N·m` 限幅 PD 下连续仿真 60 s 无 NaN，底盘高度范围
`0.416453–0.416796 m`，因此下面是交付契约问题，不是“质量过大导致必然无法站立”。

### BUG-RL-V5-001（P0）：actuator 顺序与编译后的 joint/qpos 顺序不一致

编译后的 16 关节顺序为：

```text
left_shoulder_yaw, left_shoulder_pitch, left_elbow_yaw,
right_shoulder_yaw, right_shoulder_pitch, right_elbow_yaw,
left_hip_yaw, left_hip_roll, left_hip_pitch, left_knee_pitch,
left_ankle_pitch, right_hip_yaw, right_hip_roll, right_hip_pitch,
right_knee_pitch, right_ankle_pitch
```

MJCF actuator 顺序却为：

```text
left_shoulder_yaw, right_shoulder_yaw, left_hip_yaw, right_hip_yaw,
left_shoulder_pitch, right_shoulder_pitch, left_hip_roll, right_hip_roll,
left_hip_pitch, right_hip_pitch, left_elbow_yaw, right_elbow_yaw,
left_knee_pitch, right_knee_pitch, left_ankle_pitch, right_ankle_pitch
```

任何把策略输出、`qpos[7:]` 和 `ctrl[:]` 当作同一索引契约的训练/部署程序，
都会静默控制错关节。修复要求：生成 MJCF 时按编译树中的 joint 顺序生成
actuator，或交付显式且自动校验的 `qpos_index <-> actuator_index` 映射；
checkpoint 必须记录所用顺序和源 MJCF SHA-256。

### BUG-RL-V5-002（P0）：`symmetric_crouch` keyframe 按错误顺序写入 qpos

当前 keyframe 在编译后的 joint 顺序中被解释为：

```text
left_hip_pitch=+0.18, left_knee_pitch=-0.18,
right_hip_roll=-0.36, right_hip_pitch=+0.36,
right_knee_pitch=-0.18, right_ankle_pitch=+0.18
```

其中 `right_hip_roll=-0.36`、`right_hip_pitch=+0.36`、
`right_knee_pitch=-0.18` 已超出各自限位。`mj_forward` 显示该姿态两脚无地面接触：
左脚底最低点 `z=3.406 mm`，右脚底最低点 `z=9.068 mm`，右脚中心横向漂到
`y=-125.4 mm`，不是对称蹲姿。根因是 `build_v5_mjcf.py` 用一套交错的
URDF/actuator 顺序拼接 keyframe，而 MuJoCo qpos 使用运动树深度优先顺序。

修复后的腿部 qpos（按编译 joint 顺序）应至少满足镜像结构：

```text
left:  hip_yaw=0, hip_roll=0, hip_pitch=+0.18,
       knee_pitch=-0.36, ankle_pitch=-0.18
right: hip_yaw=0, hip_roll=0, hip_pitch=-0.18,
       knee_pitch=+0.36, ankle_pitch=+0.18
```

同时应重新求解 base z，使两块鞋底实际接地，再由自动测试检查所有 qpos 均在限位内、
左右脚世界坐标镜像且存在双脚接触。

### BUG-RL-V5-003（P1）：足底 `front/rear` 名称与声明的世界 +X 前向相反

执行 `official_standing` 的世界坐标正运动学后，所有 `front_*` site 的 x 约为
`-50 mm`，所有 `rear_*` site 的 x 约为 `+26–27 mm`；而执行器配置声明
`X forward, Y left, Z up`。因此“世界前向定义”和“足底前/后传感器命名”至少有一项
相反，会污染前后足冲击、步态相位和滑移统计。修复要求：以 CAD 中真实脚尖/相机朝向
为权威，冻结世界前向；随后重命名 site 或修正根坐标变换，并添加
`front_site_world_x > rear_site_world_x`（若 +X 为前）的自动门禁。

### BUG-RL-V5-004（P2）：GitHub 分支 ZIP 不是完整离线交付，README 未说明 LFS

GitHub codeload 分支 ZIP 中 17 个 motion-link STEP 和 1 个 review GLB 是
132–134 字节的 Git LFS 指针。直接运行交付清单校验得到 `FAIL`：清单声明
`840848688` bytes，ZIP 实际只有 `20142756` bytes。应在 README 明确要求
`git lfs install && git lfs pull`，并让校验器识别 LFS pointer 后给出该根因；若目标是
“下载 ZIP 即离线复核”，则需提供包含 LFS 实体文件的 release archive。

在 BUG-RL-V5-001/002 修复前，不应继续宣称未经映射修正的 MJCF 可直接用于 16DoF
训练。允许的临时训练派生物只能重排 actuator、修正 keyframe/接地高度，且不得改变
几何、质量、惯量、碰撞体、关节轴或限位；派生过程和双 SHA-256 必须随 checkpoint
保存。

## 2026-08-09 16DoF PPO 步行门禁结果

在不改变几何、质量、惯量、碰撞体、关节轴、限位和 `1.2552512 N·m` 连续扭矩上限的
前提下，使用 RTX 4070 Laptop、128 个并行环境，将已通过站立门禁的 N12 checkpoint
继续训练至 PPO update 64。步行阶段将目标速度改为 `0.10 m/s`，收窄速度核并提高前进
奖励，同时加入扭矩、能耗、足底滑移和动作变化惩罚。训练全程无 NaN、穿地或仿真爆炸。

逐 checkpoint 的 4 s 门禁中，update 16–64 均存活，但全部未达到 `0.05 m/s` 的最低
平均前进速度。最快的 update 16 只有 `0.005699 m/s`，单脚支撑占比 `4.5%`；update 64
只有 `0.003462 m/s`，单脚支撑占比 `0%`、双脚支撑占比 `100%`。10 s 回放的平均速度
也只有 `0.001797 m/s`。结论是策略收敛到双脚站立局部最优，不能命名为 M（行走）或
Z（奔跑）checkpoint；因此本仓库只提交已通过门禁的 N12 站立 checkpoint。

### BUG-RL-V5-005（P1）：官方站姿把膝、踝四个关节放在单侧硬限位上

`official_standing` 将所有关节置零，但模型中的四个腿关节范围为：

```text
left_knee_pitch:   [-0.83285928, 0]
left_ankle_pitch:  [-0.66415928, 0]
right_knee_pitch:  [0, +0.83285928]
right_ankle_pitch: [0, +0.66415928]
```

也就是说，左右膝和左右踝在 RL 初始姿态中全部恰好位于硬限位边界；一半方向的探索会
立即被裁剪/产生限位接触，左右镜像噪声也不再对称。当前唯一声明的内侧蹲姿
`symmetric_crouch` 又因 BUG-RL-V5-002 的 qpos 排序错误而越界且不接地。模型因此没有
一个可直接用于步态学习、四个膝踝都留有双向裕量的有效中性姿态。这与本次 PPO 持续
回到双脚直立的现象一致，是需要 CAD/机械零位确认的模型约束问题；它不能仅靠继续增加
训练轮数解决。

修复要求：

1. 由 SolidWorks 运动装配确认四个零位是否真的对应实体硬挡；若不是，修正 URDF/MJCF
   的关节零位和 range，而不是只在训练代码中偏置。
2. 交付一个双脚真实接触、左右镜像、所有膝踝距两端限位至少 5 度的 gait-neutral
   keyframe，并记录它对应的实体舵机零位。
3. 自动门禁需验证 keyframe 不越界、双脚接触、左右世界坐标镜像，并在该姿态下对每个
   膝踝施加正负小动作，确认两个方向都存在可用行程。

目前没有证据支持“先减重”是修复方向：总质量 `2.745758514949 kg` 的原模型在 60 s
站立验证中的最大控制扭矩仅 `0.2361 N·m`，远低于 `1.2552512 N·m` 连续上限。应先修正
关节/actuator 映射、无效蹲姿和硬限位中性姿态，再重新训练；只有在动态步态产生后，
才能用峰值/RMS 扭矩、电流、温升和母线压降判断是否需要减重。

## 2026-08-09 生成器修复与复测状态

本节记录对上述独立审计的源头修复；历史问题和失败训练结果保留，不回写为虚假成功。

| 问题 | 状态 | 当前证据 |
|---|---|---|
| BUG-RL-V5-001 | **CLOSED** | `build_v5_mjcf.py` 从最终运动树推导编译顺序并据此重排 actuator；门禁逐项验证 `ctrl[i]`、`qpos[7+i]` 和 joint 一致，16/16 PASS。 |
| BUG-RL-V5-002 | **CLOSED** | `gait_neutral` / `symmetric_crouch` 按 joint 名生成，不再按列表拼接；base Z 由左右 sole box 联合求解，限位、双脚接地和镜像门禁 PASS。 |
| BUG-RL-V5-003 | **CLOSED** | 足底 site 按世界语义重命名；左右均满足 `front_x > rear_x`，同时修正并验证 `abs(medial_y) < abs(lateral_y)`。 |
| BUG-RL-V5-004 | **CLOSED（Git clone 路径）** | 中英文 README 已要求 `git lfs install/pull`；交付校验器会识别未实体化的 LFS pointer 并给出修复命令。GitHub 分支 ZIP 仍不承诺完整离线交付。 |
| BUG-RL-V5-005 | **DIGITAL CLOSED / PHYSICAL HOLD** | 新 `gait_neutral` 的四个膝踝距正负限位均大于 5°，双脚接地和镜像 PASS；`official_standing` 仅保留为原始标定/旧站立 checkpoint 姿态。实体零位、硬挡和编码器映射仍须 SolidWorks 运动装配与上电首件确认。 |

当前生成的 MJCF 仍为 16DoF、`nq=23`、`nv=22`、`nu=16`、
`2.745758514949 kg`；本轮没有修改几何、质量、COM、惯量、碰撞体、关节轴、关节
range 或 `1.2552512 N·m` 连续扭矩上限。机器可读结果位于
`reports/v5_original_16dof_solidworks_motion/mjcf_compile_gate.json`，模型契约位于
`cad/physical_mount_v5_original_16dof_solidworks_motion/RL_MODEL_CONTRACT.md`。生成门禁还从
`gait_neutral` 执行 5 s 限幅 PD 动力学 smoke：状态有限、底盘最低 `0.410028 m`、
最大控制 `0.504094 N·m`，PASS；这只证明复位姿态动态可用，不代表已经学会步行。

已有 `N12_standing` checkpoint 仍只证明站立，而且其 metadata 正确保留旧 canonical
MJCF 与训练时派生 MJCF 的双哈希；它不能改名为步行 checkpoint。下一轮步态训练必须直接
使用新 canonical MJCF、以 `gait_neutral` 复位，并在新 checkpoint metadata 中记录当前
MJCF SHA-256、上述 16 维顺序和训练源 commit。

## 2026-08-09 `c5cb2c7` 拉取后独立训练复测

已从远端 fast-forward 到 `c5cb2c7eb892579c273e68f0436e6285e0782b34`。最新 canonical
MJCF 通过独立编译与 60 s `gait_neutral` 限幅 PD 复测：`nq=23`、`nv=22`、`nu=16`、
质量 `2.745758514949 kg`，状态有限，base Z 为 `0.410037–0.411528 m`，最大控制扭矩
`0.497362 N·m`，双脚持续接触。未发现新的 URDF/MJCF 质量、惯量、碰撞、关节顺序、
reset 或动力学爆炸问题。

新 canonical 模型从头完成 12 次 PPO 更新（128 env，45,057 samples，RTX 4070 Laptop，
峰值显存约 5.65 GiB）。`N12_gait_neutral_standing` 在 12/12 个确定性 4 s 回合中存活，
平均 return `1277.595`，随机策略为 `290.932` 且 0/12 存活，故只通过 N（站立）门禁。

从该 N12 warm-start 的步行阶段继续到 update 64（253,954 samples）。13 个 checkpoint
各做 4 回合外部门禁：最快的 update 20 平均 `0.012538 m/s`，但只存活 3/4；全部存活的
update 24 只有 `0.003997 m/s`、单脚支撑 `0.75%`。update 44 以后单脚支撑降为 0，策略
再次收敛到双脚站立。没有 M checkpoint，因而未启动 Z 跑步训练。

### BUG-RL-V5-006（P1）：机器可读文件内嵌的 canonical MJCF SHA-256 不一致

commit `c5cb2c7` 中实际 Git blob、工作树文件和 delivery manifest 对 canonical MJCF 的
SHA-256 都是：

```text
1354289e37aabff35d3ceec91412df367e4e4e78d3d5185dcb3624cc38fba0b2
```

但以下文件仍声明另一个值
`145f5860e947c23d8ce8f27c9ac3f4ef48b5185b7f6852584c23d2232faaff18`：

- `reports/v5_original_16dof_solidworks_motion/mjcf_compile_gate.json`
- `generated/config/physical_mount_v5_original_16dof_solidworks_motion_rl_handoff.json`
- `generated/config/physical_mount_v5_original_16dof_solidworks_motion_actuator_layout.json`

这会让 checkpoint 无法唯一声明机械基线哈希。根因高度疑似生成报告时对工作树 CRLF 字节
求哈希，而提交后 Git 将 XML 规范化为 LF。修复要求：对 MJCF 强制 `.gitattributes`
`eol=lf`，在干净 checkout 中重新生成全部派生 JSON，并在 CI 中逐项验证所有内嵌哈希等于
delivery manifest 和实际 committed bytes；当前 checkpoint metadata 同时记录两值并将
`source_hash_contract_match` 标为 false，不隐藏不一致。

### RL-TRAIN-V5-001（P0，非 URDF 缺陷）：当前策略观测/奖励不足以打破双脚站立局部最优

当前 38 维 feed-forward actor 只使用 joint position/velocity、projected gravity 和 gyro；
虽然环境计算了 base linear velocity、feet contact 和 feet velocity，它们没有进入 actor。
`get_commands()` 为空、没有 gait phase/时间输入，curriculum 又恒为 1.0。策略因此既看不到
自身前进速度和触地状态，也没有显式交替摆腿相位；本次从有效 `gait_neutral` 重训仍回到
双脚站立，说明继续堆相同 PPO update 不是有效路径。

下一轮训练前应先修训练任务，而不是再次修改 URDF：将目标速度、base linear velocity、
双脚接触/速度和 `sin/cos` gait phase 纳入观测；用从站立到 `0.05 m/s` 再到 `0.10 m/s`
的速度 curriculum，并加入交替支撑、摆脚净空和落脚冲击奖励。修改会改变 observation
shape，必须从头训练新 checkpoint，旧 N12 只能作为模型/站立回归证据。跑步阶段必须继续
等待 M 门禁通过。
