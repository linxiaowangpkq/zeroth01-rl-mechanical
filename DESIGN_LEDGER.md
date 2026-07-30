# Zeroth-01 Round v3 / RL 机械设计账本

本文件只记录会改变机械、RL、采购或验收结论的事实。所有长度、质量、角度分别使用 m、kg、rad；CAD/打印件源文件使用 mm。

## 机器人元数据

- 机器人名：`zeroth01_rl_round_v3_white_eva_16dof`
- 目标消费者：MuJoCo、Isaac/Genesis 转换、RViz/robot_state_publisher、实机 Feetech 总线部署。
- 坐标：保留官方 Zeroth-01 URDF 的右手系、父子关系、关节原点、RPY 和轴符号；不从外观 CAD 反推运动学。
- 网格：官方 STL 与新增外观 STL 均按 mm 导出，URDF 使用 `scale="0.001 0.001 0.001"`。
- 源：官方 Zeroth-01 URDF/网格、用户提供 STS3250-C001 尺寸图、FEETECH 当前产品尺寸、Poppy Eva 开源头部拓扑、Waveshare 官方 4.3-inch DSI/QLED 资料。

## 可审计的逐项账本

以下文件共同构成完整的逐 link / joint 账本，避免在本文复制一份会漂移的表：

- `generated/urdf/zeroth01_rl_round_v1.urdf`：完整 link/joint、visual、collision、inertial、sensor-frame 树。
- `reports/link_inertial_audit.csv`：18 个官方/基础物理 link 的质量、COM、惯性张量、来源与正定性。
- `generated/config/round_v1_mass_properties.json`：打印外壳与电子件叠加后的逐 link 组合质量、COM、惯性和置信度。
- `reports/joint_servo_frames.csv`：16 个运动关节的父子 link、父坐标系原点/RPY、关节坐标系正轴、世界轴、限制、候选总线 ID 和待标定项。
- `reports/rl_audited_joint_limits.csv`：16 个关节的源限制、审计限制和取得交集的方法。
- `reports/mesh_frame_audit.csv`：每个官方 visual mesh 的 link-local/world-baked 判定与 COM 对照。
- `config/round_v1_electronics_layout_source.json`：显示屏、摄像头、ToF、IMU、计算单元和电池的尺寸、位置、质量来源与硬件覆盖项。

正运动定义为围绕 `joint_servo_frames.csv` 中 `positive_axis_joint_frame` 的右手正转；`urdf_to_servo_direction_sign` 与实机零位必须通过低扭矩点动标定，不能由 CAD 猜测。

## D-001：冻结原 17-link / 16-DoF 机构

- 状态：接受。
- 决策：保留原始 link 网格、16 根关节轴、父子关系、零位和审计运动范围。
- 原因：该运动树已经通过离散碰撞与动力学有限性门禁；重排舵机会同时改变安装、轴线、惯量和训练模型。
- 证据：`reports/mujoco_round_v1_gate.json`、`reports/solidworks_round_v1_gate.json`。

## D-002：白色外观只做可拆卸、非承载叠加

- 圆角胸/骨盆壳、带小耳朵的 Poppy-Eva 衍生蛋形头壳、圆润手臂套、固定 Q 版手掌和加厚鞋底不切削原骨架，不改变关节。
- 十五件选定 STL 必须通过闭合、非流形边与 STEP 体积一致性门禁；结果见 `reports/round_v1_print_mesh_gate.json`。
- 六件手臂套/手掌必须对官方 link 的保守凸包保持 `0 mm³` 交集；结果见 `reports/round_v3_arm_fit_gate.json`。
- 这些输出是外观/试装/脚底接触原型，不是承载支架、轴承座或量产紧固件签字。

## D-003：头部方案

- 开源拓扑参考：`poppy-project/Poppy-eva-head-design`，CC BY-SA 4.0；本项目重新参数化以匹配 Zeroth-01 肩部避让，而不是直接缩放 STL。
- 外观：白色蛋形前后壳、两个小型实心耳片、连续黑色圆角屏幕面板，无凸眼和嘴套。
- 显示：Waveshare 4.3-inch DSI/QLED，800×480；当前使用 105.5×8×67.2 mm 受控包络，安装中心 `(0, -0.025, 0.105) m`。
- 摄像头/ToF：位于屏幕上方额头区域；外观只露光学窗口，模块包络位于壳内。
- 未关闭项：显示厚度、排线弯曲半径、连接器、夹具孔位和打印收缩必须用实物试装券确认。

## D-004：蓝色 STS3250 仅用于 SolidWorks 快速审图

- 唯一零件：`ZEROTH01_STS3250_C001_BLUE_DIAGNOSTIC.SLDPRT`。
- 总包络：45.22 × 24.72 × 36.5 mm；35 mm 本体加两侧诊断凸台。
- 总装复用：S01–S16 共 16 个实例，每个实例的本地 +Z 输出轴与 canonical joint 正轴共线，轴心重合。
- 物理语义：只跟随父侧关节坐标系；不新增 URDF link，不重复增加舵机质量，不参与碰撞，不构成安装/干涉签字。
- 原因：现有官方/开源聚合 link 网格已经包含内部结构，无法从中可靠分离真实舵机壳和紧固件。
- 隔离文件：旧供应商 STEP 内部标识 `ST-3235M-20211119-A_ASM`、轴向 +Y、尺寸不符，禁止当作 C001 精确安装模型。

## D-005：质量与惯性

- 官方 link 惯量是“含原结构/原舵机的聚合 link”基线；禁止再叠加 16×舵机质量。
- 白色打印件按 PETG 1.27 g/cm³ 的 CAD 体积计算名义质量。
- 显示/摄像头等有供应商质量时使用供应商值；其余电子件采用明确的 RL 假设范围。
- 最终样机必须逐 link 称重并通过摆锤/系统辨识更新 `generated/config/round_v1_mass_properties.json` 后，才允许称为硬件闭环惯量。

## D-006：验证边界

- 已证明：URDF 树、16 根轴映射、网格路径、惯量有限/正定、离散关节扫描、打印网格拓扑、蓝色审图件轴线对齐。
- 未证明：连续全空间无碰撞、线束扫掠、公差栈、螺纹/轴承寿命、跌落强度、热管理、STS3250 长期连续负载。
- 硬件 walking gate 保持关闭，直到完成实物装配、线束、零位/方向标定、双腿台架和热稳态测试。

## D-007：固定式 Q 版手掌

- 左右手掌是固定连指外观壳，随原 `Left_hand` / `Right_hand` link 运动。
- 不新增手指关节、执行器或抓取能力；RL 中只作为固定 visual/collision/质量叠加。
- 当前通过静态保守凸包避让和全身离散碰撞门禁，但不构成抓取、跌落或承载签字。

## 待关闭项目

1. 获取可追溯的 STS3250-C001 原生 CAD 或实物 CMM，补齐真实安装耳、连接器和公差。
2. 打印显示夹具试装券；确认屏幕厚度、排线出口与摄像头/ToF 光学遮挡。
3. 冻结 SBC、IMU、电池单体、BMS、稳压器、连接器和线束。
4. 逐 link 称重并更新 COM/惯性。
5. 对 16 个舵机执行总线扫描、低扭矩点动、方向、零位和软限位校准。
6. 完成静态站立、吊架单腿、双腿低速和热稳态测试后再开放实机行走。
