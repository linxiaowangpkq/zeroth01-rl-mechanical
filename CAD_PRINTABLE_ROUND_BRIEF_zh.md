# Zeroth-01 圆润外观与可打印性 CAD brief

## 目标

在不改变 Zeroth-01 的 18-link / 16-actuated-DoF 拓扑、关节轴线和已验证安全关节盒的前提下：

1. 在 SolidWorks 审核装配中加入真实 Feetech STS3250 STEP，而不是只用文字元数据代表舵机。
2. 生成可逆的圆润外观件：躯干/头部外壳、关节盖与左右厚鞋底。
3. 将现有米制、非流形 STL 转换为毫米制闭合网格，明确区分“展示代理”与“功能承力零件”。
4. 生成与厚鞋底一致的 URDF/MJCF 接触几何、质量和惯量数据，供 RL 训练。

## 坐标与单位

- CAD 与打印输出：mm。
- URDF/MJCF：m、kg、s、rad。
- 根坐标：沿用 `zeroth01_rl_ready.urdf`；不重定义关节 frame。
- STS3250 STEP 原点：按目录模型保留；其输出轴候选轴为 STEP 的 Z 轴，装配前必须与每个 URDF joint axis 对齐验证。

## 已冻结的输入

- 运动学/质量基线：`generated/urdf/zeroth01_rl_ready.urdf`
- 关节/舵机 frame：`reports/joint_servo_frames.csv`
- 原始视觉网格：`generated/urdf/meshes/*.stl`
- 舵机：`source_assets/vendor/sts3250/FEETECH_STS3250.step`
- 外观参考：用户提供的圆润奶油色机器人图片，仅提取圆角、深色关节圈与厚鞋底的设计语言。

## 外观参数

- 外壳名义壁厚：2.4 mm。
- 外壳与参考网格名义间隙：0.8 mm。
- 分件装配间隙：0.35 mm/side；需要按实际打印机重新标定。
- 关节动态 keep-out：从轴心起至少 3.0 mm 额外安全间隙。
- 鞋底：在现有脚底下方增加 8.0 mm；前后/左右各扩展约 5–6 mm。
- 外观圆角：主体 12–28 mm；鞋底 3–6 mm。
- 默认打印材料质量估算：PETG，密度 1.27 g/cm³；此值是 RL 质量预算假设，不是采购锁定。

## 结构边界

- 原始 17 个 STL 是 link 级聚合视觉网格，包含舵机/支架外表面，且不是独立的舵机、支架、紧固件 BOM。
- 将聚合网格修复成闭合 STL 只能得到静态展示/碰撞代理，不能把它自动变成功能承力零件或舵机安装座。
- 圆润外壳与鞋底必须可逆，不修改关节轴或不可逆地切削未来外壳接口。
- 普通 FDM 外壳允许；双足行走的主承力路径、舵机耳座、输出盘连接和轴承支撑不得仅凭本 CAD 包宣称结构安全。

## 输出

- `generated/cad/round_v1/ZEROTH01_ROUND_V1_ASSEMBLY.step`
- `generated/cad/round_v1/parts/*.step`
- `generated/print/round_v1/final/*.stl`
- `generated/urdf/zeroth01_rl_round_v1.urdf`
- `generated/mujoco/zeroth01_rl_round_v1.xml`
- `reports/printability_mesh_audit.csv`
- `generated/config/round_v1_mass_properties.json`
- `reports/round_v1_print_mesh_gate.json`
- `reports/mujoco_round_v1_gate.json`
- `reports/round_v1_servo_axis_alignment.csv`

## 验证门槛

- STS3250：STEP 校验和通过；实体数、包围盒和输出轴方向有记录。
- CAD：全部新增打印件为闭合正体积实体；STEP 与 STL 都以 mm 输出。
- 装配：16 个舵机输出轴与 URDF joint axis 同向或反向共线，轴心误差不超过 0.05 mm。
- 干涉：至少覆盖中立、全部单关节上下限、现有 30% guarded box 随机姿态；任何只验证视觉而未验证实体的部分不得标为通过。
- RL：URDF 生成时校验通过；MuJoCo 加载、单关节动态、站立接触与随机姿态门槛通过。
- 打印：只在已知真实打印机/喷嘴/材料/profile 后才能生成并声明可用 G-code。
