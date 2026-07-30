# Zeroth-01 白色 Eva 风格 16DoF RL 机械交付

本版保留已经验证的 Zeroth-01 17-link / 16 运动关节机构，只做可逆的最小外观改动：白色圆润身体、加厚脚底、带小耳朵的 Poppy-Eva 衍生屏幕头、圆润加粗手臂套和固定式 Q 版手掌。

## 先打开这些文件

| 用途 | 文件 |
|---|---|
| 白色不透明 SolidWorks 总装 | `generated/solidworks/round_v1/ZEROTH01_ROUND_V3_WHITE_EXTERIOR.SLDASM` |
| 打开即见 16 电机的 X-ray 总装 | `generated/solidworks/round_v1/OPEN_FIRST_ZEROTH01_ROUND_V3_WHITE_EVA_16_BLUE_SERVOS_XRAY.SLDASM` |
| 单独蓝色舵机零件 | `generated/solidworks/round_v1/parts/ZEROTH01_STS3250_C001_BLUE_DIAGNOSTIC.SLDPRT` |
| STEP 审图总装 | `generated/cad/round_v1/ZEROTH01_ROUND_V3_WHITE_EVA_16_BLUE_SERVOS_ASSEMBLY.step` |
| RL URDF | `generated/urdf/zeroth01_rl_round_v1.urdf` |
| MuJoCo | `generated/mujoco/zeroth01_rl_round_v1.xml` |
| RL 一句话提示词 | `one-seq.md` |

## 本版改变

- 将之前夸张的熊耳方案改成两个小型实心耳片，并保留白色蛋形前后头壳。
- 使用一体圆角黑屏和屏幕内青色眼睛，不再使用凸眼或嘴套实体。
- 选择 Waveshare 4.3-inch DSI/QLED 800×480 受控包络；摄像头和 ToF 位于额头光学窗之后。
- 保留圆角胸/骨盆与 8 mm 加厚脚底。
- 新增可拆卸圆角上臂套、前臂套和固定式 Q 版连指手掌；手掌不是灵巧手。
- 删除旧彩色关节圆片；生成一个独立蓝色 STS3250-C001 `.SLDPRT`，在 S01–S16 复用 16 次。

蓝色舵机审图件总包络严格为 `45.22 × 24.72 × 36.5 mm`。每个实例的本地 +Z 输出轴与 canonical URDF 关节轴共线，轴心与关节原点重合。

## 必须理解的语义

16 个蓝色舵机是可见性叠加件，不是 16 个新增机器人刚体；它们不进入 URDF visual/collision/inertial，也不重复增加质量。原始聚合 link 网格仍是机械基线。

蓝色件不能证明已经有可制造的舵机安装：C001 精确安装耳、连接器避让、紧固件、轴承、线束、公差与真实干涉仍需可追溯原生 CAD 或实物测量。

## 当前验证结果

- URDF：26 links、25 joints、16 个运动关节。
- MuJoCo：16 actuators、8 sensors、1 个头部相机。
- 名义总质量：`4.997342616724 kg`。
- MuJoCo 1000 步有限性/质量一致性烟雾测试：`PASS`。
- 15 个选定打印网格：水密、绕向一致、0 边界边、0 非流形边、STEP/STL 体积误差 ≤0.5%：`PASS`。
- 6 个手臂套/手掌对原 link 保守凸包的安装检查：交集均为 `0 mm³`，`PASS`。
- 100,000 个随机姿态与全部 65,536 个关节极限角组合：报告自碰撞样本为 0。
- 16 个蓝色舵机轴心误差 `0 mm`，轴线误差小于 `0.000002°`：`PASS`。

边界与未决假设见 `DESIGN_LEDGER.md` 和 `reports/`。

## 开源头部来源

头部拓扑参考官方 [Poppy Eva head design](https://github.com/poppy-project/Poppy-eva-head-design)，固定 commit `844654a0b29fb771c23b7400997d1de3d42e0e2e`，许可证 CC BY-SA 4.0。本项目围绕 Zeroth-01 肩部避让重新参数化，没有直接缩放原 STL。

屏幕参考为官方 [Waveshare 4.3inch DSI QLED](https://www.waveshare.com/product/4.3inch-dsi-qled.htm)。

![带小耳朵和 Q 版手掌的白色 Eva 风格总装](snapshots/solidworks/round_v1/zeroth01_round_v3_white_front.png)

![16 个蓝色舵机](snapshots/solidworks/round_v1/zeroth01_round_v3_16_blue_servos_annotated_front.png)

![运动检查](snapshots/solidworks/round_v1/zeroth01_round_v3_16_blue_servos_motion.gif)

## 实机门禁

RL 初始连续扭矩参考使用 `1.2552512 N·m`，禁止把堵转扭矩作为 PPO 连续动作上限。Q 版手掌是固定外观壳，不具备抓取自由度。打印/屏幕试装、电子件冻结、逐 link 称重、舵机 ID/零位/方向标定、线束扫掠、双腿台架和热测试完成前，实机 walking gate 保持关闭。
