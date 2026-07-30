# Zeroth-01 Round V3 CAD / 运动审查简报

## 当前权威路线

保留已装配且完成离散运动/碰撞门禁的 Zeroth-01 17-link、16 运动关节机构，只做可逆的最小外观改动：

- 白色大圆角胸壳与骨盆壳；
- 加厚、圆角、防滑面预留的左右脚底；
- Poppy Eva 开源头部拓扑衍生的白色蛋形前后壳；
- 两个小型实心耳片；
- 连续黑色圆角屏幕面板；
- 额头相机与 ToF 光学窗；
- 随手臂 link 运动的圆角上臂/前臂套与固定式 Q 版连指手掌；
- 一个独立蓝色 STS3250-C001 尺寸诊断零件，在 16 个 canonical joint 位置复用。

完整决策与制造边界见：

- `DESIGN_LEDGER.md`
- `ROUND_V3_WHITE_EVA_BLUE_SERVO_BRIEF_zh.md`
- `ASSEMBLY_GUIDE_zh.md`
- `PRINT_AND_ASSEMBLY_READINESS_zh.md`

## 权威输入与来源

- Zeroth-01 官方 URDF/网格：link/joint 树、frame、轴、质量、惯量和限位。
- 用户提供的 STS3250-C001 尺寸图与 FEETECH 当前规格：执行器元数据及受控外包络。
- Poppy Eva head design，固定 commit `844654a0b29fb771c23b7400997d1de3d42e0e2e`，CC BY-SA 4.0：头部拓扑参考。
- Waveshare 4.3-inch DSI/QLED 800×480：显示方案参考；当前采用 `105.5 × 8 × 67.2 mm` 受控包络，不冒充供应商精确 STEP。
- Raspberry Pi Camera Module 3 Wide：`25 × 23.862 × 11.4 mm` 测得包络。
- VL53L5CX：传感器已选，`12 × 10 × 3 mm` 载板仍是假设。

历史 step.parts 文件的 STEP 内部产品名是 `ST-3235M-20211119-A_ASM`，不是当前 `STS3250-C001`，且输出轴为局部 `+Y`；该文件已隔离，不作为安装依据。

## 坐标、单位与质量合同

- CAD / STEP / STL：mm。
- URDF / MJCF / 配置：m、kg、rad。
- 机器人前方：`-Y`；上方：`+Z`；左侧：`+X`。
- 原始聚合 link 惯量已经包含内部机构/舵机质量，不再重复加入 `16 × 74.5 g`。
- 蓝色 STS3250 诊断件不进入 URDF visual、collision、mass 或 inertia。

## 当前验证结果

- SolidWorks 总装：`57 = 17` 原 link `+ 24` 外观/电子叠加件 `+ 16` 蓝色诊断舵机实例。
- 蓝色舵机只引用一个独立 `.SLDPRT`；旧彩色关节圆片数量为 `0`。
- 16 个舵机局部 `+Z` 输出轴与 canonical joint 轴对齐：原点误差 `0 mm`，最大轴角误差 `< 0.000002°`。
- 原机构 48 点 SolidWorks FK / parent-output 传动语义证据全部 PASS。
- MuJoCo：16 关节轴向采样、100,000 随机姿态、65,536 边界组合及 1,000 步 smoke test 均 PASS。
- URDF / MJCF 名义总质量：`4.997342616724 kg`。
- 15 个当前打印候选网格：水密、绕向一致、无边界/非流形边，STEP/STL 体积误差不大于 `0.5%`。
- 6 个手臂套/手掌与原 link 保守凸包的几何检查交集均为 `0 mm³`。

## 未证明事项

当前交付可用于 RL、仿真、SolidWorks 结构审阅和外壳试装，但不能宣称“打印后即可直接拼装行走”。仍缺：

- 可追溯的原生承载 CAD、真实舵机安装面/安装耳、舵盘、轴承和紧固件；
- 外壳定位柱、嵌件、螺纹、公差栈与完整装配工序；
- 线束全范围扫掠、连接器、散热、跌落和疲劳验证；
- 屏幕/相机/ToF 实物试装与光学、热、EMI 校验；
- STS3250 实物零位、方向、背隙、电流、温升和扭矩-速度曲线标定。
