# Zeroth-01 CAD / 运动审查简报

## 当前权威路线

保留已装配、已通过离散运动/碰撞门禁的 Zeroth-01 17-link 机构，不重新布置 16 个舵机，不新增舵机笼、输出 hub 或齿轮。圆润头/胸/骨盆、加厚脚底和电子件均为可拆、非承载叠加层。

完整当前设计见：

- `CAD_PRINTABLE_ROUND_BRIEF_zh.md`
- `DESIGN_LEDGER.md`
- `ASSEMBLY_GUIDE_zh.md`

## 权威输入

- `zeroth-sim` 几何兼容 revision：link/joint 树、frame、轴、质量、惯量和限位。
- 17 个公开源 link STL：原机构几何。
- 官方 STS3250-C001 页面/尺寸图：执行器规格元数据。
- Waveshare 双眼屏与 Raspberry Pi Camera Module 3 Wide 供应商 CAD。

历史 step.parts 文件虽然目录名为 STS3250，但 STEP 内部产品名是 `ST-3235M-20211119-A_ASM`，输出轴为 +Y；它已隔离，不能作为 C001 安装模型。

## 坐标和质量合同

- CAD/打印：mm。
- URDF/MJCF：m、kg、rad。
- 前方 `-Y`，上方 `+Z`，左侧 `+X`。
- 原始聚合 link 惯量已经包含内部机构/舵机，不重复加入 16×74.5 g。
- S01–S16 彩色圆片无质量、无碰撞、无传动语义。

## 当前验证

- SolidWorks：17 原 link + 18 外观/电子叠加件 + 16 非物理标识 = 51 组件。
- 替代舵机、笼体、输出 hub：0。
- 100,000 随机姿态与 65,536 边界组合：0 个自碰撞样本。
- URDF/MJCF 质量：`4.586857125474 kg`，一致。
- 11 个外壳/脚底 STL 网格：通过；承载路径：未放行。

## 未证明

没有原生承载 CAD、紧固件、轴承、线束、公差、材料工艺和实物 STS3250 标定，因此当前可用于 RL、SolidWorks 审阅和外壳试装，不能宣称打印后直接拼装行走。
