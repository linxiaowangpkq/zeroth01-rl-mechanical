# 已否决实验：替代舵机 / 笼体 / 输出 hub 一体化路线

## 状态

`REJECTED_NOT_SELECTED`

该实验曾尝试把 16 个替代 STS3250、parent-side 笼体和 child-side 输出 hub 叠加到公开 Zeroth-01 聚合网格上。它不属于当前交付，不得用于 RL、制造或安装判断。

## 否决根因

1. step.parts 文件内部标识为 `ST-3235M-20211119-A_ASM`，不是当前 `STS3250-C001`。
2. 该 STEP 的输出轴为局部 `+Y`，旧实验按 `+Z` 放置，造成舵机、孔位、齿轮/圆盘冲突。
3. 公开 17-link 网格已经聚合原机构，但缺少可追溯的舵机安装面、轴承、舵盘、紧固件和公差；在其表面补画笼体不能证明真实反力路径。
4. 新增 16 个舵机实体会产生重复质量/惯量风险，并破坏已验证的最小机械基线。

## 当前替代方案

- 保留原 17-link 连接、轴线、零位、限位和承载路径。
- 删除旧的 16 个彩色关节圆片。
- 依据用户尺寸图生成一个蓝色 `ZEROTH01_STS3250_C001_BLUE_DIAGNOSTIC.SLDPRT`，包络 `45.22 × 24.72 × 36.5 mm`。
- 同一个蓝色零件在 S01–S16 的 canonical parent-side joint frame 复用 16 次，局部 `+Z` 与关节轴对齐。
- 正常白色总装隐藏这些诊断件；`OPEN_FIRST` 透视总装显示全部 16 个，并显示各色电子器件。
- 小耳朵、圆润手臂套和固定 Q 版手掌均作为可拆外观跟随原 link，不用来修补舵机安装或承载路径。
- 蓝色件不进入 URDF/MJCF 的 visual、collision、mass 或 inertia，也不代表物理安装已通过。
- STS3250 的真实性能规格进入执行器元数据；真实安装仍待可追溯原生 CAD 或实物 CMM/试装。

当前设计与门禁见 `DESIGN_LEDGER.md`、`README_zh.md` 和 `reports/solidworks_round_v1_gate.json`。
