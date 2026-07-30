# Zeroth-01 Round V3 可打印外观设计简报

## 设计目标

在不改变 Zeroth-01 原始 17-link / 16DoF 机构的前提下，完成更圆润的白色外观、加厚脚底、带小耳朵的屏幕头、加粗圆润手臂、固定式 Q 版手掌和可追溯电子器件包络。所有新增外观件均为可拆卸、非承载件，不阻断未来替换外壳。

优先级：

1. 原关节连接、轴线、零位、限位和承载路径不变。
2. 正常白色总装保持整洁；内部电子件和 16 个蓝色舵机只在 `OPEN_FIRST` 透视总装显示。
3. 头壳保留屏幕、相机、ToF 视场、散热、线束和维护空间。
4. CAD、SolidWorks、URDF/MJCF 与文档使用同一坐标和部件身份映射。

## 选定几何

- 头部：Poppy Eva 开源拓扑衍生的白色蛋形前后壳，围绕 Zeroth-01 肩部避让重新参数化。
- 面部：连续黑色圆角屏幕面板与青色屏幕 UI；两个小型耳片，无凸眼、无嘴套。
- 胸部/骨盆：在原聚合 link 外增加大圆角白壳，不修改原 link 网格。
- 手臂/手掌：左右各一件上臂套、一件前臂套和一件固定连指手掌，共 6 件；它们跟随原 link，不增加关节。
- 脚底：左右加厚圆角鞋底，保持原踝关节轴和接地点语义。
- 舵机审图：`45.22 × 24.72 × 36.5 mm` 蓝色 STS3250-C001 受控包络，一个 `.SLDPRT` 复用 16 次；不是安装签字模型。

## 电子器件几何层级

| 器件 | 当前包络/状态 | 用途 |
|---|---|---|
| Waveshare 4.3-inch DSI/QLED | `105.5 × 8 × 67.2 mm`，供应商型号已选、包络受控 | 头部彩色屏 |
| Camera Module 3 Wide | `25 × 23.862 × 11.4 mm`，测得包络 | 额头相机 |
| VL53L5CX 载板 | `12 × 10 × 3 mm`，假设 | 额头 ToF |
| IMU / 主控 / 电池 | 受控包装包络，具体料号未冻结 | RL 与整机布局 |

除非 `confidence` 已升级为“供应商精确且接口复核”，电子包络不得直接用于开模、PCB 或夹具下单。

## CAD 产物

- 参数化源：`cad/round_v1/`
- V3 完整审阅 STEP：`generated/cad/round_v1/ZEROTH01_ROUND_V3_WHITE_EVA_16_BLUE_SERVOS_ASSEMBLY.step`
- V3 STEP 零件：`generated/cad/round_v1/parts/`
- 正常白色 SolidWorks 总装：`generated/solidworks/round_v1/ZEROTH01_ROUND_V3_WHITE_EXTERIOR.SLDASM`
- 透视/16 舵机 SolidWorks 总装：`generated/solidworks/round_v1/OPEN_FIRST_ZEROTH01_ROUND_V3_WHITE_EVA_16_BLUE_SERVOS_XRAY.SLDASM`
- 独立蓝色舵机零件：`generated/solidworks/round_v1/parts/ZEROTH01_STS3250_C001_BLUE_DIAGNOSTIC.SLDPRT`
- 最终打印候选 STL：`generated/print/round_v1/final/`

## 当前 15 个打印候选件

1. 胸前壳
2. 胸后壳
3. V3 头前壳
4. V3 头后壳
5. 骨盆前壳
6. 骨盆后壳
7. V3 黑色屏幕框/面罩
8. 左脚底
9. 右脚底
10. 左上臂套
11. 右上臂套
12. 左前臂套
13. 右前臂套
14. 左 Q 版连指手掌
15. 右 Q 版连指手掌

屏幕 UI、电子包络、蓝色舵机诊断件和原聚合 link 不属于这 15 个外壳打印件。Q 版手掌固定在原 hand link 上，不是灵巧手，也不承担抓取载荷。

## 接受标准

- 原始 17 link 和 16 个运动关节的变换不变。
- SolidWorks `57 = 17 + 24 + 16`，旧彩色关节标记为 `0`。
- 新增替代舵机、承载笼体、输出 hub 数量均为 `0`。
- 外壳不进入 URDF 关节树，质量/惯量只聚合到对应 link 一次。
- URDF/MuJoCo motion、mass、collision 与 smoke gate 通过。
- STL 水密、绕向一致、无非流形边，STEP/STL 体积误差不大于 `0.5%`。
- 6 个手臂套/手掌对原 link 保守凸包的交集为 `0 mm³`。

## 制造前必须补齐

- 原骨架与外壳的定位柱、螺孔、热熔嵌件、公差和可达工具空间；
- 头部屏幕夹具、透明窗材料、散热和连接器出线；
- 电池、BMS、主控、IMU、ToF 的冻结料号与载板；
- 线束路径、应变消除和关节全范围扫掠；
- 脚底压力传感器载荷路径、防滑层和材料；
- 指定打印机/喷嘴/材料/层高后完成切片与实物试装。

因此本 revision 是“RL 机械基线 + 外壳试装设计”，不是量产或直接行走装配包。
