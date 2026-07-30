# Zeroth-01 最小可靠圆润版 CAD/打印设计简报

## 设计目标

在不改变 Zeroth-01 原始 17-link 运动机构的前提下，完成更圆润的头部与身体、加厚脚底、电子件包装和可追溯颜色标识。

优先级：

1. 原关节连接、轴线、零位和守护运动范围不变。
2. 新件可拆卸、非承载，不阻断未来更换外观。
3. 外壳圆润但保留相机/ToF 视场、通风、线束和维修空间。
4. CAD、SolidWorks、URDF/MJCF 和文档使用同一坐标与身份映射。

## 选定几何

- 头部：三轴椭球壳体，不再用圆角长方体模拟。
- 面罩：复合曲面 visor，内部容纳平面双眼屏、相机和 ToF。
- 胸部/骨盆：原机构外侧增加大圆角壳，不修改原 link 网格。
- 脚底：左右加厚、圆角、保持原踝关节轴和接地点语义。
- 关节颜色：20 mm × 1.8 mm 圆片，仅作 SolidWorks 审阅标记。

## 电子件几何层级

- Waveshare 双眼屏：使用供应商精确 STEP。
- Camera Module 3 Wide：供应商精确 STEP 归档；SolidWorks 自动总装使用 `25 × 23.862 × 11.4 mm` 官方测得简化包络，避免复杂 STEP 导入卡死。
- VL53L5CX：传感器选型确定，当前 `12 × 10 × 3 mm` 载板包络是假设。
- IMU、主控、电池：RL/包装包络，型号未冻结。

电子件包络不可用于开模或 PCB 下单，除非 `confidence` 为供应商精确尺寸且接口已复核。

## CAD 产物

- 参数化源：`cad/round_v1/`
- STEP 零件：`generated/cad/round_v1/parts/`
- 圆润外壳/电子件审阅 STEP：
  `generated/cad/round_v1/ZEROTH01_ROUND_V2_COSMETIC_ELECTRONICS_ASSEMBLY.step`
- SolidWorks 原生 B-Rep 零件：
  `generated/solidworks/round_v1/parts/`
- SolidWorks 总装：
  `generated/solidworks/round_v1/OPEN_FIRST_ZEROTH01_ROUND_V2_MINIMAL_COSMETIC.SLDASM`
- 最终打印 STL：
  `generated/print/round_v1/final/`

审阅 STEP 只包含圆润/电子叠加件，不是完整原始机械机构；完整机构请在 SolidWorks 总装、URDF 或 MJCF 中查看。

## 坐标和单位

- CAD/STEP/STL：mm。
- URDF/MJCF/配置：m、kg、rad。
- 机器人前方：`-Y`。
- 上方：`+Z`。
- 左侧：`+X`。
- 电子件父 link：Torso。

## 接受标准

- 原始 17 link 和 16 转动关节变换不变。
- SolidWorks 总装 51 组件：17 原 link + 18 叠加件 + 16 非物理标记。
- 新增替代舵机、笼体、输出 hub：均为 0。
- 头、胸、骨盆、脚底外壳不进入 URDF 的关节树。
- 质量/惯量只在相应 link 上聚合一次。
- URDF/MJCF 的 motion/collision gate 通过。
- STL 水密、无非流形边，STEP/STL 体积误差 ≤0.5%。

## 制造前未决项

- 原骨架与外壳的定位柱、螺孔、嵌件和公差。
- 头部视窗材料、透光率、反射和散热。
- 电池/主控/IMU/ToF 载板和连接器。
- 线束路径、应变消除和关节全范围扫掠。
- 脚底材料、压力传感器载荷路径和防滑层。
- 真实材料、切片参数、强度、蠕变和跌落测试。

未决项关闭前，本 revision 的定位是“RL 机械基线 + 外壳试装设计”，不是可直接量产的完整机器人 CAD。
