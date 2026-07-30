# Zeroth-01 Round v3 打印与整机可装配性

## 结论

当前 15 个选定 STL 达到外观/试装件的网格门禁，但整机仍不是“打印后直接拼装并行走”的硬件成品。

原 17-link / 16DoF 机构的离散运动、URDF 与 MuJoCo 门禁已经通过；缺口在真实舵机安装、紧固件/轴承、屏幕夹具、线束、电气选型、公差和实物标定。

## 选定打印件

目录：`generated/print/round_v1/final/`

- `ZEROTH01_ROUND_V1_CHEST_FRONT/BACK.stl`
- `ZEROTH01_ROUND_V3_HEAD_FRONT/BACK.stl`
- `ZEROTH01_ROUND_V1_PELVIS_FRONT/BACK.stl`
- `ZEROTH01_ROUND_V3_VISOR.stl`
- `ZEROTH01_ROUND_V1_LEFT/RIGHT_SOLE.stl`
- `ZEROTH01_ROUND_V3_LEFT/RIGHT_UPPER_ARM_SLEEVE.stl`
- `ZEROTH01_ROUND_V3_LEFT/RIGHT_FOREARM_SLEEVE.stl`
- `ZEROTH01_ROUND_V3_LEFT/RIGHT_CHIBI_HAND.stl`

`ZEROTH01_ROUND_V3_FACE_UI` 是屏幕像素参考，不打印；蓝色 STS3250 诊断件也不打印。Q 版手掌是固定连指外壳，不是灵巧手或承载抓手。

## 网格门禁

`reports/round_v1_print_mesh_gate.json`：

- 15/15 水密、绕向一致。
- 边界边 0，非流形边 0。
- STEP/STL 体积误差均 ≤0.5%。
- 6/6 手臂套/手掌对原 link 保守凸包的几何交集为 `0 mm³`。
- 总门禁：`PASS`。

尚未指定打印机/材料/喷嘴/层高/支撑，因此切片门禁保持 `BLOCKED_EXPLICIT_PROFILE_REQUIRED`。

## 名义质量

- 打印叠加件 CAD/PETG 名义质量：`1.234870788525 kg`。
- 电子模块名义质量：`0.667 kg`。
- URDF/MuJoCo 名义总质量：`4.997342616724 kg`。

这些数值供 RL 初始域随机化；最终打印件和整机必须逐 link 称重并更新 COM/惯性。

## 建议试打顺序

1. 4.3 屏夹具/排线出口试装券。
2. 左右加厚脚底，检查踝部扫掠和压力位。
3. 前头壳 + 黑色屏幕面板，实装屏/相机/ToF 假板。
4. 后胸壳，验证电池/主控托盘、散热和维修开口。
5. 左右手臂套和 Q 版手掌，先做单侧低填充试装再成对打印。
6. 最后打印其余外壳。

发现干涉时修改可拆外壳或夹具，不修改冻结的原机构运动树。

## 门禁状态

| 门禁 | 状态 |
|---|---|
| URDF/MuJoCo 16DoF、1000 步有限性与质量一致性 | PASS |
| 16 个蓝色审图件轴线/轴心映射 | PASS |
| 15 个外观 STL 网格 | PASS |
| 6 个手臂套/手掌静态包络检查 | PASS |
| SolidWorks v3 总装/截图/Motion GIF | 见 `reports/solidworks_round_v1_gate.json` |
| 指定打印机切片 | BLOCKED |
| C001 精确安装、紧固件与轴承工程图 | BLOCKED |
| 屏幕夹具、线束扫掠、散热 | BLOCKED |
| ID/方向/零位/背隙/温升标定 | BLOCKED |
| 实物静态、吊架单腿、双腿低速 | BLOCKED |
| 无保护真实步行 | NOT AUTHORIZED |
