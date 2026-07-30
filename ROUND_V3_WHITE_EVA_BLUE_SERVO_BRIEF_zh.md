# Round v3 白色 Eva 头与蓝色舵机审图说明

## 选择

- 机构：冻结原 Zeroth-01 17-link / 16DoF 机构。
- 头部：Poppy Eva 开源拓扑衍生的白色蛋形前后壳、两个小型实心耳片和一体黑色圆角屏。
- 手臂：可拆卸圆角上臂/前臂套与固定式 Q 版连指手掌；不新增自由度。
- 屏幕：Waveshare 4.3-inch DSI/QLED，800×480，受控包络 105.5×8×67.2 mm。
- 舵机：一个蓝色 `ZEROTH01_STS3250_C001_BLUE_DIAGNOSTIC.SLDPRT`，总包络 45.22×24.72×36.5 mm，在 16 个 canonical joint 父侧坐标系复用。

## 为什么这是最小可靠改动

外观、屏幕、手臂套、Q 版手掌和蓝色审图件均不改变 URDF 运动树。正常总装用于看白色外观；X-ray 总装用于一次看清 S01–S16，特别是每侧肩部 pitch/yaw 两个电机。

## 来源

- Poppy Eva：<https://github.com/poppy-project/Poppy-eva-head-design>
- 固定 commit：`844654a0b29fb771c23b7400997d1de3d42e0e2e`
- 许可证：CC BY-SA 4.0
- Waveshare：<https://www.waveshare.com/product/4.3inch-dsi-qled.htm>

项目参数化头壳不是原 Poppy STL 的直接缩放；它围绕 Zeroth-01 肩部轴和显示包络重建。

## 验收边界

- 通过：15 件 CAD/网格、6 件手臂/手掌静态包络、URDF、MuJoCo、16 轴线/轴心映射、SolidWorks 组件与运动审图。
- 未通过且不伪装：真实 C001 安装耳/连接器/紧固件/轴承/线束/公差、显示夹具实物试装、整机称重与热稳态。
