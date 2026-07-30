# Zeroth-01 CAD / 运动审查简报

## 目标和边界

目标是提供可在 SolidWorks、URDF 和 MuJoCo 中一致消费的 Zeroth-01 机械参考模型，用于 RL 训练前的结构、关节轴、运动和碰撞审查。

这不是制造发布包。当前公开输入只有三角面 STL 和机器人描述，没有完整 Onshape/SolidWorks 原生特征、紧固件、线束、公差或材料工艺数据。

## 权威输入

| 输入 | 锁定内容 |
|---|---|
| `zeroth-robotics/zeroth-bot` | 控制和机器人接口参考 |
| `zeroth-robotics/zeroth-sim` | link/joint 树、frame、轴、质量、惯量和限位 |
| 官方 `stompymicro` Drive 包 | 17 个公开 STL |
| K-Scale 文档/历史 `kos-zbot` | 舵机型号和候选总线家族映射 |
| Feetech STS3250 资料 | 尺寸、质量、额定/堵转转矩、速度、电流和编码器分辨率 |

Onshape 匿名导出返回 401，因此没有声称获得原生装配特征树。

## 几何兼容决策

Drive STL 必须与 `zeroth-sim` 的 `33b0553...` URDF 配对。后续
`43c5baa...` 改变了 frame、惯量和 mesh 命名，却没有发布替代 STL；旧 STL 与该新 URDF 混用会产生假分离和错误质心。

本地生成器通过 `git show 33b0553...:sim/resources/stompymicro/robot.urdf`
读取冻结内容，不依赖当前工作树文件名。映射记录在
`config/mesh_name_map.json`。

## 坐标和质量合同

- 长度：m；角度：rad；质量：kg；惯量：kg·m²。
- URDF 为 Z-up，关节轴在 joint frame 中表达。
- visual/collision mesh 不缩放、不重心化、不静默交换轴。
- 质量、质心和惯量来自几何兼容的官方 URDF，并按 link 聚合。
- 聚合惯量已经包含结构和舵机；不得再额外加入 `16 × 74.5 g`。

精确数据见 `reports/joint_servo_frames.csv` 和
`reports/link_inertial_audit.csv`。

## 连接与 SolidWorks 复核

- 17 个 STL 作为 metre 单位的 SolidWorks surface parts 导入。
- 装配体由同一 URDF FK 链驱动，不另建一套手工坐标。
- 图形刷新必须调用 `UpdateBox`、rebuild 和 redraw；否则 SolidWorks
  可能显示旧包围盒，让正确的底层 Transform2 看起来像爆炸视图。
- 最终 17 个组件包围盒中心与 URDF 的最大误差为 `5.82e-17 m`。
- 16 个转动关节各检查 lower/zero/upper，共 48 个姿态；最大 Transform2
  回读误差为 `4.44e-16`。

公开 STL 没有稳定的圆柱 B-Rep mate 面，因此交付声明的是 SolidWorks
COM FK 扫掠，不声明原生 mate/motor Motion Study。

## 碰撞口径

中立装配存在四组设计性相邻嵌合：

- `Torso :: hip_yaw_left`
- `Torso :: hip_yaw_right`
- `Torso :: shoulder_yaw_left`
- `Torso :: shoulder_yaw_right`

只有这四组被白名单排除，其他自碰撞全部禁止。完整原始范围的随机组合并不安全；最大通过的统一启动盒是 30% 范围：

- 20,000 个确定性随机姿态：0 个新碰撞；
- 65,536 个盒角点：0 个新碰撞；
- 原生 MJCF 再验证 100,000 个随机姿态：0 个碰撞。

这仍不覆盖连续碰撞、装配公差、线束、紧固件、柔性件和受力变形。

## 制造前必须补齐

1. 获取或重建原生 B-Rep/参数化 CAD。
2. 加入舵机壳体、输出盘、紧固件、轴承和线束包络。
3. 定义材料、壁厚、打印/CNC 工艺和公差栈。
4. 做实体级连续碰撞与极限姿态检查。
5. 用实物治具标定零位、方向、硬限位、回差和热/电流能力。
