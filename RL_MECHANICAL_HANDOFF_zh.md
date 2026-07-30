# Zeroth-01 RL 机械交接

## 1. 训练入口

优先使用：

```text
generated/urdf/zeroth01_rl_ready.urdf
generated/mujoco/zeroth01_rl_ready.xml
generated/config/zeroth01_actuator_metadata.json
generated/config/zeroth01_collision_policy.json
```

URDF 的 `floating_base` 是 fixed joint，便于通用 URDF 工具验证。做双足/自由基座训练时：

- MuJoCo 直接使用提供的 MJCF；它已经有 free joint，`nq=23`、`nv=22`。
- 其他引擎应在 loader 中把根节点设为 floating/free，不要修改 16 个关节 frame。

## 2. 机械模型事实

| 项目 | 值 |
|---|---|
| link | 18（含质量为 0.001 kg 的描述根 `base`） |
| joint | 17（1 fixed + 16 revolute） |
| 自由度 | 16：每臂 3、每腿 5 |
| 总质量 | 3.0954718282 kg |
| 舵机 | 16 × Feetech STS3250 |
| 编码器 | 4096 count/rev，名义中位 2048 |
| URDF 力矩上限 | 2.0 N·m |
| 速度上限 | 5.0 rad/s |
| 仿真 damping | 0.53 N·m·s/rad |
| 仿真 Coulomb friction | 0.001 N·m |
| 候选控制频率 | 50 Hz |

STS3250 厂商数据：45.22 × 24.72 × 35 mm、74.5 g、12 V，
额定约 1.569 N·m，堵转约 4.903 N·m，空载约 7.874 rad/s。
堵转转矩不能作为连续 RL 力矩。

## 3. 舵机位置和候选 ID

每个输出轴在 parent frame 和中立 world frame 中的 xyz、rpy、正轴方向、
安全限位和编码器信息位于 `reports/joint_servo_frames.csv`。
该 joint frame 就是可恢复的输出轴 frame；公开聚合 STL 不能可靠分离出舵机壳体中心。

候选 ID 仅用于开机扫描顺序：

| 链 | 候选 ID |
|---|---|
| 左臂 shoulder pitch / yaw / elbow | 11 / 12 / 13 |
| 右臂 shoulder pitch / yaw / elbow | 21 / 22 / 23 |
| 左腿 hip yaw / roll / pitch / knee / ankle | 31 / 32 / 33 / 34 / 35 |
| 右腿 hip yaw / roll / pitch / knee / ankle | 41 / 42 / 43 / 44 / 45 |

这些 ID 不是当前这台 16DoF 实机的确认值，必须先做 bus scan。

## 4. 质量、质心和惯量

`reports/link_inertial_audit.csv` 给出每个 link 的：

- mass；
- link-frame COM；
- 完整惯量张量；
- 主惯量；
- 正定和三角不等式门禁。

所有条目均 PASS。惯量是结构与舵机的聚合值；不要再创建 16 个 74.5 g
舵机刚体，否则会重复计重。

建议的 domain randomization 已写入 actuator metadata：

- link mass scale：0.95–1.05；
- damping scale：0.7–1.3；
- armature scale：0.7–1.3；
- frictionloss scale：0.5–1.5；
- joint zero offset：±2°。

## 5. 控制初始化

元数据提供的候选 PD：

- 腿部：`kp ≈ 17.6815`，`kd ≈ 0.5355`；
- 手臂：`kp = 5.0`，`kd = 0.3`。

腿部值来自官方 active config；手臂值来自官方注释候选，仍需调参。
训练动作应先裁剪到 2 N·m 和 5 rad/s，并从受控启动关节盒开始。

## 6. 碰撞策略

白名单仅有四组中立装配嵌合：

```text
Torso :: hip_yaw_left
Torso :: hip_yaw_right
Torso :: shoulder_yaw_left
Torso :: shoulder_yaw_right
```

其余自碰撞必须启用。建议：

1. reset 只在 `zeroth01_collision_policy.json` 的 30% startup box 内采样；
2. policy step 做自碰撞查询或 action projection；
3. 新碰撞立即终止并惩罚；
4. 扩展关节范围时重新做随机、角点和连续轨迹验证。

完整源限位的多关节随机组合已发现自碰撞，不能直接作为无约束 action range。

## 7. 已验证的证据

原生 MJCF：

- 中立位和官方站立位：0 碰撞；
- 16 个关节动态/轴向门禁：PASS；
- 100,000 随机姿态：0 碰撞；
- 65,536 安全盒角点：0 碰撞；
- 总质量与 URDF 完全一致。

URDF importer：

- 每关节 101 点，共 1,616 轴向样本；
- 20,000 个随机姿态；
- 除四组中立白名单外，没有新增碰撞对。

SolidWorks：

- 17/17 组件连接；
- 16/16 关节 lower/zero/upper 扫掠；
- 48/48 Transform2 回读 OK；
- 最终中立位与 URDF 最大位置误差 `5.82e-17 m`。

证据文件：

```text
reports/mujoco_rl_ready_gate.json
reports/mujoco_rl_ready_urdf_motion_summary.json
reports/global_collision_box_search.json
reports/solidworks_review_gate.csv
reports/solidworks_kinematic_sweep.csv
reports/solidworks_component_placement_audit.json
```

## 8. 真机部署前的硬阻塞

`hardware_deployment_ready` 目前必须保持 `false`。逐台完成：

1. 断开负载或低力矩模式做总线扫描，确认 ID；
2. 使用机械中立治具记录 measured zero count；
3. 低力矩正向 jog，填写 URDF-to-servo direction sign；
4. 测量软/硬限位、回差、空载电流；
5. 做转矩-电流、速度、温升和连续工作包络辨识；
6. 更新 friction、damping、armature 和 domain randomization；
7. 先单腿/双腿悬空，再进入地面低增益测试。

标定入口：

```text
generated/config/zeroth01_hardware_calibration_template.csv
```

在该表 16 行全部填写并通过低力矩复核前，不得把候选 ID、2048 名义零位或
仿真方向直接当作真机事实。
