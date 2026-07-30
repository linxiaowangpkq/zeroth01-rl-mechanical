# Zeroth-01 Round V3 RL 机械交接入口

当前 RL 机械基线已经包含白色 Round V3 外壳的质量/惯量聚合、头部显示与传感器包络、IMU、四点足底压力和 ToF。

训练只使用：

- `generated/urdf/zeroth01_rl_round_v1.urdf`
- `generated/mujoco/zeroth01_rl_round_v1.xml`
- `generated/config/`
- `RL_ROUND_V1_HANDOFF_zh.md`
- `one-seq.md`

当前模型：

- 26 links；
- 25 joints，其中 16 个运动关节；
- 16 actuators；
- 8 sensors，另有固定 `head_camera`；
- 名义总质量 `4.997342616724 kg`；
- 坐标为前方 `-Y`、上方 `+Z`、左侧 `+X`。

SolidWorks 中的 S01–S16 蓝色 STS3250 只用于可视化 joint frame 和受控外包络：

- 不新增 URDF link/joint；
- 不新增 collision、mass 或 inertia；
- 不代表已经证明舵机安装耳、舵盘、轴承、紧固件或反力路径。

`zeroth01_rl_ready.*` 只保留为原机构/质量差分参考，不是新训练的机械入口。

本版圆润手臂套和 Q 版手掌已作为对应 link 的固定 visual/collision/质量叠加进入机械模型；手掌不增加自由度，也不能当作灵巧手训练。

硬件部署仍为 `false`：16 台舵机的总线 ID、方向、零位、硬限位、背隙、电流、温升、扭矩-速度曲线以及电池/主控/传感器实物质量尚待标定。
