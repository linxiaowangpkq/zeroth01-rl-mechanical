# Zeroth-01 RL 机械交接（兼容入口）

原始 `rl_ready` 基线已被带圆润外壳质量、头部电子件、IMU、四点足底压力和 ToF 的当前模型取代。

请使用：

- `RL_ROUND_V1_HANDOFF_zh.md`
- `generated/urdf/zeroth01_rl_round_v1.urdf`
- `generated/mujoco/zeroth01_rl_round_v1.xml`
- `one-seq.md`

当前模型为 26 links、25 joints（16 个转动关节）、16 actuators、8 sensors，名义总质量 `4.586857125474 kg`。`zeroth01_rl_ready.*` 仅保留作原机构/质量差分参考，不是新训练的唯一入口。

硬件部署仍为 false：16 台舵机的确认 ID、方向、零位、背隙、电流、温升和扭矩-速度曲线尚待实物标定。
