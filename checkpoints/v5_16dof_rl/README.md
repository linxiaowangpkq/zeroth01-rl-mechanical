# Zeroth-01 v5 16DoF RL milestones

这里只保存通过明确仿真门禁的冻结 checkpoint。N/M/Z 分别表示站立、步行和跑步，不代表 PPO 更新总数或实体部署许可。

| 里程碑 | 文件 | 状态 |
|---|---|---|
| N12 `gait_neutral` 站立 | `N12_gait_neutral_standing/ckpt.12.bin` | PASS；canonical 中立姿态回归 |
| N12 历史零位站立 | `N12_standing/ckpt.12.bin` | PASS / 仅旧模型回归 |
| M4 步行 | `M4_robust_walk/ckpt.4.bin` | PASS；8/8 × 4 s，0.106928 m/s |
| Z4 跑步 | `Z4_robust_run/ckpt.4.bin` | PASS；8/8 × 4 s，0.150424 m/s，飞行相 0.03 |

全部新 checkpoint 是 46 维观测、16 维全身 PPO 动作，六个手臂关节和十个腿部关节均由策略输出；每个执行器扭矩限制为 `±1.2552512 N·m`。

Z4 是仿真跑步里程碑，不是 STS3250 实机合格证明。其 10 s 负载复放显示膝关节 p95 速度约 5.0 rad/s，高于配置中的 3 rad/s 包络。对 Z4 加硬 3 rad/s 目标速率后 0/8 生存且飞行相消失；硬约束 PPO v14 训练 147,201 样本后仍为 0/8。因此实体跑步必须先关闭 `bug.md` 中的 BUG-RL-V5-007。

详细门禁、恢复、随机策略对照和硬件负载证据见 `reports/rl_training/v5_16dof_20260809/`。没有执行任何实体舵机命令。
