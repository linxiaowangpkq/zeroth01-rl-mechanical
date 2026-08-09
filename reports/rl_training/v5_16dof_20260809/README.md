# 2026-08-09 16DoF RL evidence

本目录把可审计的训练/门禁摘要随机械基线保存；原始训练工程、视频、完整逐物理步 trace 和可恢复训练入口仍位于 RL 工作区。

- `Z4_gate_report.json`：正式 8 × 4 s 跑步门禁。
- `Z4_restore_report.json`：CUDA checkpoint 恢复和 46→16 shape 契约。
- `Z4_vs_random.json`：同种子随机策略对照。
- `M4_hardware_load_report.json` / `M4_joint_hardware_statistics.csv`：步行负载审计。
- `L3_hardware_load_report.json` / `L3_joint_hardware_statistics.csv` / `L3_foot_impact_slip_statistics.csv`：10 s 跑步负载审计。
- `L3_joint_torque_speed.png`：每关节扭矩-速度散点。
- `STS3250_reference_search_2000x4x10s.json`：2,000 个参考步态、4 种子、10 s 的速度/飞行搜索。
- `V13_soft_speed_constraint_screen.json`：软速度约束 PPO 全 checkpoint 外部复放。
- `Z4_hard_3rad_slew_screen.json`：原 Z4 在硬 3 rad/s 目标速率下的复放。
- `V14_hard_3rad_training_screen.json`：硬速率执行器 PPO 全 checkpoint 联合门禁。

电流、温升和母线压降使用报告中显式记录的假设；在真实 STS3250 系统辨识前，不得视为实测值或上机许可。
