# 冻结 R1 cube grasp 场景索引

实际场景仍是 [mjcf/simulation_mvp.xml](../../../mjcf/simulation_mvp.xml)，来源 `77bd26a`。桌面、40mm cube和环境属于这一场景，不属于独立Gripper组件。

[原配置与R1协议](../../../experiments/dynamics/simulation_mvp.json)、[原验收报告及证据索引](../../../reports/nero-gripper-simulation-mvp.md)继续有效：固定R1 PASS，长保持UNKNOWN，研究几何认证BLOCKED；保留局部侧壁法向、legacy MPR及落桌接触限制。

本目录只有索引，不复制XML或模型真值，不改变物体初态、成功判据或控制参数。本轮没有运行该场景的新积分或抓取。
