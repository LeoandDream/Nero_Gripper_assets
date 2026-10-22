# 现有 NERO + Gripper 组合索引

不新造第二个权威整机模型。冻结基线 `77bd26a`：

- [静态参考整机](../../../mjcf/nero_gripper.xml)：原设计 `nu=0`，适合查看/来源/运动学核对。
- [Simulation MVP 可执行联合入口](../../../mjcf/simulation_mvp.xml)：包含桌面与cube的R1场景，9个实体执行器、8D用户位置目标。场景归属见 [R1索引](../../scenes/r1_cube_grasp/README.md)。
- [既有MVP报告](../../../reports/nero-gripper-simulation-mvp.md)：固定R1工程 `SIMULATION_MVP_ASSET_READY` / PASS；研究几何认证BLOCKED，长保持UNKNOWN，保留已知接触限制。没有将其改成通用成功率声明。

本轮不改变上述入口、控制器、原Run和下游Episode/Loader/训练；不重跑R1。独立夹爪见 [components/gripper](../../components/gripper/README.md)，它与整机共享原mesh，不替换整机文件。
