# 组件、组合与场景索引

唯一来源基线：`77bd26a58df4eda99f611de7be1013d714ddaa7f`。本轮只新增独立夹爪和薄索引，原模型、网格、R1、便携包及下游训练不变。

| 层级 | 入口 | 实际状态与依赖 |
|---|---|---|
| components | [独立 Gripper](components/gripper/README.md) · [gripper.xml](components/gripper/gripper.xml) | **GRIPPER_STANDALONE_READY**，仅静态+一次空载控制；2 slide / 1 equality / 2原生执行器，复用692个手指geom；[实测报告](../reports/issue35-standalone-gripper.md) |
| assemblies | [NERO + Gripper](assemblies/nero_gripper/README.md) | 现有整机；静态参考为原 `mjcf/nero_gripper.xml`，可执行联合入口为原 `mjcf/simulation_mvp.xml`。仅固定R1工程READY；研究几何认证BLOCKED，长保持UNKNOWN |
| scenes | [R1 cube grasp](scenes/r1_cube_grasp/README.md) | 现有冻结桌面/cube场景；保留原固定R1 PASS及接触限制，不在本轮重跑 |

本组件XML加载只需MuJoCo 3.2.3和manifest列明的仓库网格/纹理。控制示例另用NumPy/Pillow；[manifest](components/gripper/manifest.json)列出实际运行文件的相对路径和SHA。XML单文件不是独立分发包，勿脱离依赖直接拷走。本轮不重新打包。

未来资产仅为 **PLANNED**，没有建立空目录或声称实现：`components/nero/`（优先引用已有厂商机械臂）、`components/dh116/`、`assemblies/nero_dh116/`、`assemblies/dual_nero_gripper/`。

## 未来左右实例与安装约定（未实现）

- 两个实例用 `left_` / `right_` 前缀，覆盖body、joint、geom、actuator、equality、site及所有引用。例：`left_gripper_joint1` / `right_gripper_joint1`；两个安装坐标为 `left_flange_mount` / `right_flange_mount`。
- `flange_mount` 对应现有整机的 `flange_link` 坐标，而非它的碰撞壳；到 `gripper_base` 的变换为 `xyz=(0,0,0.006)m, rpy=(0,0,0)`。未来各臂的安装frame决定世界位姿，禁止靠改变手指局部几何做镜像。
- mesh/material/texture优先共享并只注册一次，保持此组件引用的原字节。如果需要各自的asset命名空间，同样给mesh/material/texture及引用加实例前缀；禁止重复注册同名资产。此次没有实现实例化加载器。
- 一只夹爪的动作仍是1D开口命令；现有单臂7+1=8D契约不改。本轮不预设最终双臂action向量，也不实现双臂控制、运动学或验证。

研究级几何BLOCKED与本次组件空载控制是不同验收范围；即使空载READY，也不等于夹持成功、任意物体能力、复杂接触或实机标定通过。
