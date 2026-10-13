# NERO + Gripper 原生 MuJoCo 动力学实验

当前能力：**空载关节/夹爪控制通过；立方体物理抓取未通过。** 这是独立 `dynamics-control` 开发工作树，未合并或推送到原资产 main。结果、失败证据与能力边界见 [results.md](results.md)；独立审查见 [review.md](review.md)。

受保护资产 `8e8b805e28bd07a6799d3d838574912ab2280912` 的 source、MJCF、静态报告和127项SHA保持原字节。本目录只有一个运行入口、控制器、配置和必要测试。`runs/` 保留不可覆盖的完整实验记录；`debug/` 是开发日志。抓取代码和配置属于已执行但未通过物理验收的候选，不能作为已验收抓取能力发布。

## 控制契约与参数

输入为8D绝对位置目标 `[joint1..joint7(rad), source_gripper_command(m)]`。机械臂采用源关节范围，Joint2另限制在保守候选 `[-0.2,1.5]rad`；夹爪源命令0..0.1m映射两个实体目标 `+w/2,-w/2`，原双指 equality 保留。这个宽度坐标不是实机精确指尖净间距。机器人状态仍9D；加入自由立方体后总qpos16D/qvel15D。九个内部servo通道不是九个独立用户控制自由度。

九个 MuJoCo affine general actuator 使用单位gear：比例反馈及 `qfrc_bias` 前馈写入Nm/N控制量，速度反馈在原生actuator bias中隐式积分。KP机械臂 `[160,160,160,80,40,40,30]Nm/rad`、KV `[12,12,10,6,3,3,2]Nms/rad`；每指KP800N/m、KV8Ns/m。总actuator力限分别100Nm/10N，来自源URDF effort描述；增益是仿真候选，不是厂商标定。模型机械阻尼、惯性、几何、摩擦、接触和equality参数保持源值。唯一新增碰撞排除是已批准的 `base_link/link1`，授权及具体候选见 `base-filter-approval.json`。

物理积分为1ms/implicitfast，目标更新100Hz，反馈每个物理步计算。接口目标速率上限0.2rad/s、夹爪源宽度0.03m/s；实际速度另检查≤1rad/s、手指≤0.1m/s。异常形状/数值/越界目标拒绝且保持最后有效目标；scratch IK对未能满足位姿阈值的目标显式报错。此接口没有自动避障或实机安全认证。

所有执行使用真实 `mj_step`，执行中不写qpos，不使用物体actuator、weld、吸附或外力。qpos仅在合法初始化、scratch规划和明确标记的离线回放中设置。IK使用SciPy bounded least_squares，只生成同一8D接口的关节轨迹。依赖：Python3.11.9、MuJoCo3.2.3、NumPy1.26.4、SciPy1.15.3、Pillow11.2.1，视频使用系统ffmpeg；本机复用原资产 `.venv`，异机安装原 `requirements.txt` 并补充 `scipy==1.15.3`。

## 运行

在本工作树根目录执行；新的物理Run必须使用未占用的run-id，已有Run不会被覆盖：

```bash
/data_robot/projects/nero_gripper_assets/.venv/bin/python -m unittest discover -s experiments/dynamics -p 'test_*.py' -v
/data_robot/projects/nero_gripper_assets/.venv/bin/python experiments/dynamics/run.py --stage control --filtered --run-id my_control
# 当前抓取候选已知失败；本命令是复现试验，不代表抓取已验收
/data_robot/projects/nero_gripper_assets/.venv/bin/python experiments/dynamics/run.py --stage grasp --filtered --run-id my_grasp --plan-from-run grasp_20261014_002
# 保存的真实积分状态离线回放，不运行mj_step；已有视频不会覆盖
MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1 /data_robot/projects/nero_gripper_assets/.venv/bin/python experiments/dynamics/run.py --stage replay --run-id my_grasp --episode fixed
```

不提供 `--plan-from-run` 时重新计算全部冻结条件的scratch IK。`risk`阶段用于保留原先单对排除候选的前后诊断，不代表新增物理变更获得批准。正式抓取判定在运行前冻结，失败保留；第一固定条件失败时，按事先规则不执行两个偏移条件。本轮没有LIBERO/OpenPI、训练或真机动作。
