# NERO + Gripper 原生MuJoCo动力学实验

受保护资产：`8e8b805e28bd07a6799d3d838574912ab2280912`。本目录仅增加独立派生模型、控制、简单抓取及验收；原source、MJCF、静态报告与SHA清单保持原字节。本轮不接入LIBERO/OpenPI，不执行真机动作。

控制契约：8D绝对目标 `[joint1..joint7(rad), source_gripper_command(m)]`，物理qpos仍9D。夹爪命令范围0..0.1m，实体目标分别为±命令一半；原双指约束保留。控制参数以有来源的仿真候选标明，不称硬件标定。

执行顺序：复验静态资产及依赖 → 核查官方Joint2定义及底座接触 → 执行保持/单轴/多轴/夹爪动态检查和异常输入验证 → 固定规则立方体抓取/抬升/保持及少量冻结条件复测 → 独立只读审查。抓取成功条件、物体参数与测试初始条件须在执行前保存，所有失败保留。

所有运动必须以 `mj_step` 积分；qpos只允许合法初始reset和离线回放设置。禁止物体weld/吸附、隐藏外力、随意碰撞过滤、动态改变物体/成功条件。若需要改物理假设，先给出具体候选diff和同条件前后证据，等待用户确认再采用。

开发候选和异常诊断放在 `debug/`；实际运行完整日志、状态轨迹和视频放在唯一 `runs/` 目录，程序拒绝覆盖。正式结论引用原始Run，不复制事实数据库或建立HTML平台。

当前阶段结果和待确认项见 [results.md](results.md)。在本工作树根目录可复现：

```bash
# 使用已建立的资产环境；异机先安装requirements-build.txt
/data_robot/projects/nero_gripper_assets/.venv/bin/python -m unittest discover -s experiments/dynamics -p test_controller.py -v
/data_robot/projects/nero_gripper_assets/.venv/bin/python experiments/dynamics/run.py --stage risk --run-id new_risk_run
/data_robot/projects/nero_gripper_assets/.venv/bin/python experiments/dynamics/run.py --stage control --run-id new_control_run
```

`risk`只做未批准单对过滤候选的前后诊断；`control`当前仅运行未过滤模型。两者都保存真实积分状态、命令与接触；源几何不改。实际执行日志路径见results，不能把CLI正常结束或控制跟踪PASS解释为已完成抓取验收。
