# NERO + Gripper — MuJoCo Simulation Assets

松灵 NERO 七自由度机械臂及原生双指夹爪的可复用 MuJoCo 仿真资产，面向运动控制、接触动力学、操作任务、模仿学习、强化学习和具身智能实验。机械描述来自冻结的厂商 URDF/Xacro；资产以支持高保真仿真研究为目标，实机动力学一致性尚未标定。

[组件 / 组合 / 场景资产索引](assets/README.md)：独立 Gripper 与原整机、R1场景的入口、依赖及真实验收范围。

![NERO 与原生双指夹爪](reports/preview.png)

## 功能与范围

| 功能 | 当前支持范围 |
|---|---|
| 机械资产 | 源文件可追溯；运动学、安装关系、质量/惯性、限位及网格边界通过静态核对 |
| 查看与加载 | 相对路径 MJCF、URDF；可视/碰撞查看与无桌面导图 |
| 空载关节控制 | 7D 机械臂 + 1D 夹爪绝对位置目标；原生限力执行器与真实 `mj_step` 跟踪通过指定动作验收 |
| 物理抓取 | Simulation MVP 在一次固定 R1 场景通过原抓取及控制健康判据；重复性、偏移任务与泛化尚未验证。旧代理模型的失败记录保留 |
| 实机保真度 | 尚未验证；不将厂商描述一致性或空载控制结果等同于硬件级物理真实性 |

## 安装与加载

以下命令对应 Draft PR #26 的候选分支 `feature/collision-architecture-v1`；没有合并 `main`。需要能在 PATH 中调用的 Python 3.11 解释器；若安装时使用其他命令名，将 `python3.11` 替换为该解释器的命令。

```bash
git clone --depth 1 --branch feature/collision-architecture-v1 https://github.com/LeoandDream/Nero_Gripper_assets.git
cd Nero_Gripper_assets
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

### Simulation MVP：可执行控制模型

独立入口 [simulation_mvp.xml](mjcf/simulation_mvp.xml) 已包含桌面、40 mm / 100 g cube、执行器与原 R1 接触配置。它复用原 v2.1 每指346体，双指共692个碰撞 geom；原静态模型、visual、惯性及控制器保留。直接加载无需重新生成网格，也不依赖旧实验工作树。

```bash
# 加载检查
.venv/bin/python -c "import mujoco; m=mujoco.MjModel.from_xml_path('mjcf/simulation_mvp.xml'); print(m.ngeom, m.nu)"
# 带原控制器的交互查看：保持 R1 初始姿态；需要桌面/OpenGL
.venv/bin/python experiments/dynamics/simulation_mvp.py view
# 以下输出目录必须尚不存在；static/contact 不积分，smoke 真实运行一次完整 R1
.venv/bin/python experiments/dynamics/simulation_mvp.py static --output debug/mvp_static_001
.venv/bin/python experiments/dynamics/diagnose_simulation_mvp.py --output debug/mvp_contacts_001
.venv/bin/python -m unittest discover -s experiments/dynamics -p test_simulation_mvp.py
.venv/bin/python experiments/dynamics/simulation_mvp.py smoke --output debug/mvp_smoke_001
.venv/bin/python experiments/dynamics/verify_simulation_mvp.py --run debug/mvp_smoke_001 --report debug/mvp_verify_001.json --evidence debug/mvp_verify_001.npz
```

本次工程状态 **`SIMULATION_MVP_ASSET_READY`**：一次34,150步的真实 R1 抓取/抬升/保持/释放完成，hold最低抬升96.146 mm、漂移0.574 mm、双指支持100%、机器人—cube最大穿透0.526 mm。研究级几何认证仍为 **BLOCKED**，局部侧壁法向及 certifier UNKNOWN 未被追认通过。legacy MPR 人工深接触探针的法向异常、释放落桌穿透等限制见[实际测试与证据](reports/nero-gripper-simulation-mvp.md)；不要将一次固定成功推广为成功率或复杂 manipulation 验收。

从仓库根目录加载纯静态参考模型：

```bash
.venv/bin/python - <<'PY'
import mujoco
model = mujoco.MjModel.from_xml_path('mjcf/nero_gripper.xml')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
print('nq, nv, nu, neq:', model.nq, model.nv, model.nu, model.neq)
PY
```

输出为 `9 9 0 1`。此参考配置不带执行器；控制配置在运行时从它派生，共享同一套机械资产和网格。

```bash
.venv/bin/python examples/view_static.py
.venv/bin/python examples/view_static.py --collision
# 无桌面服务器，需要可用的 EGL/OpenGL 环境
mkdir -p debug
MUJOCO_GL=egl .venv/bin/python examples/view_static.py --image debug/preview.png
MUJOCO_GL=egl .venv/bin/python examples/view_static.py --width 0 --image debug/closed.png
```

查看器只设置姿态并调用 `mj_forward`。`--width` 是源 URDF 命令坐标，范围 0–0.1 m，并非标定后的指尖净间距。

## 空载控制

```bash
.venv/bin/python experiments/dynamics/run.py --stage control --filtered --run-id my_control
```

输入接口为 `[joint1, …, joint7, gripper_command]` 的 8D 绝对目标：机械臂用 rad，夹爪用 m；内部映射到 9 个实体关节和 9 个限力执行器。`Controller.set_target(action)` 接收目标，`advance_command()` 限速更新，`apply(data)` 写入执行器输入；由调用方执行 `mujoco.mj_step`。接口、参数及接触过滤依据见 [模型设计](docs/MODEL.md) 和 [机器接口清单](docs/robot-contract.json)。

`--filtered` 只排除受控派生模型中的 `base_link/link1` 相邻碰撞对。运行会保存配置、源码快照、状态轨迹和接触记录到 `experiments/dynamics/runs/my_control/`；run-id 必须尚未使用，已有记录不会覆盖。

## 实验性抓取

以下为 40 mm、100 g 自由立方体的接触实验，当前预期会报告验收失败并返回非零。失败候选用于分析接触限制。

```bash
.venv/bin/python experiments/dynamics/run.py --stage grasp --filtered --run-id my_grasp
# 保存实际积分轨迹的离线回放；需要系统 ffmpeg
MUJOCO_GL=egl .venv/bin/python experiments/dynamics/run.py --stage replay --run-id my_grasp --episode fixed
```

执行中通过 `mj_step` 运动，不使用物体吸附、weld 或隐藏外力。回放读取保存状态，本身不积分；视频不能替代成功判定。冻结任务条件在 [grasp.json](experiments/dynamics/grasp.json)，状态与验收范围见 [验证说明](docs/VALIDATION.md)。

## 模型规格与设计

| 项目 | 规格 |
|---|---|
| 实体部件 / 总质量 | 12 / 5.12892366 kg，含官方法兰、夹爪基座和两指 |
| 关节状态 | 7 个旋转关节 + 2 个移动关节；双指通过一个线性 equality 耦合 |
| 夹爪命令映射 | `q_left = w/2`，`q_right = -w/2`，`w ∈ [0, 0.1] m` |
| 网格 | 12 个带 UV 的可视 OBJ 与 12 张 PNG 颜色调色板，62 个分量凸碰撞体；原始 DAE/STL 保留 |
| 控制配置 | 1 ms / `implicitfast`；目标 100 Hz、反馈 1 kHz；增益属于仿真配置 |
| 验证环境 | Python 3.11、MuJoCo 3.2.3；其他引擎版本的检查范围见验证说明 |

[MODEL.md](docs/MODEL.md) 解释机械结构、惯性转换、虚拟命令关节消去、双指软约束、凸包近似及相邻碰撞过滤的目的、依据与代价。[BUILD.md](docs/BUILD.md) 说明冻结输入、离线重建与维护约束。

## 验证与来源

```bash
.venv/bin/python scripts/test.py
.venv/bin/python scripts/test.py --layer static
.venv/bin/python scripts/test.py --layer dynamic
# 在回归测试后另保存完整空载控制验收轨迹
.venv/bin/python scripts/test.py --control --run-id my_acceptance
sha256sum -c SHA256SUMS
```

机械来源固定为 [agx_arm_urdf@a68b8e8](https://github.com/agilexrobotics/agx_arm_urdf/tree/a68b8e876a2c6fcb0ecee7a7943129e4d48dd864)，29 项来源路径、Git blob 与 SHA256 见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。源资产的 MIT 许可保存在 [AGX_MIT.txt](licenses/AGX_MIT.txt)，来源与对照资料的许可范围见 [SOURCES.md](licenses/SOURCES.md)。

凸包会填充源凹部；几何边界一致不证明接触表面真实。Joint 2 的官方来源范围存在差异，当前实验命令采用保守区间，实机限位未确认。抓取、全工作空间控制及真实硬件性能仍需进一步验证。自动化结果、适用边界和必要的历史证据入口集中在 [VALIDATION.md](docs/VALIDATION.md)。
