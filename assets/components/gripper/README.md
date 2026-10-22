# Standalone NERO Gripper

源：冻结 `77bd26a` 的 [simulation_mvp.xml](../../../mjcf/simulation_mvp.xml)。独立入口：[gripper.xml](gripper.xml)。状态：**GRIPPER_STANDALONE_READY**，仅静态来源等价+一次空载控制；[实际结果与限制](../../../reports/issue35-standalone-gripper.md)。

安装参考 `flange_mount` 固定于世界原点，表示源整机 `flange_link` 的坐标系；保留 `gripper_base` 相对它沿z的6mm偏移。没有复制NERO机械臂、法兰碰撞、桌面或cube。两指各346个collision geom，基座4个collision geom及3套原visual/material保留，网格直接引用同一仓库 `meshes/`。

命令 `w∈[0,0.1]m` 是1D绝对位置，实体目标 `q1=+w/2, q2=-w/2`，原equality仍为 `q2=-q1`。这里的w是原命令坐标，不声称是标定的指尖净距。状态只有两个真实slide DOF，不含虚构开口关节。

固定根适配：MuJoCo不对world焊接体应用原父子过滤，因此组件显式排除 `gripper_base↔gripper_link1/2`，复现源整机已有的隐式排除。手指—手指接触继续启用；这不是额外关闭原本可发生的接触。静态测试逐对核对243,951对geom的接触资格，实际结果见验收报告。

原两个限力general执行器保留：`kp=800 N/m`由反馈入口实现，`kv=8 Ns/m`在原执行器bias中，限力±10N；0.03m/s命令限速、100Hz更新、1kHz反馈/积分不变。入口 `GripperController.set_target(w)` 接收标量或单元素向量，`advance_command()`限速，`apply(data)`写2D力输入；调用方负责 `mj_step`。2D力ctrl、9D整机ctrl都不是用户开口动作。

从仓库根加载和查看：

```bash
python -c "import mujoco; m=mujoco.MjModel.from_xml_path('assets/components/gripper/gripper.xml'); print(m.nq,m.nv,m.nu,m.neq)"
```

静态查看（需要桌面显示；只做一次forward和viewer同步，不调用mj_step）：

```bash
python - <<'PY'
import time
import mujoco
import mujoco.viewer
model = mujoco.MjModel.from_xml_path('assets/components/gripper/gripper.xml')
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        viewer.sync()
        time.sleep(.02)
PY
```

安装仓库原requirements（MuJoCo3.2.3）。从仓库根执行仅静态的来源对照和一次新的空载序列：

```bash
python tests/test_standalone_gripper.py --report /tmp/gripper-static-new.json
python examples/gripper_control.py --output /tmp/gripper-empty-new
```

仅重算已有原始Run，不加载模型或新增积分：

```bash
python tests/test_standalone_gripper.py --run /path/to/existing/empty_001 --report /tmp/gripper-offline-new.json
```

物理命令会执行一次9500步/9.5秒无负载序列：open→middle→close→reopen，输出原始trajectory、每阶段指标、日志索引和曲线。目录必须不存在，不自动重试；不要把它误认为零积分检查。初始w=.08m，沿用源配置；初始化之后不写qpos、不施加外力。

依赖和源SHA见 [manifest.json](manifest.json)，不复制/生成碰撞网格。XML相对引用仓库网格/纹理，控制示例还需要manifest；仅单拷XML不保证加载。提取脚本 `scripts/extract_gripper.py` 只用于审计来源，拒绝覆盖已交付组件，正常使用无需重建。

清单的358项为组件XML、控制脚本、353个mesh和3张纹理；manifest自身的SHA另在运行前封存和验收报告中记录，避免自指。外部Python包使用仓库原requirements与报告记录的实际版本。

原R1成功、长保持UNKNOWN、研究几何BLOCKED及legacy MPR局部接触限制保留；独立空载通过不认证独立抓取、全幅工作面或硬件真实性。
