# NERO + 官方两指夹爪仿真资产

以厂商URDF/Xacro为机械依据的独立资产包。来源锁定 [agx_arm_urdf@a68b8e8](https://github.com/agilexrobotics/agx_arm_urdf/tree/a68b8e876a2c6fcb0ecee7a7943129e4d48dd864)，采用该版本的官方法兰、夹爪基座和两指模型。

```text
urdf/       相对路径展开URDF；source保留官方原字节
mjcf/       保留实体机械字段的MuJoCo模型
meshes/     原始DAE/STL、派生可视OBJ与分量凸碰撞体
examples/   静态查看入口
scripts/    离线重建与独立URDF核对
reports/    静态等价性、碰撞近似、引擎兼容性和预览
docs/       接口、构建方法与模型边界
licenses/   官方MIT许可及来源
```

## 查看

首次克隆并建立运行环境：

```bash
git clone https://github.com/LeoandDream/Nero_Gripper_assets.git
cd Nero_Gripper_assets
python3.11 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

在仓库根目录运行：

```bash
.venv/bin/python examples/view_static.py
.venv/bin/python examples/view_static.py --collision
```

无桌面服务器可导出静态图片：

```bash
MUJOCO_GL=egl .venv/bin/python examples/view_static.py --image preview.png
MUJOCO_GL=egl .venv/bin/python examples/view_static.py --width 0 --image closed.png
```

`--width` 是源URDF的虚拟开合命令坐标，范围0..0.1米；两指分别取其±一半。这不是经过真机标定的实际指尖净间距。查看器只设置姿态、调用 `mj_forward` 并显示，不积分物理、不执行策略。

`meshes/visual`包含带UV的OBJ网格与12张小型PNG颜色调色板（总计1213字节），保留源材料的局部多色、银白面板和NERO标识；原始DAE仍保留。这些调色板是原配色的运行资源，不需要原先的JPG集合。

![静态参考姿态](reports/preview.png)

## 已验证范围

- 官方29项源文件逐字节保存，Git blob ID与SHA256可复核。
- 12个实体部件、9个标量关节、1个双指线性约束，执行器数量0。原URDF额外的空虚拟命令关节在MJCF中消去；具体映射见 [接口契约](docs/robot-contract.json)。
- 11组合法静态配置独立核对源URDF前向运动学、质量/惯性、安装变换、关节轴/范围和可视/碰撞包围边界；结果见 [原始报告](reports/static-validation.json)。
- 总质量5.12892366kg；质量、惯性中心、轴和限位核对误差0，完整惯性重建最大误差约1.73e-18 kg·m²。几何边界最大误差约9.27e-9m，属于浮点转换核对值，不代表接触精度。
- MuJoCo3.2.3完成上述静态核对；当前LIBERO环境的2.3.7另行通过编译及参考姿态 `mj_forward`，该兼容性检查范围见 [报告](reports/libero-engine-compatibility.json)。
- 8项测试通过，涵盖源字节、虚拟关节消去、mimic、独立静态核对、迁移后编译、查看器，以及源参考被同时篡改和碰撞mask清零的拒绝反例；详细记录见 [验证记录](reports/tests.txt)。独立审查与两项补强见 [审查记录](reports/review.md)。

![夹爪闭合、中间、张开三种静态姿态](reports/gripper-states.png)

## 接触和控制边界

碰撞体为62个源分量凸包，原STL和原DAE均保留。凸包仍会填充源凹部；所有源碰撞三角面有分量归属记录，不能据此称碰撞表面完全等同。两指各一组4面平面标识保留并入已包含它的凸包，没有给平面伪造厚度。

11组静态配置都报告base_link/link1约5mm的代理重叠；目前没有新增显式碰撞过滤。零开合命令下两指之间还有约1.35e-10m的数值接触。它们如实保留在报告中，静态字段等价不表示动态接触已经验证。细节见 [模型边界](docs/MODEL.md)。

本包提供资产和静态视图，尚未提供可执行的末端控制器、7D策略动作适配或动态抓持验收。机器人实体qpos为9D；将来VLA接口可设计为8D末端/夹爪状态与7D末端/开合动作，但不能将策略动作直接写进当前模型。

## 迁移与重建

复制本包的交付文件即可；`.venv`、`.upstream`、`.git`与`debug`不属于运行依赖，运行环境单独建立：

```bash
python3.11 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

重建和完整核对需要额外构建依赖：

```bash
.venv/bin/pip install -r requirements-build.txt
.venv/bin/python scripts/build.py
.venv/bin/python scripts/validate.py
.venv/bin/python -m unittest discover -s tests -v
sha256sum -c SHA256SUMS
```

`build.py`从已保存源快照离线重建；首次源导入可使用 `--source-repo /path/to/pinned/agx_arm_urdf`，脚本拒绝其它提交。源文件清单和URI来源分别在 `SOURCE_MANIFEST.json` 和展开URDF/原始Xacro中。构建方法、数值处理与首次失败记录说明见 [BUILD.md](docs/BUILD.md)。
