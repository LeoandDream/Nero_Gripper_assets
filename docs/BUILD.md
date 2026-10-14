# 冻结来源重建与维护

## 输入与输出

```text
urdf/source/ + meshes/source/ + SOURCE_MANIFEST.json
    → urdf/nero_gripper.urdf
    → mjcf/nero_gripper.xml + meshes/visual/ + meshes/collision/
    → docs/robot-contract.json + reports/mesh-conversion.json
```

输入来自 `agilexrobotics/agx_arm_urdf@a68b8e876a2c6fcb0ecee7a7943129e4d48dd864`。正常重建使用仓库自带快照，不需要 ROS、Xacro 服务或联网。构建器只生成静态参考资产；控制模型由现有控制器在运行时派生。

## 环境与命令

在仓库根目录，Python 3.11 的依赖统一由 `requirements.txt` 固定，包含 MuJoCo、NumPy、SciPy、pycollada、trimesh 和 Pillow。`requirements-build.txt` 保留为同一依赖清单的兼容入口。

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
```

以下重建命令会写入派生资产；在单独的 Feature 分支或临时源码副本执行，并检查差异：

```bash
.venv/bin/python scripts/build.py
.venv/bin/python scripts/validate.py --output debug/static-validation.json
.venv/bin/python scripts/test.py
git diff -- urdf mjcf meshes docs/robot-contract.json reports/mesh-conversion.json
```

`scripts/validate.py` 默认写 `reports/static-validation.json`，用 `--output` 可将维护核对结果保存在开发区。构建器不自动改写文档、验收结论或 `SHA256SUMS`。原校验清单用于检查已交付文件；若有经审查的预期变更，应在验证后更新相应校验值，不能先更新校验值掩盖未知差异。

首次导入或显式重新冻结来源时，构建器支持下面的维护命令，并拒绝不同提交：

```bash
.venv/bin/python scripts/build.py --source-repo /path/to/pinned/agx_arm_urdf
```

该操作重写来源快照，通常使用已有交付文件无需执行。来源版本变更应另行审查，不是普通构建步骤。

## 转换约束与设计依据

- 原始 URDF/Xacro、DAE/STL 和许可保持原字节；展开 URDF 使用可迁移的相对资源路径。独立核对器将展开描述与冻结源记录比对，防止派生 URDF 和 MJCF 同时出错却相互验证。
- 保留实体安装关系、质量、惯性中心、完整惯性、关节轴和范围。完整惯性显式分解为主轴/主惯量，再以编译模型重建张量核对；不用碰撞凸包重估机器人惯量。
- 消去无物理内容的夹爪命令叶子，以两个实体手指及 `q2=-q1` equality 表达源 mimic；避免虚构质量。保留源命令与实体坐标映射。
- 可视 DAE 保留场景变换和三角面，转换为带 UV 的 OBJ 与小型 PNG 颜色调色板，保留各材料图元的 diffuse 配色；接受 8 bit 颜色量化与渲染光照差异。
- 碰撞 STL 按精确坐标连通分量生成凸包，不修补源拓扑、不移动顶点、不加厚平面。记录全部来源面归属，并明确凸包填凹的接触代价。
- 构建输出静态模型 `nu=0`、一个 equality、无显式碰撞 exclude。受控配置的执行器、时间步和限定相邻过滤由运行时派生逻辑明确管理；重建静态资产不会把控制配置写回参考模型。

详细工程取舍和物理边界见 [MODEL.md](MODEL.md)。源码中的构建逻辑与机器接口清单必须一致；更新接口元数据时同步维护构建器的描述输出。

## 独立验证与兼容性

`scripts/validate.py` 以独立 URDF 前向运动学核对闭合、中间、张开及八组合法关节姿态，检查质量/惯性、安装、轴/范围、mimic、几何包围边界、来源一致性和碰撞角色 mask。静态过程只调用 `mj_forward`。

`scripts/test.py` 汇总静态与动态回归；动态层检查控制输入、力限、共享机械字段和可搬迁源码的真实空载控制。测试通过不等于抓取验收通过。固定环境与各验证层的范围见 [VALIDATION.md](VALIDATION.md)。开发输出和日常 Run 保存在被忽略的 `debug/`、`experiments/dynamics/runs/`；维护时保留唯一原始证据及来源索引。
