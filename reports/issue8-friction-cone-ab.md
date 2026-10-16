# Issue #8：冻结初态 pyramidal / elliptic 单变量诊断

2026-10-17，Ubuntu 实际执行，所有数据 **diagnostic-only**。

**结论：68 ms 的预言成立，早期下滑减小且触桌延迟；持续抓持没有建立。** elliptic 在 68 ms 相对下降 0.447043736 mm，pyramidal 为 1.020889861 mm，减少 0.573846125 mm（本轨迹 56.2104%）。elliptic 随后仍下滑并在 4.416 s 重触低桌，两组最终均相对下降约 50.000382 mm。本轮是固定姿态承载诊断，没有下达抬升目标、没有评估或更新正式抓取成功率。

## 冻结输入与唯一配置差异

- 物理/控制基线：`feature/delivery-convergence@1e4d633730f62a398513d33991fa96f9c4977daa`。
- 本分支父版本：`feature/grasp-failure-diagnosis@bf5dc44ab4e97650dfcb0ebdfc550047b87ec947`。
- 对照 A：旧 `diag_free_load_ab_001/B` 的低桌面 pyramidal 原始记录，完整复用，**新增积分 0 步**。避免与旧实验的 A/B 标签混淆，本文使用锥名称。
- 实验 B：新 `diag_cone_elliptic_001`，一次 elliptic 运行，14,100 个真实 `mj_step`，14.1 s；进程退出 0，wall time 18.398 s。Ubuntu 22.04 x86_64 / Python 3.11.9 / MuJoCo 3.2.3 / NumPy 1.26.4，复用已有环境，无安装或依赖改动。

派生诊断 XML 的实际完整差异只有：

```diff
-  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81" />
+  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81" cone="elliptic" />
```

旧模型使用默认 `pyramidal`。对照编译检查 459 个公开模型字段一致，所有其它 option 字段一致；实际 solver=Newton、integrator=implicitfast、impratio=1、dt=0.001 s。质量、摩擦、condim、几何、equality、位置目标、桌面/物体、base/link1 过滤及正式判据均保留。XML 引用旧工作树资产，逐个核对其实际文件与冻结资产字节一致，没有另造权威模型或改正式 MJCF。

原 B 的 model/snapshot/trajectory/contacts/protocol/observations/initial-contacts，以及共同 snapshot 与当次 runner，均核对原 SHA。旧模型 SHA256 `e42b051da4e13dca061b691cee1c8f6231adadb9068fd815697c210660f02ee0`；新模型 `79f5f9116e812687e9c4592f9faf42e9660276f77329297e690be4a0db28d376`。

## 初态与运行语义

直接使用旧 B 已合法生成的 **156 维 `mjSTATE_INTEGRATION`**，t0=`29.128000000012616 s`；本轮不重放原前缀。恢复后、forward 前，完整 integration state、qpos/qvel、时间、ctrl、warmstart 和 controller requested/command 逐元素相等；控制器代码/参数保持冻结版本。完整初态 float64 向量按 C 顺序原始字节的 SHA256：

`38fed316c9733655dcf6f6db655204f69e21d0d4f84db3916808eb1a83cb5d10`

requested/command 的共同 SHA256（float64 原始字节）为 `ac9809da982cc3ce9385b193a989750f745b42426bbd4284679ab6d85ee66f5b`。

forward 后缓存**不同**：约束维度 9→7；qacc/warmstart 最大分量差约 44.693459，qfrc_constraint 最大分量差约 0.051709；qfrc_bias 相等。两份完整 cache NPZ 与差异说明已保存，不能声称所有缓存相等。新旧 initial-state.npz 文件哈希不同，因为文件还包含各自的 post-forward state；它们不替代上述共同 pre-forward 向量哈希。

两组 14,100 条记录的 time、requested、command、physical_target 精确相同。执行器 ctrl 随实际状态反馈而不同，这是相同控制器作用于分岔状态的结果，没有把旧力指令误当位置目标回放。新执行复用原 `Recorder.step`：Controller.apply→mj_step→mj_forward→记录；仅初始化恢复一次，没有积分期间写 qpos、状态重置、外力、附着或隐形支撑。

预言在准备阶段写入并封存：**elapsed 68 ms 时，cube COM 与 gripper_base 原点的世界 Z 间距相对初态下降严格小于 1.000 mm**。`prediction.json` 的 SHA256 为 `26520c7a006eab3f043dc7e81103dc2c597ca1ea760eb6d2ea2ba8a09aa128d8`。所有下表时间均为相对 t0，取样行号为 `round(elapsed/dt)-1`。

## 实际结果

正数表示相对夹爪向下，速度是 cube 的**世界 Z 速度**，不是局部 Z 轴或下降量的差分；相对夹爪的世界 Z 速度也保存在 comparison.json。

| elapsed | pyramidal 相对下降 mm | elliptic 相对下降 mm | pyramidal 世界 vz mm/s | elliptic 世界 vz mm/s |
| --- | ---: | ---: | ---: | ---: |
| 68 ms | 1.020890 | 0.447044 | -32.477955 | -14.626927 |
| 0.751 s | 19.176514 | 6.404821 | -21.738595 | -7.588948 |
| 1.317 s | 39.352011 | 11.440628 | -109.107820 | -7.597367 |
| 1.328 s | 41.219163 | 11.523413 | -210.012008 | -7.490705 |
| 14.1 s | 50.000382 | 50.000382 | ≈0 | ≈0 |

| 事件 | pyramidal | elliptic |
| --- | ---: | ---: |
| 首次相对下降 ≥1 mm | 0.068 s | 0.141 s |
| 首次双指受力不足（任一指 Fn<原阈值 0.05 N） | 1.317 s | 4.419 s |
| 首次失去双指几何接触（任一指无接触） | 1.320 s | 4.419 s |
| 首次 cube/table 几何接触及正向桌面支持力 | 1.329 s | 4.416 s |
| 首次 robot/object 穿透 >原阈值 1 mm | 0.751 s | 2.164 s |
| 首次双指联动误差 >原阈值 0.2 mm | 未观测到 | 4.458 s |

elliptic 在触桌前的下滑速度较小、无桌支撑阶段延长约 3.087 s；**4.419 s 的双指受力不足已发生在触桌后**，不能全部当作独立悬空承载能力。共同无桌区间取 rows `[0:1328)`，至最后样本 1.328 s，严格排除旧 B 首次触桌的 row1328。

共同区间的逐步双指世界 Z 合力平均值：pyramidal `0.488429+0.476066=0.964495 N`；elliptic `0.475650+0.504829=0.980479 N`，桌面支持力两组均为 0，物重 0.981 N。elliptic 接近承重平衡但仍有向下速度，不能以接近物重的均力认定止滑。力日志是 step 后 forward 的瞬时值，不是积分时冲量。

摩擦利用率按**每条正 Fn 接触采样**、实际 condim=3 分别计算：

- pyramidal：`(|Ft1|/mu1+|Ft2|/mu2)/Fn`（L1）。
- elliptic：`sqrt((Ft1/mu1)^2+(Ft2/mu2)^2)/Fn`（L2）。

实际滑动系数 mu1=mu2=1。新组逐步保存 Contact.dim/friction5；旧组依据已验证的冻结模型同优先级、相同系数组合重建，未伪造旧 efc_state。共同无桌窗口 pyramidal 正 Fn 接触采样 2643 条，`|rho−1|≤1e−10` 的 1106 条，中位数 0.937082；elliptic 分别为 2656、12、中位数 0.693892。它们是“步×接触索引”的采样计数，不能解释为独立接触点、时间比例或原求解器 stick/slip 标签。每组各自完整无桌窗口长度不同，其统计也另存，不能混作同长对照。

控制/数值记录：新组状态全部有限，无数值警告、外力或超执行器限幅；最大臂目标误差 0.0027215 rad。最大 robot/object 穿透 pyramidal 2.830960 mm、elliptic 1.958534 mm，均超原 1 mm 标准。新组最大联动误差 0.258720 mm>0.2 mm，故冻结 Recorder 给出的 `control_health=FAIL` 保留；首次超限在触桌后。进程退出 0 只表示运行完整完成，**不是**控制验收或抓取 PASS。末样本两组双指力为 0，桌面 Fz=0.981 N，物体由桌面承重。

## 命令、证据与审查

工作树：`/data_robot/projects/.worktrees/nero-gripper-elliptic-ab`，分支 `feature/grasp-elliptic-ab`。Python 命令前缀为 `/data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python`。

```bash
python experiments/dynamics/grasp_cone_ab.py prepare --source-dir /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/experiments/dynamics/runs/diag_free_load_ab_001/B --output-dir experiments/dynamics/runs/diag_cone_elliptic_001
python experiments/dynamics/grasp_cone_ab.py run --output-dir experiments/dynamics/runs/diag_cone_elliptic_001
python experiments/dynamics/diagnose_cone_ab.py --pyramidal /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/experiments/dynamics/runs/diag_free_load_ab_001/B --elliptic experiments/dynamics/runs/diag_cone_elliptic_001 --output-dir debug/issue8/analysis_001
python -m unittest discover -s experiments/dynamics -p test_grasp_cone_ab.py -v
python -m unittest discover -s experiments/dynamics -p 'test_diagnose*.py' -v
```

以上实际命令均退出 0；5 项针对单一差异、源哈希和拒绝覆盖的检查、17 项既有保存记录 fixture 检查通过，全部新增测试积分为 0。开发初次 5 项检查因实现不存在而失败；随后 XML 空白 fixture 发现一处拼接问题，已在准备/物理执行前修正，原日志保留。未运行会增加物理实验的全量仿真测试，未新增 CI/强制检查流程。准备目录排他创建，运行 STARTED 排他创建并拒绝覆盖已有输出。

原始新 Run：`experiments/dynamics/runs/diag_cone_elliptic_001/`，含 model.diff、prepared/prediction/protocol、初态、双方 post-forward cache、当次 runner 源码、trajectory、contacts、contact-parameters、observations、warnings、execution、result。旧 Run 保留原处。本次实际运行 runner SHA256 `b0b9e3e4c087b6c072160f78a511bc3c3184730e22fd5ce37ef752160cd4350e`。

| 原始文件 | SHA256 |
| --- | --- |
| 旧 trajectory.npz | `7eb0976355caba3bc3d940e7026f36f93e75bb60536d6c74f1f42cf3d792942d` |
| 旧 contacts.jsonl | `243c3ed224e5766f0ec17254c5144e51805dd4ff256c1e136d32b7c92529ee1c` |
| 新 trajectory.npz | `fea51467c8001321dddea0f16eb85bf60f9184f2fdb2d507fd9a1464dfad948d` |
| 新 contacts.jsonl | `89aeb60697d7c6badde163965439b2b2fe37bffc02845b1a41d36a33ec5d43b5` |

`debug/issue8/` 保留 Issue、命令/退出码、日志、`state-and-common-summary.json`、`analysis_001/{comparison.json,episodes.json,contact-utilization.csv,*-series.npz}`、`final-verification.json`、review.md 与 evidence-manifest.json。SHA 清单含大文件本地路径；不提交大日志或复制资产。

独立只读审查未发现 Critical/Important 或阻挡发布的 Minor 缺陷；核对了唯一差异、冻结状态、原始数组、事件和 L1/L2 计算，没有重跑积分。审查范围不包含随后撰写的本报告和远端发布，发布及报告另行按记录核对；审查结论只支持独立 Draft PR，不代表合并批准。原 main/delivery/诊断分支与既有工作区均未改变，PR #3 保持 Draft。

## 推断边界与下一步

本次确定性对照支持“cone 配置会显著影响这一初态的早期下滑和触桌时刻”，但不支持统计总体显著性、正式抓取成功或实机可靠性。cone 是全局选项，同时改变所有接触的约束表示及软接触动力学；没有隔离锥截面几何、软约束、控制器响应和接触迁移各自的贡献，也未证明摩擦锥是唯一失败根因。

本轮不继续调参。下一步可先只读分析 elliptic 触桌前记录中的约束/软接触与剩余下滑，解释“合力接近物重却仍下降”的时序；任何新增动力学干预需另行批准。本 Issue 的单变量预测检验已经完成，持续承载问题仍未解决。
