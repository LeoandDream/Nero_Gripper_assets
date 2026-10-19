# Issue #16：离线接触几何审查

2026-10-19，Ubuntu 实际只读执行。**ANALYSIS PASS（保存输入范围）；PHYSICAL GRASP FAIL；ROOT CAUSE UNKNOWN，碰撞检测输出敏感性获得部分支持。** 本轮新增 Episode / `mj_step*` / 接触再生成 / 力求解全部 **0**，未改模型、参数、动作、初态、原判据或旧 Run；不合并。

## 冻结与输入边界

独立工作树 `/data_robot/projects/.worktrees/nero-gripper-contact-geometry`，分支 `feature/contact-geometry-readonly`，精确派生自 `feature/bounded-full-grasp@1bd91cc4c3fc7723eab94881ceb7c4cc3e655316`。已读 Issue #16、#14、PR #15 及其最新独立审查；物理/控制/标准仍为 `1e4d633730f62a398513d33991fa96f9c4977daa`。

保存输入一致性 **PASS**：405 次引用核验对应 **261 个唯一路径**（144 次重复引用），无缺失/不一致；prepared 的25项产物、86项引用资产、全部62个碰撞OBJ及原执行快照均匹配。原R0/R1各18个文件保留，前后SHA不变；分析脚本另逐项封存/复核149个核心输入。R0/R1模型同字节，SHA `05b364b0696e0afa90bc39a07bcc46e655933ba14037fc5e1cf26108e6249492`。

**历史执行身份仍有 UNKNOWN**：`grasp_support_ab.py` / `grasp_cone_ab.py` 没有运行期独立快照/hash；当前字节与冻结源及两次执行记录的Git版本一致，不能补证当时是否存在未提交修改。该边界不阻断对已保存模型/状态/接触的几何审查，不能声称完整历史执行来源已闭合。

## 三帧：相同 geom 对，入口法线突跳

六帧实际均为 `cube_geom`（body `cube`，box）与 `gripper_link2_collision_000`（body `gripper_link2`，mesh，同名碰撞代理）；每帧共2个接触，指2为 event index **1**，未换geom。以下穿透均直接保留原 `-dist`；SAT是独立凸几何的最小平移深度，**不替代原判据**。

| Run / 零基row | 原穿透mm | 对cube法线世界Z | 独立凸几何深度mm | 几何最近源三角片 |
| --- | ---: | ---: | ---: | ---: |
| R1 23622 | .731115 | +.026813 | .731175 | 226 |
| R1 **23623** | **1.061109** | **−.594889** | .731783 | 226 |
| R1 23624 | .732999 | +.027047 | .732999 | 225 |
| R0 33317 | .825624 | +.032522 | .825624 | 225 |
| R0 **33318** | **1.054967** | **−.299733** | .829786 | 226 |
| R0 33319 | .834871 | +.032588 | .835328 | 225 |

R1入口23622→23623：法线转 **46.418812°**，接触点世界位移仅 **.00058053mm**，cube平移.002366mm、转.003829°，指2平移.000305mm、转.0000682°。接触点cube-local约 `[.139,−19.507,.139]mm`，在侧面中部；finger-local约 `[.095,−10.917,.263]mm`。两帧最近同一源片226，候选片法向 `[0,−.032781,−.999463]` 不变，异常保存法向在finger系为 `[-.382881,+.596202,−.705652]`；它与**任一**凸包面法向最小夹角31.335°。因此没有证据把入口突跳直接解释为近邻表面换面；但不能排除边/顶点组合或内部支持点选择。

R1出口23623→23624法线反转46.435228°，接触点却移动 **12.313mm**；只能说**异常入口前两帧**几乎不动。R0入口法线转38.147712°，接触点移动27.376mm，且cube-local Z≈19.437mm接近上缘；不能假定与R1有同一材料点或同一局部机制。两次独立几何深度均连续、未对应原记录的首次超限尖峰。

完整XYZ、双方局部点/法向、pose差、材料点相对切向速度和原event见[关键帧机器证据](issue16-evidence/summary.json)。切向速度由保存qvel与纯运动学Jacobian计算；六帧指2分别为R1 **3.630/2.732/10.504mm/s**、R0 **13.122/11.365/14.177mm/s**，不是接触采样点的跨帧差分速度。**实际 collision feature / witness / portal ID 均未保存，UNKNOWN**；最近片编号只是源OBJ零基几何候选。

## 全段：重复现象，但不能解释全部超限

| 原记录统计 | R0 | R1 |
| --- | ---: | ---: |
| >1mm 接触数 / 不同物理行 | 156 /156 | 93 /93 |
| 连续区间；单帧 /持续 | 66；31 /35 | 34；17 /17 |
| 最长区间 | 35488–35498，11行 | 26810–26821，12行 |
| 指1 /指2超限接触数 | 1 /155 | 6 /87 |
| 阶段 | 全部lift | lift37 /hold56 |
| 最大原穿透 | 1.820234mm，row35505 | 1.791856mm，row26800 |
| ≥30°相邻法线转换；两端均未超限 | 154；37 | 68；12 |
| 超限接触前/后有≥30°转换 | 90 /156 | 41 /93 |
| 上述超限中独立凸几何仍>1mm | 14 /156 | **8 /93** |

每个超限物理行恰有1个超限contact，计数相同不代表独立物理事件。转换仅比较**相邻行、同geom对、双方唯一contact**，不以index追踪材料点；本数据无多接触匹配歧义。10°/30°/60°只作并列描述，未平滑/删除样本，未改1mm标准。[全部249条超限及几何对照](issue16-evidence/exceedances.csv)、[连续区间](issue16-evidence/intervals.csv)（含全体与geom/phase两种scope）、[全部≥10°转换索引](issue16-evidence/normal-jumps.csv)支持逐行复查。

**关键反证：R1 rows25984–25991连续8行**的独立凸几何深度也达1.003346–1.010226mm，保存法向仍接近侧面法向。原最大1.791856mm对应独立深度.891845mm，但不能把全部93条归为尖峰，也不能以SAT替原距离删除85条。原抬升/漂移等失败仍保留。

## 归因与唯一下一步

| 可能原因 | 状态 | 支持、反证与边界 |
| --- | --- | --- |
| 局部凸代理/面边顶点切换 | **UNKNOWN** | 反复回跳存在；R1入口最近片及局部面法向不变。缺真实feature/support ID，不能宣布碰撞代理错误或面切换已证实。 |
| 碰撞检测方向/深度选择 | **SUPPORTED，部分** | 两种独立凸几何构造一致，原法线/深度仍突跳。冻结`nativeccd=false`，版本源码的相应凸碰撞路径返回MPR结果；尚未做同姿态算法对照，不能宣布MPR bug或其为唯一根因。 |
| 坐标/符号/记录处理错误 | **CONTRADICTED，限已核字段** | 全段frame/行号/时间一致；所分析420帧的独立FK/刚体速度复算一致。不证明未保存内部数据或历史helper身份。 |
| 软约束、累计滑移及其它机制 | **UNKNOWN** | 真实凸几何重叠、连续8行超限和高度/漂移失败仍在。约束力求解不直接生成固定姿态的碰撞法线；缺因果干预，不能分离各贡献。 |

唯一建议：**另行授权同一批保存姿态下MPR与native GJK/EPA的碰撞查询单变量对照**，仍为零积分、零力求解，几何/状态全部固定，先检查检测器返回方向/深度，优先于改mesh或调solver。本轮没有执行该干预；即使解释跳变，也不能自动消除连续8行几何超限或把抓取改判PASS。

依据：[MuJoCo3.2.3 contact字段/符号](https://mujoco.readthedocs.io/en/3.2.3/APIreference/APItypes.html#mjcontact)、[mesh凸包与编译坐标](https://mujoco.readthedocs.io/en/3.2.3/XMLreference.html#asset-mesh)、[同版本凸碰撞实现](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_collision_convex.c)。这些解释实现路径，不证明实际选中了哪个特征；力只读自原post-step forward日志，不是积分冲量。

## 验证与复核入口

两个脚本只新增离线分析：[contact_geometry.py](../experiments/dynamics/contact_geometry.py)、[contact_patterns.py](../experiments/dynamics/contact_patterns.py)。**6项几何＋6项模式fixture均退出0**。几何分析一次初版与一次字段澄清/独立FK校验后的最终版均退出0；未覆盖，数值共有101,561个字段一致。最终420帧/840个接触，engine仅允许kinematics/comPos/jac及名称查询，其余`mj_*`入口拒绝调用。

独立Reviewer用Minkowski差凸包复核506个关键/超限接触，深度与SAT差≤8.7e−18m；用另一套XML刚体链/速度和核对420帧，速度差≤1.12e−16m/s。源mesh到实际编译坐标双向点集偏差≤3.47nm，不能解释R1首次约.329326mm差异；近似float32重建不冒称完整编译器复现。审查未发现未解决的Critical/Important计算错误；后续交ChatGPT独立验收。

实际cwd为本工作树，Python复用 `/data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python`，Ubuntu22.04 / Python3.11.9 / MuJoCo3.2.3 / NumPy1.26.4 / SciPy1.15.3 / trimesh4.8.3，无安装。实际完整命令、退出码、读取操作、脚本SHA与本地证据见[manifest](issue16-evidence/manifest.json)。复跑仅执行两个分析脚本或对应`unittest discover`，为`--output-dir`指定新空目录；拒绝覆盖。

原Run仍在 `/data_robot/projects/.worktrees/nero-gripper-full-grasp/experiments/dynamics/runs/issue14_full_grasp_002/{R0,R1}/`；完整派生结果、命令、审计和独立复核在本树 `debug/issue16/`。大Run及5.8MB完整几何记录不入Git。仅新Draft PR，以`feature/bounded-full-grasp`为base；PR #15/main及所有既有Run不改、不合并。
