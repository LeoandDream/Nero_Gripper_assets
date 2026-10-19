# Issue #23：NERO 源几何 fingertip pads 单候选验收

2026-10-19，Ubuntu 22.04。**Gate 0 = UNKNOWN / STOP（`WORKFACE_ISOLATION_UNKNOWN`）；Gate 1 = NOT_RUN。** 首个查询姿态 R1 row23622 已出现两指 pad 与保留护体同时接触同一 cube，尚不能证明工作面已与原代理隔离，按预注册停止。没有调整候选或继续其他姿态，新增物理 Episode、积分步、力求解和控制器执行均为 **0**。不能称 B 抓取成功、失败或已改善。

## 冻结与来源

独立分支 `feature/nero-fingertip-pads-ab`，基线为 Issue 指定的 `feature/panda-gripper-reference@d4a37dafb9a7b9cb3392a8937590dd091a5e4a7a`。控制器、config、grasp protocol 和批准过滤文件逐字节核对 `1e4d633730f62a398513d33991fa96f9c4977daa`。原 R0/R1 模型 SHA 为 `05b364b0696e0afa90bc39a07bcc46e655933ba14037fc5e1cf26108e6249492`。Issue #18 的 152 项历史输入重新验 SHA，通过后纳入本轮共 179 项输入封存；运行后再次全通过。正式模型、源 mesh 和旧 Run 未改。

唯一候选在首次查询前提交为 **`db78a9649854ba6d4cb6f607ae06c9192c49a445`**，查询时工作树干净。候选 SHA：

`12d0c170c53319917704a07f4756908aba778c4467f850d9c1775201616d018c`

见 [冻结候选](issue23-candidate.json)、[预注册](issue23-evidence/preregistration.json)、[派生模型 diff](issue23-evidence/candidate.diff)。只有一版实际生成的候选，未扫描、重试或根据查询结果移动 pad。Panda/robosuite 仅沿用 [#21 源码审查](panda-gripper-reference.md) 的工作面与护体分离思路；尺寸和材料没有从 Panda 复制。

## pad 怎样从 NERO 得到

机械来源为 `agilexrobotics/agx_arm_urdf@a68b8e876a2c6fcb0ecee7a7943129e4d48dd864`。源 STL、DAE、转换后的 visual/collision OBJ 逐项核对。mesh 坐标与 finger body 局部坐标一致；**local y=0 是指尖，y≈−76.5mm 是指根，local −z 朝夹持内侧**。左右源三角片最大坐标差约 `1.77e−18m`，源到 visual 顶点对应误差小于 `9.58e−8m`。

| 源工作面 | 零基 STL / visual OBJ 三角片 | 局部范围与外法向 | 冻结 pad 全尺寸（mm） |
|---|---|---|---|
| 指尖短平面 | 545–548 / 2082–2085 | y=−10.022178..0mm，z≈0；n=(0,0,−1) | 20 × 10.022178 × 2 |
| 内侧斜面 | 549–556 / 545–552 | y=−30.335130..−10.022178mm，z=1.805596..0mm；n≈(0,−.088540,−.996073) | 25.429763 × 20.393043 × 2 |

逐片点集、局部切向基、拟合误差、source/visual 对应编号见 [源几何](issue23-evidence/source-geometry.json)。斜面最大平面拟合残差约 `5.14e−11m`。短平面虽是单独的二维连通片，但与主体边界重合，向后能找到约2mm实体背板；斜面背面独立核对也支持2mm法向厚度，不能把短平面误判为悬空装饰片。

每面取一个全长、最窄端宽度的**前表面内接矩形**，朝实体内侧挤出2mm。原面为渐宽多边形，因此不声称矩形覆盖全部真实工作面。斜面 box 两个背角相对源侧壁约有 **37.06μm** 外伸，仍在原000凸包内；这是一项已记录的近似，不能称整个 box 严格内接源实体。两个 pad 的背部在折线附近有小内部楔形交叠，也不能称 pad 体积完全互斥。

`collision_000` 覆盖主体及短平面/斜面，是目前实际承担抓取的代理；其凸包填平了凹处，浅斜面不等于真实源斜面。`collision_001` 是指根盒状连通分量，约 y=−76.5..−70mm，保留不变。不是把001当作指尖。

历史只读归约覆盖 Issue #18 冻结的 **422 姿态 / 844 contacts**：全部落在两指000，001为0。代表点最近 visual 区域为短平面683、斜面160、侧边/其它1。这里的“最近三角片”不是引擎实际 feature 或材料见证点，未用这些代表点反向拟合 pad；#18 已存 native 结果也没有混入本轮 B。[历史归约摘要](issue23-evidence/historical-contact-summary.json)。

## 护体组织与静态结果

候选只在派生 B.xml 中移除两指原000 geom：从其凸包减去两个向工作面外侧开放的矩形棱柱，在对应位置放入上述 box pad，差集分成每指12个凸护体；原001、visual、惯性、关节、equality、actuator均保留。没有改过滤。独立纯几何审查每指16384点无差集成员错误，体积独立计算残差约 `1.46e−18m³`，护体之间无正体积交叠；这些核对结果支持分割实现与定义一致，**不证明接触隔离或所有极限姿态安全**。[候选几何复核](issue23-evidence/candidate-review.json)。

A/B 编译后核对75项关节、驱动、equality和机械字段，以及全部 option；非目标 geom 及新 geom 的有效接触材料核对相同。

实际只查询 **R1 row23622**，保存 qpos SHA 与 #18 预注册相同。A 先查询并通过历史几何复现，才查询 B。全部返回点、dist、法向有限，qpos 未变、warnings 为0，time/qvel/ctrl/constraint state 未进入动力学。

| row23622 | A：原表示 | B：冻结 pads 表示 |
|---|---:|---:|
| 全部 contact 数 | 2 | 23 |
| 指1 pad / 护体 contact 数 | 0 / 原000为1 | 6 / 4 |
| 指2 pad / 护体 contact 数 | 0 / 原000为1 | 6 / 7 |
| 最大 robot/cube 穿透 | 0.731115mm | 0.731175mm |
| >1mm contact 数 | 0 | 0 |

A 的 dist、point、oriented normal 与历史结果误差均为0。B 的点/法向及每个 geom pair 的全部记录见 [原始查询](issue23-evidence/queries.jsonl)，**不按数组索引把2个与23个 contacts 强行配对**。

保留的 guard2/6 包含短平面矩形两侧的原000楔区：x约10..12.715mm及−12.715..−10mm，y约−10.022..0mm。投回已保存 finger pose 后，其代表点 z约0.116/0.121mm，朝 cube 的局部法向 z约−.999639，支持“原朝内侧缘仍在参与夹持”的解释。代表点不能定位实际材料 feature，因而 **pad+guard 共存不等于已经证明同材料点重复约束或重复受力**；但未解释的工作面隔离问题足以使 Gate 0 停止。[独立静态结果复核](issue23-evidence/static-review.json)。

该姿态没有报告非 cube 的新增接触。仍未验证 open/close 极限、自碰、完整非工作面防碰覆盖及各姿态净间隙；上层8D命令和关节定义保留，但不能因此声称真实净间隙已经标定。由于第一帧停止，R1其余154帧（包括23623/24、25984–25991、正常controls及完整超限邻帧集合）、R0全部267帧及极限姿态 **NOT_RUN**。没有将部分静态结果写成 Gate 0 PASS。

## 运行与复核

Python3.11.9 / MuJoCo3.2.3 / NumPy1.26.4。运行前归档实际导入的三个项目模块、脚本、A/B模型、全部引用资产及config/protocol等 **192项文件**，记录 SHA、Git HEAD/status、环境版本；大原始 NPZ/JSONL 保持原位，以既有 SHA 核验。启动时核验 helper 来源；每次查询前检查模块字节及新增项目模块。静态查询没有导入 Controller 或执行抓取 helper，不能把本轮封存冒称已补齐历史 helper 的执行期身份。

API审计：`mj_kinematics=2`、`mj_collision=2`，仅另有名称查询，禁止API尝试为0。没有 `mj_step`、完整 `mj_forward`、约束构建、接触力求解或控制器。Gate 1 未启动，故没有新 A/B 抓取轨迹，也没有新的 hold/漂移/支持率/耦合/饱和等物理指标。原80mm/2s/10mm/95%/1mm全标准保持，候选完整抓取结果 **UNKNOWN_NOT_RUN**。

Ubuntu 本地完整证据根目录：

`/data_robot/projects/.worktrees/nero-gripper-fingertip-pads/debug/issue23/`

其中 `candidate-001/static/{sealed-inputs,executed-source}` 是执行前归档；旧 Run 仍在 `nero-gripper-full-grasp/experiments/dynamics/runs/issue14_full_grasp_002`。精简 [机器摘要](issue23-evidence/summary.json)、[执行前环境](issue23-evidence/runtime-before.json)、[归档收据](issue23-evidence/archive-receipt.json)、[命令及退出码](issue23-evidence/commands.json)、[运行后输入/10个工作树完整性](issue23-evidence/integrity-after.json) 可在线复核。

只重跑必要纯几何验证时：

```bash
cd /data_robot/projects/.worktrees/nero-gripper-fingertip-pads
PY=/data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python
"$PY" -B -m unittest discover -s experiments/dynamics -p test_fingertip_pads.py -v
"$PY" -B experiments/dynamics/inspect_fingertip_sources.py --output debug/issue23/source-review-new
```

3项解析几何测试通过；发布的源面提取脚本执行退出0，关键源几何字段与冻结 basis 完全一致。必要的候选生成/静态查询命令已保存，但**不重跑已停止的 campaign，不生成或调整第二版候选**。[源码审查](issue23-evidence/code-review.json) 未发现未解决的关键/重要问题；这不是 Gate 0 PASS。

## 结论与唯一下一步

本轮建立了可追溯的 NERO 工作面、冻结唯一候选，并用实际静态查询发现其隔离组织仍有关键 UNKNOWN。它解决了“pad尺寸从哪里来、000/001各做什么、能否直接进入动力学”的问题；没有解决抓取，也没有证明 pads 无效或所有1–5 box方案不可能。

**下一步只建议另行审查 taper 侧缘的工作面/护体归属，先明确能避免原朝内侧缘与 pad 未解释共存的几何边界，再决定是否授权下一版候选。** 本轮不继续查询、改 pad、调物理或运行动力学。分支保持 Draft，不合并 main / PR #22，交独立审查。
