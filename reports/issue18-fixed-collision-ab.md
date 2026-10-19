# Issue #18：固定历史姿态 MPR / native GJK-EPA 碰撞查询

2026-10-19，Ubuntu实际执行。**QUERY REPRODUCTION PASS；H1 collision-algorithm sensitivity PASS；PHYSICAL GRASP仍FAIL；ROOT CAUSE UNKNOWN（检测路径对保存尖峰的贡献获支持）；MODEL CHANGE AUTHORIZATION NONE。** 这是固定姿态查询，不是新的抓取：新增Episode、`mj_step*`、`mj_forward*`、约束构建/求解、接触力求解及控制器执行均为 **0**。

## 冻结与先后顺序

独立分支 `feature/fixed-state-collision-ab` / 工作树 `/data_robot/projects/.worktrees/nero-gripper-collision-query`，从指定 `53c99982ffb9d5b8cec51849dba0a33e2f9b751e` 派生；已读Issue #18、#14、#16及PR #17最新审查。按#14/#16清单核验 **398次引用 /191唯一路径**，全部一致且后验不变；原Run各18文件、86项引用资产（含62个碰撞mesh）保持原位。R0/R1原XML同SHA `05b364b0696e0afa90bc39a07bcc46e655933ba14037fc5e1cf26108e6249492`。历史两个helper缺执行期独立快照的身份限制仍为 **UNKNOWN**，不补证完整历史来源闭合。

1. 先按原始记录封存全部249个历史超限行及±1邻行、关键六帧；另按预先规则选定同lift正常帧 **R1 23620/23621**（同指2 geom对、原穿透<.9mm、相邻法线变化<30°）。最终 **R0 267 /R1 155，共422个唯一姿态**。
2. Gate0只查询MPR：R1关键三帧、25984/25985、两个正常帧，以及R0关键三帧，共 **10行/20 contacts**。`dist/point/oriented normal`最大误差均 **0**，通过固定容差1e−9m /1e−8m /1e−5°，才继续。
3. 在任何native查询前封存H1/H2：[预注册、完整行集/qpos SHA及Gate0原证据](issue18-evidence/preregistration.json)。预测原件SHA **`dc96f8805e00dcbf2ed7f0e8891a057418c5ca3a451477f411d0182885077852`**；提交 **`6fa15f756f6fcfc29961c9b477a09f5843e9358e`**（10:49:05 +08）先于02:50:11 UTC启动的A/B阶段及首次B查询。
4. 再对封存集合执行 **422次A＋422次B**。全部A仍逐接触复现原记录，844 contacts的三项最大误差均0。总计 **legacy432 /native422＝854次`mj_collision`**，没有查询集合外姿态、扫参或重跑物理。

每次新建独立`MjData`，只复制保存qpos，调用 **`mj_kinematics → mj_collision`**；查询时time=0、nefc=0、scratch qvel/ctrl/constraint force为0。只读几何，不读/重算接触力。B通过`mj_copyModel`语义独立复制，只把`enableflags:0→64`（NATIVECCD）；**462个公开可比较模型字段及其余28个option字段一致**，原XML/mesh未写。允许入口之外的`mj_*`默认拒绝，实际forbidden attempts=0。

## H1 / H2：主结果与反证

| 指2接触，零基row | A/MPR穿透mm | B/native穿透mm |
| --- | ---: | ---: |
| R1 23622 | .731115 | .731115 |
| R1 **23623** | **1.061109** | **.731472** |
| R1 23624 | .732999 | .732999 |
| R0 33317 | .825624 | .825624 |
| R0 **33318** | **1.054967** | **.829786** |
| R0 33319 | .834871 | .834977 |

H1严格使用事前AND：R1 row23623相对#16独立MTD参考 **.731783mm** 的误差，从A的 **.329326mm** 降至B的 **.000311mm**（降低 **99.905517%≥50%**）；B的23622→23623法线跳变 **.002815953°<20°**，A为46.418812°。**两项均满足，H1 PASS**；不等于native是真值、MPR是bug或抓取已修复。

H2是边界观察：**R1 rows25984–25991八行，B全部仍>1mm**，范围 **1.003346–1.010226mm**，与A几乎一致。没有用SAT或native结果替代原判据；这些反证不能被尖峰解释消除。

| 仅冻结查询集合 | R0 | R1 |
| --- | ---: | ---: |
| 姿态 /对应robot-cube contacts | 267 /534 | 155 /310 |
| 原记录及fresh-A >1mm行/接触数 | 156 /156 | 93 /93 |
| B >1mm行/接触数 | 14 /14 | 8 /8 |
| A→B消除 /新增超限行 | 142 /0 | 85 /0 |
| A /B最大报告穿透mm | 1.820234 /1.017191 | 1.791856 /1.010226 |
| pair/contact-count变化；对应UNKNOWN | 0；0 | 0；0 |

全体844 contacts中，距离变化超原复现容差325条、法线变化超容差326条；**接触点844条都改变**（最小.280749mm，最大19.877965mm）。两个正常指2 control的距离/法线在容差内一致，但返回点也分别移动约 **.965254/.965019mm**，不能声称native只改尖峰。它们是检测器返回的代表点变化，**不是材料点轨迹或物体实际移动**。所有方向统一为字典序第一geom→第二geom（此处cube→finger）；实际collision feature/portal ID仍UNKNOWN。比较器的measurement FAIL仅指几何变化超容差，不代表查询流程或H1失败。

## 排除范围、遗留与唯一建议

相同qpos、同资产/参数，fresh-A对历史全部复现而只换检测路径即可改变输出，支持**碰撞算法选择参与这些尖峰的数值表现**；排除了本查询中输入变化、行号/坐标解释或缺少完整forward造成差异的解释。没有识别内部support/portal机制，不能宣布MPR缺陷或错误碰撞代理为唯一根因；真实凸代理重叠仍存在，也没有测量native真实动力学下的力、滑移、抬升或保持。**原抓取FAIL不变，正式MJCF不改。**

唯一建议：**另开并授权一组有界完整R1动力学A/B，唯一变量仍为nativeccd，先完整封存所有运行依赖，按原全部标准验收。** 当前结果只能支持开展这项验证，不能代替验证；本Issue没有授权或执行该步骤，也不将native配置合入正式资产。

## 复现与证据

Ubuntu22.04 / Python3.11.9 / MuJoCo3.2.3 / NumPy1.26.4 / SciPy1.15.3，复用现有环境，无安装。[查询脚本](../experiments/dynamics/fixed_collision_query.py)的实际顺序为`prepare → gate0 → preregister → ab`；[纯保存数据reducer](../experiments/dynamics/diagnose_fixed_collision.py)不导入MuJoCo。最终各命令退出0；5项边界＋8项匹配＋4项reducer fixture通过，全部为零真实查询测试。原有12项离线fixture也通过，未运行任何会积分的全量测试或新增CI。

运行前独立审查修正了新wrapper的代码封印绑定与持久STOP，两个未使用准备草稿query-001/002均0查询且保留；最终只对query-003执行一次Gate0和一次A/B。没有因科学结果重试、调整API或阈值。独立Reviewer直接读取原始NPZ/JSONL和新查询记录，核对422个qpos SHA、全部844对应、H1/H2和统计；再逐条核对提交CSV及16个关键姿态，未发现未解决Critical/Important。此为Ubuntu会话审查，不冒称ChatGPT已亲自读取大Run。

- [精简机器结果](issue18-evidence/summary.json)、[全部844对应的archive/A/B几何CSV](issue18-evidence/contacts.csv)、[关键六帧＋八行H2＋正常帧三方几何](issue18-evidence/key-geometry.json)、[版本/原始路径/SHA/实际命令/退出码/调用计数](issue18-evidence/manifest.json)。
- 原Run：`/data_robot/projects/.worktrees/nero-gripper-full-grasp/experiments/dynamics/runs/issue14_full_grasp_002/{R0,R1}/`。新查询与执行源码快照：本工作树`debug/issue18/query-003/{gate0,ab}/`；完整派生分析和审查：`debug/issue18/{reducer,result-review,query-review,input-audit}/`。大Run不复制入Git，原Run与既有分支均未改。
- 最小API和复制语义参考：[3.2.3 API](https://mujoco.readthedocs.io/en/3.2.3/APIreference/APIfunctions.html#mj-collision)、[碰撞driver](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_collision_driver.c)、[凸碰撞路径](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_collision_convex.c)、[Python model复制实现](https://github.com/google-deepmind/mujoco/blob/3.2.3/python/mujoco/structs.cc)。内部清空失效efc元数据不是约束构建或力求解。

仅独立Draft PR，base=`feature/contact-geometry-readonly`；PR #17与main不改、不合并，等待ChatGPT独立验收。
