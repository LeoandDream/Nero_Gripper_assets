# Issue #2：冻结抓取失败诊断

2026-10-17 更新：已批准的固定姿态自由承载A/B见第6节；随后获准的只读承载预算见第7节。第1–5节保留首轮诊断记录。

## 1. 固定版本与环境 — PASS

Ubuntu 22.04 x86_64，Python 3.11.9，MuJoCo 3.2.3，NumPy 1.26.4，SciPy 1.15.3。
独立 worktree：`/data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis`；仿真运行时 HEAD 为 `1e4d633730f62a398513d33991fa96f9c4977daa`。
原 153 项 SHA 与四份实际执行源码均核验一致。仅新增诊断脚本、测试及本次证据说明；未改原模型、物体、增益、摩擦/接触/equality、过滤或成功标准，未访问机器人硬件。`main`、`feature/delivery-convergence` 均未改。

## 2. 实际命令与状态

| 执行 | 实际结果 |
|---|---|
| `.venv/bin/python scripts/test.py`（新增诊断器前） | PASS，静态9+动态12，含真实37,000步空载控制测试 |
| `.venv/bin/python experiments/dynamics/run.py --stage grasp --filtered --run-id diag_baseline_001` | FAIL，退出1；49,387个真实 `mj_step`，无程序异常；仅fixed进入物理执行 |
| `.venv/bin/python scripts/test.py`（诊断器审查修正后） | PASS，退出0，静态9+动态20，共29项；其中新增诊断器8项 |
| `.venv/bin/python experiments/dynamics/diagnose.py --episode-dir experiments/dynamics/runs/diag_baseline_001/fixed --output-dir debug/issue2/analysis-final` | PASS，退出0，49,387行NPZ/JSONL对齐；输出JSON与阶段CSV，现存目录拒绝覆盖 |
| `.venv/bin/python debug/issue2/hold_only_probe.py` | PASS（仅诊断执行），`diag_hold_only_002`，43,228步；未评估正式抓取成功 |
| 离线FK/三关键帧 | PASS，读取已保存状态，零 `mj_step`；`grasp_center`仅为非物理规划site |

首轮辅助保持对照 `diag_hold_only_001` 因逐行重复解压NPZ过慢，由操作者中断于454步，退出130；原记录及源码保留，不计为完成对照。缓存相同数组后重跑为002，不改变任何物理/控制/判据。仅一组控制变量条件，无参数扫描。完整命令、依赖和真实退出码见下列证据目录。

## 3. 阶段性接触证据

行号均从0开始，JSONL行号为行号+1。接触力为每步后 `mj_forward` 重算的瞬时值，不是积分平均力。

| 阶段 | 首/末采样时刻s | 双指均≥0.05N比例 | 桌面接触比例 | 核查结果 |
|---|---|---:|---:|---|
| open | 0.001–1.847 | 0 | 100% | 无手指/物体接触 |
| approach | 1.848–15.947 | 0 | 100% | 无手指/物体接触 |
| close | 15.948–28.128 | 1.16% | 100% | 27.952s开始接触；末行双侧达到0.2N触发 |
| grasp_settle | 28.129–29.128 | 100% | 100% | Fn末值1.059/1.071N，桌面仍承重 |
| lift | 29.129–43.228 | 31.60% | 98.55% | 手上移、cube不随动；接触迁移/穿透，继而失指 |
| hold | 43.229–45.228 | 0 | 100% | 双指Fn=0，桌面向上力≈0.981N |
| release | 45.229–49.387 | 0 | 100% | 物体仍在桌面 |

| 事件 | 时间s / 行 | 证据 |
|---|---|---|
| 首次双指触发 | 28.128 / 28127 | Fn=[0.207216,0.210574]N；该判定只要求单一样本，不认证稳定承载 |
| 提升开始 | 29.129 / 29128 | 进入提升阶段；cube仍在桌上 |
| cube相对夹爪位移超过1mm | 29.558 / 29557 | 相对settle末：规划点上升1.210mm，cube仅上升0.208mm；此1mm仅描述性诊断标记 |
| 首次短暂离桌接触 | 29.579 / 29578 | 指向上力合计0.680N<重量0.981N；1ms后桌接触重现，全程最长离桌接触段仅10ms |
| 首次穿透超过1mm | 30.074 / 30073 | 规划点已上升7.162mm、cube仅0.218mm；相对偏移6.944mm |
| 最大穿透1.818mm | 33.519 / 33518 | 接触分布已移至cube上缘：cube局部z从settle约-9.4mm变到+19.5mm（半高20mm）；世界z=0.169585m；指1朝cube法向z=-0.90666，其世界Fz=-0.66987N |
| 首次失去双指有效支持 | 33.584 / 33583 | Fn=[0.047168,0.326726]N，此后未恢复双侧有效支持 |
| 首次耦合残差超过0.2mm | 33.593 / 33592 | 晚于穿透和失去支持；峰值0.212700mm在33.606s |

全程九个执行器的限力饱和样本比例均为0，外力数组均为0。原验收最大抬升仅0.138361mm（相对原始cube中心），非80mm；与settle末已沉降位置比较的高度变化不是同一指标。
第一段提升29.129–31.918s内，94.66%样本仍有桌接触，手指向上合力中位数仅0.442N，桌面向上力中位数0.552N。仅把scalar Fn相加会漏掉实际力的方向。

接触世界力使用 `F_cube = sign * frame.T @ [Fn,Ft1,Ft2]`，cube为第二body时sign=+1，第一body时为-1。全49,387行重构总力与保存的cube平移 `qfrc_constraint` 最大差8.05e-16N，独立复核通过。[MuJoCo 3.2.3源码依据](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_core_smooth.c)

![三个保存状态的离线重建，不是成功抓取证明](issue2-keyframes.png)

图片的行号、时刻与源轨迹SHA在 `issue2-keyframes.json`；图像只说明相对位置，接触/受力结论来自原JSONL。

## 4. 已证实问题与未证实假设

**PASS（诊断事实）**：正常建立双指接触，单帧触发与记录一致；提升时cube不能跟随，持续依赖桌面；接触沿cube侧面迁移至上缘附近，出现斜向/向下法向、穿透超限，随后失去双指支持。执行器饱和、最先发生的耦合超限不支持作为这一轮最早失效解释。

**PASS（单变量对照）**：29,128步提升前缀 `qpos/qvel` 与基线逐步完全相同（最大差0），只将随后14.1s的提升参考替换为末帧原位保持。双侧有效接触比例100%，最大穿透0.757561mm，最大耦合残差0.003296mm，无执行器饱和；不支持“相同夹持状态在等时长原位自行失稳”。该对照中桌面平均承重0.981006N，不能算自由承重或正式抓取成功。

**FAIL（物理抓取）**：fixed未通过，偏移条件未进入物理执行。保持期漂移接近0反映物体停在桌上。

**UNKNOWN（更深根因）**：凸包局部接触几何、软接触/摩擦正则化、抓取接触位姿和提升参考轨迹各自贡献尚未隔离；单组保持对照不能证明其中任一因素是唯一根因。实机真实性仍未知，不据此判整个机械资产无效或认定仿真器bug。

## 5. 首轮唯一建议实验 — 当时 UNKNOWN，现已批准并执行

建议在相同夹爪姿态/命令下做“固定姿态自由承载”对照，隔离桌面承重与提升运动。仅在诊断派生场景中将桌面 `pos z: 0.065 → 0.015m`（top从0.13降到0.08m）。A/B以基线settle末合法到达的同一robot/cube状态作为reset初态，保持相同夹爪命令、重力、质量、摩擦、equality、增益和判据；执行中不覆盖物体状态，记录去掉桌面支持后是否能提供0.981N上托。
该方案会改变诊断场景几何位置。首轮未实施；维护者随后在Issue批准这一项差异，执行结果见第6节。未合并main。

## 证据位置

以下路径相对上述worktree；原始数据均保留且未提交大型轨迹/视频：

- `experiments/dynamics/runs/diag_baseline_001/`：summary、execution、执行源码；`fixed/`下grasp-result、result、trajectory、contacts、model、protocol。
- `experiments/dynamics/runs/diag_hold_only_002/`：对照轨迹/接触/模型/协议及diagnostic-result；`diag_hold_only_001/`保留中断结果、原脚本和原因。
- `debug/issue2/{environment.json,baseline-execution.json,execution-status.json,frozen-tests.log,final-tests-after-review.log}`：版本、真实命令/退出码、测试输出；较早的`final-tests.log`亦保留。
- `debug/issue2/{analysis-final,hold-analysis-final}/`：最终只读诊断JSON与阶段CSV；较早分析目录亦保留。`independent-force-audit.json`为独立受力重构与事件复核；`contact-surface-check.json`记录接触点在cube坐标系的位置。
- `debug/issue2/{offline_pose.py,offline-pose.json,offline-pose.npz}`：离线相对姿态，明确零积分。
- `debug/issue2/{hold_only_probe.py,hold-only-hypothesis.json,evidence-manifest.json}`：单变量假设、复现脚本及原始文件SHA索引。

基线 `fixed/trajectory.npz` SHA256：`a1bc4f26640e1b6575f682de0d2b1f71592865816ea794d3b29b84b9b2c8da49`。
基线 `fixed/contacts.jsonl` SHA256：`7af9cf593dd94662061678dadbee678326a263652e983fbd49d59dfdbd31a56c`。

## 6. 已批准的固定姿态自由承载 A/B

### 6.1 版本、单变量与共同初态 — PASS

沿用上述Ubuntu/Python/MuJoCo环境及冻结物理源码；运行时诊断分支HEAD为 `2c651ca0fd5e5b454dc735a7fa30a6c6b387779c`，新增脚本另存完整源码与SHA。正式资产、原控制器/配置/判据及历史Run未改。

唯一原始XML差异是同一桌面中心z：A=0.065m，B=0.015m。编译后的全部公开模型数组中，仅 `geom_pos[0,2]` 及其派生静态 `bvh_aabb[0,2]` 变化；物理/求解器选项全等，显示边界统计随桌面位置自动变化。差异及初态证据先保存、审核，再执行A/B。

先从基线真实重放29,128步到settle末（row29127，t=29.128s）。qpos/qvel/ctrl/actuator_force/qfrc_constraint逐步最大差均为0。随后A/B各用一次合法reset恢复相同156维 `mjSTATE_INTEGRATION`（含速度、warmstart、ctrl、时间、外力、equality等）及Controller的requested/command；不清零微动，不在执行中写状态。

**状态比对边界**：A/B在初始化 `mj_forward` 前156维逐位相同。引擎随后按各自接触重算并更新15维warmstart：A最大变化1.42e-14，B为54.8843；qpos/qvel仍相同。这是撤去桌面支撑产生的求解响应，不是人工给B不同初值。前后状态均保留，不能把after-forward状态也称为完全相同。

### 6.2 实际执行 — PASS；官方抓取结果未评估

```bash
.venv/bin/python experiments/dynamics/grasp_support_ab.py prepare --source-dir experiments/dynamics/runs/diag_baseline_001/fixed --output-dir experiments/dynamics/runs/diag_free_load_ab_001
.venv/bin/python experiments/dynamics/grasp_support_ab.py seal --output-dir experiments/dynamics/runs/diag_free_load_ab_001
.venv/bin/python experiments/dynamics/grasp_support_ab.py run --output-dir experiments/dynamics/runs/diag_free_load_ab_001
.venv/bin/python experiments/dynamics/diagnose_ab.py --a-dir experiments/dynamics/runs/diag_free_load_ab_001/A --b-dir experiments/dynamics/runs/diag_free_load_ab_001/B --output-dir debug/issue2-ab/analysis
.venv/bin/python scripts/test.py
```

各命令均退出0；A/B各14,100个真实mj_step，固定14.1s，采样29.129–43.228s。连同一次前缀重放，本次诊断共57,328步；没有追加第三组或调参。最终静态9+动态33=42项测试PASS（本次新增runner4项、只读分析9项）。原正式抓取成功率未更新。

prepare结束后审查补齐了两组protocol/model/state的联合执行校验；未重放或改初态。旧prepare源码/原preflight及新执行源码/独立seal均保留。protocol的script SHA对应prepare版本，`reviewed-readiness.json`的run_script SHA对应实际执行版本；所有组的文件在A首步前共同校验。

### 6.3 接触与下落时序 — 记录 PASS；B持续独立承载 FAIL

下表行号均为各组新轨迹的0基行号，JSONL行号=行号+1；elapsed相对共同初态29.128s。vz<−1mm/s、相对下降1mm是事前固定的描述性标记，不是新验收标准。相对下降使用cube与gripper_base原点的**世界Z间距变化**；夹爪局部z朝下，物体下落反而对应local z增加。

| B事件 | t / elapsed s / 行 | 原始观测 |
|---|---|---|
| 首次下落标记 | 29.129 / 0.001 / 0 | vz=-0.00190343m/s；双指世界Fz合计0.810429N，桌面0，重量0.981N |
| 相对世界Z下降≥1mm | 29.196 / 0.068 / 67 | 相对下降1.020890mm，vz=-0.032478m/s |
| 首次机器人/cube穿透>1mm | 29.879 / 0.751 / 750 | 穿透1.104415mm；下落已先发生 |
| 最大机器人/cube穿透 | 30.106 / 0.978 / 977 | 2.830960mm |
| 首次失去双侧≥0.05N | 30.445 / 1.317 / 1316 | Fn=0.044540/0.201110N；相对下降39.3520mm |
| 首次重新触及低桌面 | 30.457 / 1.329 / 1328 | 首个几何接触及正向桌支撑；cube.z=0.107541m、vz=-0.219822m/s；桌Fz=2.290419N，双指0 |
| 末帧 | 43.228 / 14.100 / 14099 | cube.z=0.099784489m，双指0，桌Fz≈0.981N |

| 区间 | 双指世界Fz合计均值N | 桌面世界Fz均值N | 判读 |
|---|---:|---:|---|
| A全部14.1s | -0.00000792 | 0.981006 | 双指接触100%，物体由桌承重 |
| B首次重触前1328样本（29.129–30.456s） | 0.964495 | 0 | 全部vz<0；双指确实提供上托，但未停止下滑 |
| B首次重触起12772样本（30.457–43.228s） | 0 | 0.982721（含碰撞瞬态） | 已失指，随后由低桌面承重 |

B触桌前双指Fz合计范围[-0.138666,1.522958]N，Fn均值1.04114/1.03808N，切向力模长均值0.64912/0.64834N。不能以某帧Fz大于重量、均值接近重量或末帧稳定，宣称持续承载成功。每个事件的完整接触点、法向与力均在JSONL及诊断JSON中。

A/B无执行器饱和、无关节越界、无外部施力。B最大耦合残差0.0954205mm，未超原0.2mm门槛。固定的是目标命令，实际机器人仍自由积分：B重触前gripper_base最大位置漂移1.075199mm、最大下沉0.997730mm、最大旋转0.238743°，7轴最大目标误差0.00300751rad；不能将其描述为机械臂被钉死。B的数十毫米相对下滑不是整体手部位移的假象。

A与旧 `diag_hold_only_002` 后14,100行逐步比对，qpos/qvel/ctrl最大差分别5.65e-15/5.55e-15/3.73e-14。独立重构接触世界总力与cube平移qfrc_constraint最大差为A=9.16e-16N、B=8.33e-16N；含切向及body顺序，采样仍为post-forward瞬时力。

### 6.4 假设与下一项唯一建议

**PASS（已证实）**：在该已批准初态与瞬时撤支撑条件下，恒定姿态命令下也出现持续相对下滑、穿透超限、失指及落桌。失败不需要先执行原提升轨迹；原位有接触不等于独立承载。**FAIL**：B未在14.1s内持续独立支撑0.1kg物体。该诊断结果不计正式抓取成功或失败次数。

**UNKNOWN**：摩擦承载边界、局部凸包法向/软接触及载荷转移瞬态的各自贡献仍未隔离；不能据此证明所有载荷转移方式都失败、原提升轨迹正确或某项参数是唯一根因。

**当时的下一项唯一建议，现已完成**：用现有B记录做离线接触承载预算复核；结果见第7节。本轮未追加物理实验。

### 6.5 本轮可复核证据

根目录仍为上述独立worktree，所有大型Run留本地：

- `experiments/dynamics/runs/diag_free_load_ab_001/`：`model.diff`、`model-diff.json`、`common-state.npz`、`preflight.json`、`reviewed-readiness.json`、prepare/run源码副本及execution；A/B各含model/protocol/initial-state/initial-contacts/trajectory/contacts/observations/result/diagnostic-result。
- `debug/issue2-ab/{issue-updated.json,predeclared-plan.json,prepare-execution.json,run-execution.json}`：批准内容、事前假设/固定指标、真实命令与退出码。
- `debug/issue2-ab/analysis/{diagnosis-ab.json,intervals.csv,events.csv}`：只读事件与分段指标。
- `debug/issue2-ab/{independent-audit.json,independent-servo-audit.json,independent_audit.py,full-tests.log,full-tests-execution.json,evidence-manifest.json}`：独立复核、测试和SHA索引。旧审计版本亦保留。

共同初态NPZ SHA256：`aee269e559c74a1cd3005acd627328b2679c6286ee96e696297b1430be72ccf8`。A/B数据另见本轮SHA索引；未覆盖首轮任何Run。

A轨迹SHA256：`2a3070eed2cb9b852d9601a4f8bb4d31aebf020a67966a809bace63e58e04b37`；B轨迹：`7eb0976355caba3bc3d940e7026f36f93e75bb60536d6c74f1f42cf3d792942d`。

## 7. 既有记录的只读接触承载预算

**PASS（范围/来源）**：基于PR HEAD `8d66da28a49cd351f64ccad40b0528ca9b515faf`，校验A/B的trajectory、contacts、model、protocol、observations、initial-state及initial-contacts共14份原SHA。仅读取旧Run，**新增mj_step=0**；未改资产/控制器/正式判据、未更新成功率。复用 `diagnose_ab.load_ab_episode` 做对齐验证；局部接触参数与运动学在scratch data中仅用 `mj_fwdPosition`/`mj_jac` 重建，不重新求解接触力。A的7,968、B的2,645个接触事件均与原geom/body顺序、位置、距离、完整frame精确匹配，最大差0；重建参数与原始记录的力明确分开保存。

### 7.1 四窗口逐指分解

B严格使用rows0–1327，排除重触桌面row1328；A取同样前1,328行作对照。按积分步左端归窗，四窗rows分别0–67、68–750、751–1316、1317–1327。最后请求窗1317–1329ms截断到1328ms，实际只有11步；不是12ms。下表为**每步先按指合计、再对窗口求均值**（单位N）；逐接触明细另存CSV，不把各接触的平均值当单个摩擦锥。

| B窗口ms / 指 | Fn | Σ\|Ft\| | Fz法向 | Fz切向 | 总Fz |
|---|---:|---:|---:|---:|---:|
| 0–68 / 1 | 1.012807 | .688474 | -.000030 | .466019 | .465989 |
| 0–68 / 2 | 1.014669 | .690463 | .000018 | .468878 | .468896 |
| 68–751 / 1 | 1.041698 | .683910 | -.001043 | .489641 | .488597 |
| 68–751 / 2 | 1.040998 | .686806 | -.001147 | .494012 | .492865 |
| 751–1317 / 1 | 1.064106 | .615020 | -.025322 | .525736 | .500415 |
| 751–1317 / 2 | 1.056221 | .608138 | -.039694 | .504377 | .464683 |
| 1317–1328 / 1 | 0 | 0 | 0 | 0 | 0 |
| 1317–1328 / 2 | .067990 | .067990 | -.004780 | .067814 | .063034 |

全段双指法向Fz均值=-.028876782N、切向=+.993371968N，合计=.964495186N；桌面/其他物体Fz均为0。作用于cube的投影使用body顺序符号及 `frame.T`，法向与两切向分别投影，包含倾斜法向的向下作用。重构Fz与原cube平移qfrc_constraint最大差4.44e-16N。

接触分布从cube局部z约−9.4mm向上缘迁移，正Fn接触最高+19.829mm；跨帧不保证同一材料点。前三窗两指切向相对速度的世界Z均值分别约[−14.96,−15.01]、[−26.50,−26.51]、[−31.63,−33.38]mm/s；末窗有力的指2约−57.40mm/s。751–1317ms中作用cube法向Z最小为−.9307/−.8911。末窗指1只有两个零Fn几何接触，摩擦锥比值为未定义，不能算仍有摩擦支持。所有点的世界/双body局部位置、法线、相对速度XYZ及力分量在逐接触CSV中保留。

### 7.2 动量核对 — 定量近似成立，精确闭合 UNKNOWN

核对 `mΔvz` 与 `Σ(Fcontact,z−mg)dt`，m=.1kg、mg=.981N、dt=.001s。无桌面区间1.328s，实测Δvz=−.210014051m/s，mΔvz=−.0210014051N·s；与该动量变化相符的接触平均力应为.965185689N。

| 日志力取样 | 预测Δvz m/s | 预测减实测残差m/s | 冲量残差N·s |
|---|---:|---:|---:|
| 右端：当前post-forward | -.219183931 | -.009169879 | -.000916988 |
| 左端：初态力+前一行post-forward | -.211279242 | -.001265191 | -.000126519 |
| 梯形：上述两端均值 | -.215231586 | -.005217535 | -.000521754 |

四窗总Fz均值为.934886/.981462/.965098/.063034N，对应实测Δvz为−.032480/+.010739/−.087369/−.100904m/s；逐窗三种冲量残差见JSON。中窗均力略高mg但物体已有负速度，只是部分减速，仍在下滑。左端估计明显接近实测，但力日志是**步后重算瞬时值**，未保存积分时接触力；控制更新和求解时序贡献未分别识别，不宣称守恒精确闭合，也不由平均力差直接推断摩擦极限。

### 7.3 实际摩擦约束 — 合力几何占用 PASS，原求解器状态 UNKNOWN

只读编译模型及逐接触匹配显示：`cone=pyramidal`、`condim=3`，各相关geom的priority=0、solmix=1、friction3=[1,.005,.0001]，无explicit pair（npair=0）或override（enableflags=0）。同priority时摩擦逐元素取最大值，滑动/滚动系数扩展为两轴，实际每个接触friction5=[1,1,.005,.0001,.0001]；condim3只启用法向及两个滑动切向，不能把未启用的扭转/滚动系数当承载项。[MuJoCo 3.2.3参数合成规则](https://mujoco.readthedocs.io/en/3.2.3/modeling.html#contact-parameters)

对每个正Fn接触（Fn>1e-12N），本四棱锥合力占用为 `ρ=(|Ft1|/μ1+|Ft2|/μ2)/Fn`；不是圆锥L2比值，也不是按指平均后的比值。[四棱边基及摩擦锥依据](https://github.com/google-deepmind/mujoco/blob/3.2.3/src/engine/engine_core_constraint.c#L955-L969)

B共2,643个正Fn接触中，1,106个满足几何边界 `|ρ−1|≤1e-10`；四窗分别76/136、691/1366、330/1132、9/9，另两个零Fn接触不定义ρ。首次边界为row14 / t29.143s / elapsed15ms / 指1：Fn=1.016706N，Ft=[−.468490,−.548216]N，ρ=1，L2比值仅.709277，切向相对速度11.259mm/s。B逐点ρ中位数.937082、切向速率中位数26.858mm/s；A相同区间2,656个正Fn接触无边界，ρ最大.334719，切向速率中位数.210mm/s，且桌面承重约.980994N。

因此**PASS**：B存在明显接触相对滑动，部分原始记录合力到达已核验四棱锥边界，后期法向倾斜进一步削减向上合力。**UNKNOWN**：原始efc_state/完整棱边力未记录，无法通盘验证原求解器stick/slip标签或全部约束利用状态；软接触允许非零接触速度，不能把速度非零直接等同原求解器滑移状态。首次下落1ms也早于首次边界15ms，尚不能证明摩擦锥是唯一根因。

### 7.4 下一项唯一可证伪假设（未执行）

探索性假设：在相同B初态、命令及所有其余参数下，**仅把cone从pyramidal改为elliptic，68ms时相对世界Z下降将小于1mm**；若仍≥1mm则反驳该预言。依据是15ms时L1已达1而L2仅.709，存在锥形式敏感度的可能。cone是全局选项，会影响所有接触；即使改善，也不能区分几何边界与软约束动力学的贡献，[两种锥的软接触动力学并不相同](https://mujoco.readthedocs.io/en/3.2.3/computation/index.html#friction-cones)。这是对未来单一诊断差异的建议，非本轮授权；需用户确认后才可实施，未生成或执行该变体，未改正式验收。

### 7.5 只读证据与核验

本机worktree下 `debug/issue2-budget/`：`issue.json`、`plan.json`；`contact-metadata.json`及`reconstruct_contact_metadata.py`；`result_002/{budget.json,contacts.csv,windows-per-finger.csv}`（10,613个逐接触行，包含A的桌接触，及32个逐窗分组行）；两份`independent-{impulse,friction}.json`及对应只读helper；`evidence-manifest.json`。初稿空集合错误记录及原脚本另存，未覆盖输入。

复核：`.venv/bin/python debug/issue2-budget/contact_budget.py --output-dir debug/issue2-budget/recheck_001`（使用新目录，拒绝覆盖）。本轮仅正式报告有源码仓库变更，未新增生产分析模块或回归体系。现有纯记录fixture测试 `.venv/bin/python -m unittest discover -s experiments/dynamics -p 'test_diagnose*.py' -v` 为17项PASS；本轮未运行包含mj_step的全量仿真测试。根计算与独立冲量核算逐窗一致至1e-15。
