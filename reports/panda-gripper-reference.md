# Issue #21：从 Panda 参考设计审查两套 NERO Gripper

2026-10-19，Ubuntu 只读源码审查。**唯一候选：为 Self-built 的真实指尖工作面建立专用 primitive contact pads 的接触表示。** 本轮只提出候选和证伪方法，不实现；不建议同时更换 actuator/tendon。两个 actuator + equality 是需要检查的冗余驱动结构，**尚未证明它造成当前抓取失败**。

新增模型加载、MuJoCo 导入/API、碰撞查询、`mj_step*`、`mj_forward*`、控制器执行、benchmark 和物理 Episode 均为 **0**；模型、参数、成功判据和历史 Run 未改。仅独立 Draft PR，base=`feature/fixed-state-collision-ab@6727a86fb8ea0d6c325a8f7f13fc849a9afa90a7`，不合并。

## 参考版本与来源边界

- **Menagerie Panda**：`62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35` 的 `panda.xml`、`hand.xml`、README，及解释 README 所需的 `mjx_panda.xml`。全部实际在线读取，不以 README 代替 XML。[Panda][P] / [hand][H] / [README][R]。
- **robosuite**：`5ce6643f3092639d08f7b0f90ed1c6a84f50552c` 的 PandaGripper、GripperModel、ManipulatorModel 及实际控制缩放源码；它是另一种实现，不冒称 Menagerie 的同一模型。[gripper Python][RG] / [gripper XML][RX]。
- **Official NERO**：`agilexrobotics/agx_arm_sim@f8cd8b147c75d59e14f90fb0646770eefa268ed0` 的原生 MJCF；另行标注已有临时 `link5↔link7` exclude + kp80 诊断，不混入原版。[原 XML][O]。
- **Self-built NERO**：上述本分支基线；机械来源为 `agx_arm_urdf@a68b8e876a2c6fcb0ecee7a7943129e4d48dd864`，静态 XML 与受控派生分开。现有失败物理 Run 为 Issue #14 的 elliptic/impratio10、nativeccd=false 配置；#18 的 native 结果仅是保存姿态查询，不是已采用的新物理模型。[静态模型][S] / [派生及控制器][C] / [冻结结果][G]。

外部 commit、逐文件 URL/访问 UTC/SHA、本地来源与历史证据路径见 [来源清单](panda-gripper-reference-sources.json)。机制解释依据冻结 MuJoCo 3.2.3 文档，不把源码阅读冒称为运行或真机标定。

## Panda 的 equality、tendon、actuator 分别做什么

两个 slide joint 都取 0..0.04 m；右指 body 旋转使同号局部坐标产生对向运动。joint equality 的默认线性关系令两指位置同步；它是软约束，不保证带载残差恒为零。fixed tendon `split` 则定义 `l=(q1+q2)/2`，其本身没有再加入一个硬位置约束。唯一 gripper actuator 作用于该 tendon。[默认参数][P1] / [实体关节与几何实例][P4] / [tendon、equality、actuator][P2]。

按 transmission 的梯度，标量 actuator force `p` 产生两指广义驱动力 `(p/2,p/2)`；**等分的是执行器输入的传力份额，不是保证两侧接触法向力相等**。接触、惯性、阻尼和 equality 反力仍会影响最终负载。equality 单独不能表达这一对称传力；只有 tendon、没有 equality，也不保证两指等位移。[MuJoCo transmission][D1] / [fixed tendon][D4] / [equality][D2]。

这里 actuator8 是 affine `general`，等效位置伺服：`p=0.01568627451*u -100*l -10*ldot`，`u∈[0,255]`，对应无载目标 `l≈0.04*u/255`；scalar force 限制 ±100 N。**不是每指接触力 ±100 N，也不是 0..255 N 的力指令。** 单一输入驱动共同开合坐标，避免为两指独立指定不相容目标；这是该模型选择，不是所有成熟夹爪必须使用 tendon。[actuator][P2] / [维护者说明][R1]。

每指有 **5 个手工 box pad**，同时保留 `finger_0` collision mesh；不是删除全部手指碰撞、只留理想夹持平面。工程意义是显式定义局部工作面，并将外观、指体防碰和指尖接触表示分开；可能改善接触面的可控性，但本次没有运行证据证明更稳，box 边角仍可发生特征切换。[pad 默认][P1] / [实例][P4]。

contact exclude 只屏蔽指定 body 对，不负责同步、分力或抓住物体。**独立 `hand.xml` 明写 hand↔左右指两项 exclude；整臂 `panda.xml` 只有 link0↔link1 显式 exclude**，不能说两文件完全相同；父子体默认过滤另起作用。README 的历史 link7↔link8 exclude 描述也不等于当前 XML。[hand excludes][H1] / [panda excludes][P3] / [过滤语义][D3]。

## 必须逐项比较的设计

判断列是两套 NERO 相对标准 Menagerie Panda 的综合设计关系，**不是性能或物理可信度评分**。只取 Issue 规定的四种标签。

| 设计维度 | Panda Menagerie | Official NERO 原版 | Self-built NERO | 判断 |
|---|---|---|---|---|
| gripper action DOF | 1，u=0..255 | 1，finger1 目标0..0.05 m | 1，源命令w=0..0.1 m | MATCH |
| physical finger joints | 2 slide，均0..0.04 m | 2 slide，q1正/q2负 | 2 slide，q1正/q2负；机械臂另7轴 | MATCH |
| coupling | joint equality，q2=q1 | equality，q2=−q1 | 同一源 mimic 符号关系 | PARTIAL |
| force sharing | fixed tendon，两项各0.5 | 单驱动指1，equality传递负载；无tendon | 两指独立有限力输出+equality反力，无固定分力比例 | STRUCTURAL DIFFERENCE |
| actuator topology | 7 arm + 1 tendon gripper | 7 arm + 1 joint position gripper | 静态nu=0；派生7 arm + 2 finger general force servos，nu=9 | STRUCTURAL DIFFERENCE |
| fingertip collision | 每指5个box pad，独立于视觉 | 无专用primitive pad，整指STL mesh | 无专用pad；主要接触在指1/2的`collision_000`凸代理 | STRUCTURAL DIFFERENCE |
| full-finger collision | pad之外仍有finger mesh碰撞 | 每指1个整指mesh geom | 每指2个源连通分量的凸代理mesh；未因接触失败删指体碰撞 | PARTIAL |
| contact exclusions | hand与panda文件显式列表有别，见上文 | 原base_link↔link1；临时诊断另排link5↔link7 | 静态无exclude；批准的派生只排base_link↔link1 | PARTIAL |
| solver/equality softness | equality显式solref=.005 1、solimp=.95 .99 .001 | equality/contact softness未显式写；原elliptic/10 | 同样继承引擎softness；当前物理Run elliptic/10 | PARTIAL |
| grasp contact semantics | XML只定义模型，不自带抓取PASS判据 | kp80临时诊断按20mm/1s及承载等条件得到最小PASS | 原80mm/2s、≤10mm漂移、≤1mm穿透等完整判据仍FAIL | PARTIAL |
| sim-specific manual edits | 手工pads、excludes、伺服调节；MJX另简化 | 有原生servo/过滤；临时改动不能称正式修复 | 资产转换、受控派生、批准过滤与逐项诊断均需保留来源 | PARTIAL |

NERO事实逐项来自[原 XML][O]、[Self-built XML][S]、[控制器与配置][C]及 #14/#16/#18 已保存报告。相同 ±10 N actuator 限制不代表相同总夹持能力，更不代表每指法向力10 N。两路线没有共同的任务/控制/判据，**这张表不是 Official vs Self-built benchmark**。

## robosuite 的交叉核查与“过驱动”判断

robosuite 将 GripperModel 与机械臂模型分开，当前 `add_gripper` 位于继承 RobotModel 的 **ManipulatorModel**，在末端装配并合并XML；不需要因此再拆一个仓库。`important_geoms` 单独标识左右指和pad，sites提供抓取/力矩参考帧，site本身不附着物体，也不自动定义成功。[装配边界][RA] / [标识与动作][RG]。

其 XML 是 **两个 position actuator、每指一个box pad+指体mesh，无 tendon、无 equality、无显式 contact block**。PandaGripperBase继承的动作dof=actuator数2；PandaGripper覆盖为1，`sign(action)`经speed=.2累积成反向两路normalized目标，再缩放到各自ctrlrange；`−1`开、`+1`关。它证明“上层1D”不取决于“内部一个actuator”，但**不是双actuator+equality组合的直接实证**。[XML][RX] / [动作][RG] / [基类][RB] / [控制串接][RT] / [实际缩放][RC]。

| 待判断命题 | 结论 | 证据及限制 |
|---|---|---|
| Self-built在一个耦合开合方向上有两个驱动通道 | SUPPORT | 结构冗余存在；finite-force actuator不是额外的硬位置约束。 |
| 仅凭两个actuator+equality即可断言必然失同步或饱和 | CONTRADICT | 现有目标严格满足`q1*=w/2, q2*=−w/2`；#14 R0/R1控制健康PASS，最大coupling残差.057017/.059128mm，低于原.2mm，饱和比例0。 |
| 冗余内力已造成本次滑移/穿透FAIL，或反过来已完全无害 | UNKNOWN | 缺独立equality载荷/互抗功分解及只改拓扑的反事实；小残差并不证明小内力。 |

用两指统一的“开合正方向”看，两路驱动力可分解为共同开合分量和差动分量；非对称接触、误差、偏置或限幅可能产生由equality承担的差动内力。它是应验证的机制，不能从通道数量直接推成故障。当前没有依据把更换tendon作为优先改造。`docs/MODEL.md`旧“带载未过.2mm”表述早于#14，不能代替上述最新保存数值；本轮不修改旧文档。[#14结果][G]。

Official已做对的是 **单一原生位置伺服＋源mimic＋单标量输入**，与Menagerie/MJX的一驱动开合思路接近，结构更直接；**不等于已具备Panda tendon分力或primitive pads**。临时exclude+kp80获得的最小PASS仍有1.370134mm最大穿透、1.089645mm联动残差及后段下滑，且原版link5↔link7凸包伪接触尚不是正式修复，不能用其PASS证明全面优于Self-built。[官方诊断结果评论][OK] / [本地证据清单](panda-gripper-reference-sources.json)。

## 不神化 Panda，也不改变上层8D契约

README记录手工collision简化、excludes和actuator调节；MJX进一步减少/屏蔽碰撞、保留equality但**移除tendon，改为单指joint伺服**。源码显示降低kp/kv可对应arm；夹爪由tendon长度反馈100变为direct-joint350、速度反馈仍10，传动已变，不能泛称“夹爪kp/kv也降低”。`hand.xml`与完整模型的差别同样说明它们是为用途选择的工程近似，不是唯一物理真值。[MJX说明][R2] / [驱动源码][M] / [碰撞defaults][M1] / [碰撞实例][M2]。

上层维持 `[J1..J7, g]` **7+1维**，内部实体关节可以是9个。Self-built现有`g=w`是源虚拟命令坐标，**不是已标定的指尖净间隙**；Official的`g`是单指目标，robosuite则是开闭增量方向，Panda是0..255重映射。不能直接互换标量或把两种8D动作数据混为同一种语义。未来VLA只需在已有边界记录单位、正负方向、绝对/增量、范围和时间语义；保留Self-built现有接口，不复制Panda的控制刻度/行程/增益。[NERO接口契约][SC]。

还须区分：Self-built 8D用户位置目标最终变为 **9D原生力ctrl**；Official 8D原生position ctrl与用户规划角也可能因冻结重力偏置不同。兼容8D不意味着可把9D状态直接当8D动作、把实测qpos当目标，或忽略adapter。[控制器][C] / [已有官方数据契约](https://github.com/LeoandDream/Nero_Gripper_assets/issues/19)。

## 唯一候选与下一组可证伪实验（仅方案，未授权执行）

**候选：将Self-built手指的接触表示明确分为“按NERO真实工作面标定的primitive指尖pads＋保留必要指体防碰代理”。** 不是照抄Panda尺寸/摩擦，也不是在同一区域无条件叠加原凸包与box。未来diff仅允许手指接触几何表示；视觉、关节、惯性、安装、equality、actuator/控制参数、全部接触材料和solver参数、过滤、8D契约保持不变。若原整块代理占据同一工作面，须避免重复或错误外伸接触，并核验非工作面防碰覆盖，不能为减少失败记录删掉指体碰撞。

理由来自成熟模式及本项目证据，而非已证根因：#16/#18实际接触在`collision_000`，同geom法线/代表点会跳变；固定姿态只换native即可消除首次尖峰，反驳了“必然是CAD坏了”。另一方面，R1 rows25984–25991的连续真实凸代理重叠仍>1mm，原高度/漂移失败仍在；显式工作面值得验证，但**没有证据保证pads会消除滑移或抓取失败**。primitive变化还会改变检测路径及接触数量，即使改善，也不能单独归因于“平面形状更正确”。[几何审查][E16] / [固定查询][E18]。

下一步只建议另开授权Issue验证此候选，不并行尝试tendon改造：

1. 先按原始NERO几何确定实际工作面和净间隙，封存唯一候选及尺寸误差界；复用#18冻结422姿态核对工作面覆盖、穿透和非工作面防碰，保持原1mm判据。法线连续性只作诊断，不能要求消除真实边缘切换。若工作面依据不足、出现假接触/重复接触或漏防碰，候选不成立并停止，不移pad寻找成功。
2. 几何审查成立后，仅在**另行明确授权的**有界完整R1 A/B中验证：A原contact表示、B唯一候选；同一冻结合法初态、动作/控制代码、原elliptic/impratio10/nativeccd=false及全部判据，先封存所有运行依赖。建议最多A/B各1个Episode、无扫参/重试。检查全部原抓取项目与全部robot/cube穿透，不只统计pad接触；若仍FAIL就如实否证“这项改造足以修复”，不能只凭法线变平滑宣布成功。

上述是一个结构候选的分阶段验收，不是本轮实施许可。尚不知道真实指尖材料/净间隙校准、源凸代理内部检测feature、冗余内力份额，以及pads对真实闭环抓取的影响。历史两个helper执行期身份仍UNKNOWN，沿用#16/#18边界，不补证已闭合。本报告核验为只读设计分析；**Self-built完整抓取仍FAIL，正式模型改动授权NONE**。

[P]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/panda.xml
[H]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/hand.xml
[R]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/README.md
[P1]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/panda.xml#L7-L35
[P2]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/panda.xml#L253-L277
[P3]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/panda.xml#L284-L286
[P4]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/panda.xml#L218-L240
[H1]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/hand.xml#L93-L107
[R1]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/README.md#L41-L45
[R2]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/README.md#L48-L58
[M]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/mjx_panda.xml#L254-L270
[M1]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/mjx_panda.xml#L17-L34
[M2]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/mjx_panda.xml#L217-L241
[RG]: https://github.com/ARISE-Initiative/robosuite/blob/5ce6643f3092639d08f7b0f90ed1c6a84f50552c/robosuite/models/grippers/panda_gripper.py
[RX]: https://github.com/ARISE-Initiative/robosuite/blob/5ce6643f3092639d08f7b0f90ed1c6a84f50552c/robosuite/models/assets/grippers/panda_gripper.xml
[RA]: https://github.com/ARISE-Initiative/robosuite/blob/5ce6643f3092639d08f7b0f90ed1c6a84f50552c/robosuite/models/robots/manipulators/manipulator_model.py
[RB]: https://github.com/ARISE-Initiative/robosuite/blob/5ce6643f3092639d08f7b0f90ed1c6a84f50552c/robosuite/models/grippers/gripper_model.py
[RC]: https://github.com/ARISE-Initiative/robosuite/blob/5ce6643f3092639d08f7b0f90ed1c6a84f50552c/robosuite/controllers/parts/gripper/simple_grip.py#L175-L186
[RT]: https://github.com/ARISE-Initiative/robosuite/blob/5ce6643f3092639d08f7b0f90ed1c6a84f50552c/robosuite/controllers/composite/composite_controller.py#L86-L103
[O]: https://github.com/agilexrobotics/agx_arm_sim/blob/f8cd8b147c75d59e14f90fb0646770eefa268ed0/mujoco/agilex_arm/agilex_nero/nero.xml
[S]: ../mjcf/nero_gripper.xml
[C]: ../experiments/dynamics/controller.py
[SC]: ../docs/robot-contract.json
[G]: issue14-results.json
[E16]: issue16-contact-geometry.md
[E18]: issue18-fixed-collision-ab.md
[OK]: https://github.com/LeoandDream/Nero_Gripper_assets/issues/7#issuecomment-6257759446
[D1]: https://github.com/google-deepmind/mujoco/blob/476599f8b6175ef5a3040c47ebb3cd99a83c5365/doc/computation/index.rst#L262-L291
[D2]: https://github.com/google-deepmind/mujoco/blob/476599f8b6175ef5a3040c47ebb3cd99a83c5365/doc/XMLreference.rst#L4574-L4618
[D3]: https://mujoco.readthedocs.io/en/3.2.3/computation/index.html#selection
[D4]: https://github.com/google-deepmind/mujoco/blob/476599f8b6175ef5a3040c47ebb3cd99a83c5365/doc/XMLreference.rst#L4892-L4959
