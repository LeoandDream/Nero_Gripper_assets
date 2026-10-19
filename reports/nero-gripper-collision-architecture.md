# Recommended NERO Gripper Collision Architecture v1

Issue [#25](https://github.com/LeoandDream/Nero_Gripper_assets/issues/25)，2026-10-19，Ubuntu 22.04。**唯一推荐：完整功能接触层＋有语义的侧壁/背体/安装框架。** 不再以中心矩形 pad 加顺序切割 guards 表示工作面，不预设 Panda 的 pad 数量。

**状态：`ARCHITECTURE_BLOCKED_BY_GEOMETRY_UNKNOWN`，`implementation_ready=false`。** 已完成源表面清单与唯一功能架构；尚未确认整个装配实体的外边界及内部接口，不能给出已验证闭体、确定的底层 geom 数量或可执行 MJCF。下面8项是每指的**语义组件族，不是8个已定型 geom**。本轮不实现，也没有导入 MuJoCo、编译模型、MuJoCo碰撞查询、`mj_step`、控制器执行、抓取/benchmark或参数修改。

## 1. 来源与坐标

基线 `feature/panda-gripper-reference@d4a37dafb9a7b9cb3392a8937590dd091a5e4a7a`，独立文档分支 `feature/collision-architecture-v1`，不继承 #23 candidate 开发分支。固定来源与逐文件 SHA 见 [来源清单](nero-gripper-collision-sources.json)：

| 来源 | 冻结版本 | 本轮核查 |
|---|---|---|
| NERO URDF/Xacro、STL、DAE | `agx_arm_urdf@a68b8e876a2c6fcb0ecee7a7943129e4d48dd864` | 装配/关节引用、原始表面、转换后的 visual 与000/001 |
| Official NERO MJCF/STL | `agx_arm_sim@f8cd8b147c75d59e14f90fb0646770eefa268ed0` | 每指整份mesh collision；两份指 STL 与 URDF 源逐字节相同 |
| Menagerie Panda | `62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35` | `panda.xml`、`hand.xml`、README、`finger_0/1.obj` |
| robosuite Panda | `5ce6643f3092639d08f7b0f90ed1c6a84f50552c` | gripper XML、`finger.stl`、visual、动作边界的既有审查 |
| MuJoCo3.2.3文档源 | `476599f8b6175ef5a3040c47ebb3cd99a83c5365` | 普通mesh的凸包语义、碰撞mask语义 |

每指 **2086个源三角片，分为37个不重复区域，遗漏0**。这是源三角片库存的完整性，**不等于装配外表面已完整认证**。STL以文件三角形顺序、OBJ以`f`行顺序从0编号。DAE到STL顶点差≤约97.2nm、源到visual≤约95.8nm；不能按重排后的三角片或编译mesh主轴坐标解释索引。左右指对应三角片坐标最大差约`1.77e−18m`。

mesh与finger body局部帧一致：`y=0`是指尖、负y朝根部、`−z`朝夹持内侧；侧壁以局部`±x`命名，避免混同世界左右。指体在gripper_base中近似`z_base=138mm+y_local`。整指范围约`x±28mm, y[−76.5,0]mm, z[0,26.5]mm`。

![源几何功能区域，未生成碰撞候选](nero-gripper-functional-surfaces.png)

## 2. 一根手指真实有哪些区域

下表是源几何证据及**本架构的用途建议**；“主接触”不是厂商已标注材料/用途。完整37区的三角片、范围、法向分布、平面边界点和邻接见 [结构化清单](nero-gripper-functional-regions.json)。曲面不能用平均法向替代。

| 区域 / 源STL三角片 | 局部位置与法向 | 物理职责建议与相邻关系 |
|---|---|---|
| 大内平面：557–577 | y约−71..−34.71785mm，z=2mm，n=(0,0,−1) | 主内接触候选；接过渡面、薄层侧缘和根部缺口；不是仅因“靠根”就只防碰 |
| 指尖短内平面：545–548 | y=−10.02218..0mm，z≈0，n=(0,0,−1) | 精细指尖夹持候选；完整渐宽六边形，接ramp、两侧缘、tip端面 |
| 内斜面：549–556 | y=−30.33513..−10.02218mm，z=1.80560..0mm，n≈(0,−.088540,−.996073) | 斜抓/导入/过渡接触；接短平面与肩部，不能用原凸包浅斜面替代 |
| 两段肩部过渡：578–579 / 580–581 | y=−34.71785..−30.33513mm，z=2..1.80560mm；n分别约(0,−.049474,−.998775)、(0,−.005166,−.999987) | 连续内轮廓的真实折面；辅助接触，非两种新材料pad |
| tip前缘：592–593、1962–1965 | y≈0，x±10mm，z约0..7mm，n=(0,1,0) | 探碰、插入、贴桌/推拨边缘；与短内平面及背体端部相邻，不是零厚度线或天然capsule |
| 薄层及背体两侧壁 | `thin_layer_*_sidewall` / `backing_outer_*_sidewall`；分布见JSON，主要朝±x，随轮廓倾斜 | 侧抓/推拨和防碰；接工作面真实taper边、tip、外背面，不能留下“朝内侧缘guard”承担同一工作面 |
| 外背面、凹袋及孔周 | `back_outer_roof_and_transitions`、`back_pocket_*`、`backing_*_bore` | 外侧接触与保护；背袋底约z8.22..9.63mm，外壳最高z26.5mm；不得用一个大凸包封闭源袋口 |
| 根部安装块：0–503 | x±8.5mm、y约−76.5..−70mm、z4..26.5mm；多方向法向 | 安装/环境防碰，非本架构预定夹持面；源含缺口、四个凹座和中心孔，装配态占据仍UNKNOWN |
| 薄层背面504–541与承托接口660–697 | z约2..4mm，近重合反向表面 | 内部接口候选；不另发射一层暴露“抓取pad”，需要装配并集确认 |

三个关键修正：

- 大内平面面积约 **1492.611mm²**，明显大于短平面229.624mm²和ramp585.146mm²。内轮廓5组共37个三角形的质心均为从夹持侧进行源三角形射线检查的首交；不能用过去cube触点把大平面排除。该采样不证明所有点可达或厂商实际用途。
- 大平面是**非凸23顶点轮廓**，有中央根槽、较大的非对称左缺口及非凸侧轮廓。其二维凸包为1613.033mm²，多填 **120.421mm²（8.068%）**。单box/单凸面不能精确代表它；这个面积差不是运行时失效或抓取性能证据。ramp凸包也多填约0.049333mm²，不能悄悄忽略为零。
- 已知约2mm的是前薄层厚度依据，**不是整个指尖厚度**；tip端面到z≈7mm。四个背体y向孔、侧孔、根部凹座/中心孔以及背袋均已记录。源孔在实物装配后是否被紧固件或滑块占据尚不确定，既不能盲目封孔，也不能断言实物全部开放。

## 3. 唯一推荐的组件组织

所有名称加`finger1_`/`finger2_`前缀，保留在原finger body，仍为同一刚体。一个功能组件可含几个有源依据的凸实体；底层数量、子实体名称和封闭边界目前为UNKNOWN，禁止用匿名序号填充。

| 组件族 | 推荐primitive类型 | 源对应、职责和边界 |
|---|---|---|
| `inner_main_contact_plate` | 按源非凸轮廓分区的有限厚凸多面体组 | 拥有完整大内面及其真实边缘；不可只取中心矩形，不能填根槽/左缺口；是一个功能工作区，未必一个geom |
| `distal_tip_contact` | 源轮廓闭合polyhedral cap，凸细分待定 | 拥有短内面、真实tip前缘及相应侧边；不额外叠一个capsule；背面和与chassis的实体接缝未定 |
| `inner_ramp_contact` | 源斜面楔形凸多面体组 | 拥有完整渐宽斜面；不把两侧真实夹持条带留给“防碰”代理；保留真实微小非凸边界 |
| `inner_transition_contact` | 按真实肩部两facet组织的凸楔组 | 分别命名`shoulder_bevel`/`shoulder_blend_facet`等物理区；连接大内面与ramp，不复制Panda小box |
| `sidewall_negative_x` | 保持源外轮廓的凸侧壁/框架片 | 负责剩余背体−x侧壁；前述接触实体已拥有的侧边不再次覆盖；辅助操作与保护 |
| `sidewall_positive_x` | 同类凸侧壁/框架片 | 对应+x侧壁；必须尊重左右不对称缺口，不能盲目镜像一张AABB |
| `outer_chassis` | 保留凹袋/孔口的语义凸框架 | 负责外背、袋边、孔周与背体根帽；“粗粒度”仅指次要细节简化，不准整体凸包把可达空区填满 |
| `mount_root` | 保留缺口的安装框架凸分组 | 负责源安装块；未来子体应对应真实安装侧耳、桥部等；孔槽装配占据未明前不冻结实体 |

这套架构选择的是**一个完整主内接触区＋几何所需的tip/ramp/shoulder辅助区＋侧壁/背体/安装框架**。不是“一个矩形主pad”，也不是“每个区域恰好一个primitive”。不根据历史某帧结果选择切口或移动几何。

`collision_000`当前把主体与短平面合成凸包，跨越多个功能区；`collision_001`是安装块的包围凸体，也填其缺口。**v1建议两者都只作历史参考，未来活动表示由上述语义组件整体替代/重新组织**，不在原000/001上继续叠pad。原文件、SHA和已验收机械字段保留；本轮实际未替换任何文件。

职责约束：完整工作面及taper边缘必须有唯一解释的外边界归属；内部封口面保持在实体并集内部。真实相邻面可同时接触同一物体，**多contact本身不是重复约束证据**；禁止未解释的同工作面多重代理。薄层侧缘跨多个接触区，JSON明确其精确组件接缝未解，未把37区库存误称为最终geom映射。

**不建议新增explicit contact exclude。** 保留既有冻结配置；同finger内无需为几何组件添加body对排除。“防碰”是用途说明，不是关闭对cube或其它任务物体的碰撞/摩擦；侧壁、背体、安装区对所有可达物体、桌和环境仍应正常传力。不能靠过滤掩盖不良几何。

## 4. 与已有实现的关系

| 实现 | 可迁移的原则 | 不能照搬或过度推断 |
|---|---|---|
| [Official NERO][O] | 源整指轮廓自然包含内面、侧壁、背体和安装形状；同一finger/关节结构 | [普通mesh碰撞按凸包][D]，凹斜面、根槽、背袋和孔口可能被填；URDF/STL来源正确不等于碰撞表面正确 |
| [Menagerie Panda][P] | 手工显式tip局部表示，同时保留finger body保护 | 五box不是五个真实独立部件；大box近似中央后退区、四小box近似四角近共面的区；大box与四小box、且与finger_0碰撞凸包存在重叠，不能据名字保证互斥或材料真实性 |
| [robosuite Panda][R] | 简单pad＋finger mesh的命名、关注区域组织 | 单box也与mesh凸包重叠；其mask仍允许body接物。不能说mesh只防碰、pad独占抓取；不同摩擦和servo不能迁入本轮NERO |

本轮对固定XML/源mesh进行纯几何复核：Menagerie大box与两上小box分别重叠90mm³、与两下小box分别105mm³，也与`finger_0`凸包有正体积交叠；robosuite单box与finger凸包也有正体积交叠。**这只说明近似组成，不是新接触查询或性能比较。** 保留finger mesh的意义是覆盖局部pad以外的实体；不能神化为其全部几何职责已经隔离。源码及几何依据的路径/SHA见来源清单，本地详细审查保存在`debug/issue25/reference-audit/`。

上一版guards不适合作为长期架构的原因是**由矩形棱柱切割顺序决定子块，而非完整真实功能面与实体结构**：朝内taper边仍交给了“保护体”，难以解释未来斜抓、侧推和孔槽附近接触。不是因为“12个太多”；将`guard_2`改名也不能修复职责。#23已存第一帧共存只是隔离未明的证据，没有证明重复力，本轮未继续修该候选。

## 5. 不改变控制语义

维持2个finger slide joint、原equality、两个finger actuator，以及kp/kv/force limit、friction、solver/nativeccd和action adapter。上层仍为 **`[J1..J7,w]`的8D绝对目标**，arm用rad，`w∈[0,.1]m`，`q1*=w/2, q2*=−w/2`，w增加表示开口增加；内部仍是7 arm＋2 finger的9路限力ctrl，不迁移Official单伺服或Panda tendon。

`w`是源虚拟命令坐标，**真实净指间距尚未标定**。在理想对称、同截面实际关节姿态下，源对向内面可写`gap(y)≈q1−q2+2*z_inner(y)`：短面z≈0，大内面z≈2mm，二者几何开口差约4mm。因此不能把一个标量对所有工作区都叫“真实净间距”，更不能改adapter弥补新几何。碰撞角色元数据不改变动作、观测或成功判据。

## 6. 已确定、阻断及下一步

**已确定**：源版本/对应关系、五段连续内轮廓、完整37区库存、平面边界与法向、主面非凸、tip完整高度、源孔槽/背袋、000/001的来源覆盖，以及唯一组件组织。未从颜色推断橡胶、摩擦或软硬。

**关键UNKNOWN**：薄层与承托面的装配并集外边界、侧/背封口与实体接口；孔槽装配态占据。源在10nm顶点等价分析后仍有19条非流形缝边，z2..4mm存在近重合反向接口。这里没有完成整个实体外壳的认证；不能把每个source triangle当外露接触面，也不能把任意法向挤出当真实闭体。阻断锚定这些实体/外露边界，不是因为凸分组数还没选，也不证明源文件无法被进一步澄清。

因此唯一v1停留于**推荐功能架构**，可执行geom清单和尺寸冻结仍BLOCKED；真实用途/材料、硬件净间隙及动力学性能也未标定。未设新Gate、未运行测试基础设施或实验，未修改正式模型、旧Run、#23候选或其它工作树。

**下一步唯一验证：离线核验冻结源的“薄层＋背体＋安装块”装配外边界与孔槽占据，形成可追溯的外露面/内部接口清单。** 先使用现有DAE/分件与剖面证据，若仍不唯一再求厂商CAD/装配剖面；本轮不生成新碰撞模型或用query猜边界。该问题闭合后，才有依据另行授权凸实体设计与验证。

核查范围：18项本地冻结输入、17项实际读取参考来源；两指37区各2086片覆盖无遗漏/重复；无新MuJoCo调用或动力学。分析脚本与完整3MB原始清单仅保存在Ubuntu `debug/issue25/`，不新增工程框架。交付仅本报告、结构化JSON、来源清单及一张源几何图，供独立审查。

[O]: https://github.com/agilexrobotics/agx_arm_sim/blob/f8cd8b147c75d59e14f90fb0646770eefa268ed0/mujoco/agilex_arm/agilex_nero/nero.xml
[P]: https://github.com/google-deepmind/mujoco_menagerie/blob/62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35/franka_emika_panda/panda.xml
[R]: https://github.com/ARISE-Initiative/robosuite/blob/5ce6643f3092639d08f7b0f90ed1c6a84f50552c/robosuite/models/assets/grippers/panda_gripper.xml

[D]: https://github.com/google-deepmind/mujoco/blob/476599f8b6175ef5a3040c47ebb3cd99a83c5365/doc/XMLreference.rst#L1110-L1116
