# Issue25：唯一 finite-patch candidate

**最终状态：`V23_FINITE_PATCH_REPARTITION_BLOCKED`。** 原11条normal、上一轮新增15条normal、12条surface及明确指定的source2030点全部通过，但本次有限BSP分割生成过多薄体，真实违反0.1mm最小尺度；保存网格的完整outside-footprint材料保持证书仍UNKNOWN。本候选拒绝，不进入MJCF或物理验证，不生成第二候选。

任务：[评论6274041658](https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6274041658)。构造规则先独立提交于 **`1fc695f`**。几何起点严格是原`6eba4f3`的346体，geometry SHA `79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e`；没有以失败v2.2几何继续修改。v2.2报告只提供no-regression坐标。

## 1. 实际构造与范围

- 每个A source patch保留support plane和有限polygon footprint。104个注册influence prism覆盖32个授权基线实体；法向范围从该基线实体有界support范围冻结，切向范围只来自有限source polygon。
- footprint使用固定Minkowski方形边界余量，Euclidean半径`NUM/64`；分析grid为`NUM/4096`。没有移动source plane，也没有放宽v2.1 A/B/C、protected void或尺度规则。
- 每次只对当前凸块与**有限prism外侧cap的交集**分割：保留所有prism外分支及source plane内侧部分，仅删除最终落在有限cap内的叶。没有whole-entity infinite-plane replacement。
- 同一基线实体内只按冻结的共享支持面/凸并集条件合并分割碎片；不跨基线实体全局压缩，不按尺度失败删除材料。
- 新候选 **1387体/指 = 314个未改实体 + 1073个局部后继**，比346体增加1041体。32个授权实体均被split；最多V20273分成159体，V20280为138体，V20157为129体。这**未达到“少数几个凸体”的runtime目标**。
- main7、tip4、ramp7、shoulder3、mount_root118及其它未授权实体均保持原样；314个完整entity对象的canonical JSON bytes及vertices/faces二进制数组逐字节相同。

新geometry SHA：

`4050c488325b409532360ee9d8dc3311bd3a53bc36b75949af637bfc8b59fd96`

[唯一候选](nero-gripper-v23-finite-partition.json) · [完整旧→新映射](nero-gripper-v23-mapping.json) · [所有finite footprints/prisms与预注册规则](nero-gripper-v23-preregistration.json)。以下名称区间均连续，JSON另列出每一个后继ID。

| 原entity | 后继数 | 连续后继ID |
|---|---:|---|
| V20140 | 20 | V20140_FP001 … V20140_FP020 |
| V20143 | 19 | V20143_FP001 … V20143_FP019 |
| V20154 | 62 | V20154_FP001 … V20154_FP062 |
| V20157 | 129 | V20157_FP001 … V20157_FP129 |
| V20160 | 39 | V20160_FP001 … V20160_FP039 |
| V20166 | 9 | V20166_FP001 … V20166_FP009 |
| V20175 | 5 | V20175_FP001 … V20175_FP005 |
| V20205 | 4 | V20205_FP001 … V20205_FP004 |
| V20207 | 5 | V20207_FP001 … V20207_FP005 |
| V20245 | 5 | V20245_FP001 … V20245_FP005 |
| V20259 | 25 | V20259_FP001 … V20259_FP025 |
| V20263 | 59 | V20263_FP001 … V20263_FP059 |
| V20267 | 41 | V20267_FP001 … V20267_FP041 |
| V20273 | 159 | V20273_FP001 … V20273_FP159 |
| V20274 | 15 | V20274_FP001 … V20274_FP015 |
| V20280 | 138 | V20280_FP001 … V20280_FP138 |
| V20284 | 23 | V20284_FP001 … V20284_FP023 |
| V20291 | 31 | V20291_FP001 … V20291_FP031 |
| V20294 | 5 | V20294_FP001 … V20294_FP005 |
| V20297 | 51 | V20297_FP001 … V20297_FP051 |
| V20301 | 4 | V20301_FP001 … V20301_FP004 |
| V20304 | 4 | V20304_FP001 … V20304_FP004 |
| V20317 | 6 | V20317_FP001 … V20317_FP006 |
| V20322 | 8 | V20322_FP001 … V20322_FP008 |
| V20324 | 5 | V20324_FP001 … V20324_FP005 |
| V20329 | 43 | V20329_FP001 … V20329_FP043 |
| V20330 | 21 | V20330_FP001 … V20330_FP021 |
| V20333 | 72 | V20333_FP001 … V20333_FP072 |
| V20334 | 7 | V20334_FP001 … V20334_FP007 |
| V20337 | 5 | V20337_FP001 … V20337_FP005 |
| V20339 | 51 | V20339_FP001 … V20339_FP051 |
| V20344 | 3 | V20344_FP001 … V20344_FP003 |

## 2. 重点回归：39/39通过

| 集合 | 结果 |
|---|---|
| 原11个Tier A normal witnesses | **11/11 PASS** |
| source1987 | 恢复到正确source支持面，保存三角面法向偏差 **0°** |
| v2.2新增15个normal失败点 | **15/15 PASS** |
| v2.2新增12个surface失败点 | **12/12 PASS** |
| 显式source2030点 `(19.434445381,-46.729538687,6.169381551)mm` | **PASS**；射线位移约`4.575e−10mm`，含NUM距离上界`1.005e−7mm`，保存面法向偏差`8.538e−7°` |

回归不是只读取metadata：在保存姿态几何上直接检查solid/void两侧、边界支持面，再确认到**保存的triangle**的距离及实际法向。39项通过面中最大法向偏差约`8.538e−7°`，远小于0.05°。

V20273_FP037的保存坐标极薄，Hull数值未决，**没有把它从候选删掉**。规则化solid查询显式排除此数值未决单元；它的bbox距所有39个回归点至少3.587mm，完全位于各10µm匹配球外，因此不能影响这些局部PASS。它仍是整候选的数值实现阻断，未被追认有效。

source2030由V20334_FP001的原A面承载。source608的finite footprint不再向z≈6.169mm区域外推，本轮没有重复上一轮约0.326mm的材料切除。

## 3. 全A sidewall与其它新增退化检查

326个A source faces、777个冻结外露patch全部检查。645个取得同面连续覆盖子检查通过，773个中心点回归通过；没有取得新的真实normal反证。**这不等于完整surface/contour/normal认证PASS。**

原先点PASS而本轮点UNKNOWN的patch有2个：`E0188:1:1 / source1975`、`E0803:4:1 / source2018`。没有将UNKNOWN叫作真实FAIL，也没有抹去这些回归不确定性。

| 检查 | 结果与边界 |
|---|---|
| A target surface扫描 | 436 triangles /3052固定点；到任意closed facets的最大样本距离约0.01416µm；未检出source外露、runtime VOID且距离>10µm的充分反例。该距离不是已认证union exterior全局最大值 |
| 全A双向surface / contour / normal | **UNKNOWN**；完整实际exterior、contour和replacement责任未闭合，不能用点PASS代替 |
| B扫描 | 1700 triangles /11900点；最大到closed facets距离41.577642µm，未检出充分的>50µm真实退化反例；全域surface/contour仍UNKNOWN |
| C | 没有retier或扩大豁免；内部C面没有被重新要求作为runtime外露面。完整源域/材料集合验收仍UNKNOWN |
| 全局材料Boolean | 原输入无法完成union，返回`Error.NotManifold`；明确记UNKNOWN，没有绕过或更换正式模型 |

未发现充分反例不能证明B/C或所有原PASS patch都未退化。详见[逐patch、点回归及尺度结果](nero-gripper-v23-verification.json)与[新增退化/历史区域检查](nero-gripper-v23-supplement.json)。

## 4. 真实尺度FAIL，不能解释成仅数值未决

**729个实体**仅从保存顶点的某个轴向包围盒宽度，就能独立证明内切球直径小于0.1mm；不依赖薄体LP求解成功与否。

清楚的非数值反例：**V20157_FP028** 的z宽度为 **0.013787870503mm =13.787871µm**。任何内切球的直径均不能超过这个宽度，明显小于要求的100µm。其LP估计约0.008148372mm，结论一致。

保存坐标的精确有理数宽度进一步给出全体最小内切球直径区间：`0 <= d_min <= 8.673617379884035e−16mm`。这是含数值级碎片的坐标上界，**不是声称可靠求出了精确最小直径**。构造原始LP还有4个负估计；负数不具有物理意义，全部保留并标注，未拿来充当真实尺度。

因此即使重点回归通过，候选仍在运行时尺度和碎片数量上明确不合格。该结果定位本次BSP/局部合并实现的问题，尚未证明finite-patch原则本身不可行。

## 5. footprint外材料证明：理想恒等式与保存实现分开

完整split trace有12446个节点。32个root的二分树逐支核验；**572个被删除叶**均显式含有某个注册finite prism的全部约束。原始分割树的理想集合关系因此通过：footprint外分支没有被构造规则删除。

保存后的网格还需要独立实现证书，不能凭这个理想恒等式自动PASS。本轮实际结果：

- 3953个终端区域取得精确有理数Farkas不可行证书；
- 75个区域取得连续saved-convex-cover证书，包含2214条LP对偶界；
- **1639个终端区域未决**；
- 1041个新实体取得正凸组合/距离证书，连续证明其位于原基线材料NUM范围内；另32个实体的这项证明未完成；
- 所有32个root的**完整保存实现outside-preservation状态均为UNKNOWN**。没有发现足以确认footprint外真实损失的反例，也不能据此排除损失。

LP证书不是采样：只在分析cover网格通过精确闭合凸性检查后，才用精确有理数复算对偶上界。虚拟cover的浮点triangulation不能认证为精确支持网格时，保持UNKNOWN，没有放宽NUM或A/B/C规则。

[完整continuous/material证书](nero-gripper-v23-preservation.json.gz) · [全部分割树](nero-gripper-v23-split-trace.json.gz)。gzip是原始JSON的无损封存，原trace约51.6MB，解压SHA及原件位置均在provenance；没有借压缩修改几何。

## 6. Protected void与历史未决

34条backing/背袋连续路径、131个mount占据见证、四个blind-recess轴floor及五条v2.1 central-bore路径全部重查，子检查通过。冻结mount孔本身与原backing阻挡保持原规则。

side channels、rear pocket、five backing bores、four blind recesses、central bore、root slot、major notches的**完整壁面/口缘/连接关系不退化仍UNKNOWN**；没有把有限路径通过或“只减材料”当作完整PASS。保存模型的数值薄体、完整材料集合与outside证书未决仍有影响。

旧10项使用原坐标重查，不拿新后继face index替代历史face。triangle2770取得连续内部覆盖子证书；其余9项仍UNKNOWN。原atlas的4项全部重查，仍4项UNKNOWN。没有在真实尺度FAIL后再重建整套全局atlas以寻求PASS。

## 7. 执行、完整性与停止

Ubuntu实际离线运行。Python3.11.9、NumPy1.26.4、SciPy1.15.3、任务私有Shapely2.1.2/Manifold3D3.5.4。没有安装系统依赖。复核现有候选时只运行后3条，先准备上述依赖；**不要再次运行生成命令**。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/v23_finite_patch.py preflight
# 上一步先提交于1fc695f。build有generation-started/候选文件双重保护。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/v23_finite_patch.py build
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/verify_v23_finite.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/certify_v23_preservation.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/supplement_v23_checks.py
```

成功分析命令退出0表示完成记录，并不表示候选验收通过。原始日志、早期验证错误和执行源码快照在`debug/issue25/v23-finite/`。候选生成后未修改；只修正分析脚本对数值未决单元的分类和结果汇总。

完整SHA、实际import模块、日志、有理数宽度证书、LP对偶与正凸组合的独立复算见[provenance](nero-gripper-v23-provenance.json)。314个未授权实体、19项机械输入、原v2/v2.1/v2.2证据与另11工作树均核验不变。没有正式MJCF、MuJoCo/query/integration、grasp、物理调参、新Issue或main修改；继续同一Issue25 / Draft PR26。

**本轮停止，不生成第二候选。下一步唯一建议：独立审查V20157的split tree及FP028宽度证书，判断如何在保持有限prism材料关系的局部凸覆盖中避免薄remainder；先审构造与证明，再决定后续几何授权。**
