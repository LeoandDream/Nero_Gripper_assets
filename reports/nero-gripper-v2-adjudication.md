# Issue #25：346体候选只读真值裁决

任务：[评论6271215742](https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6271215742)。2026-10-20，Ubuntu22.04。候选仍为 `6eba4f34c5ff5029eb849804e3d2e9c535f58756`；未修改任何候选几何、半空间、冻结规则或旧报告。

**Exterior：`V2_EXTERIOR_ADJUDICATION_MIXED`。** 既有可证明的 certifier/Boolean 误报，也有保存半空间系数字面值下的极微真实缝隙；14条记录尚不能给出单一可靠分类。未证实宏观几何缺陷，也不能宣布只修certifier就可以通过。

**Central bore：`POLICY_SOURCE_CONTRADICTION_CONFIRMED`。** mount本身的五条孔内路径贯通，冻结backing在完整装配中原本就封住跨接口出口；v2保持原阻挡。

原候选仍是 **`V2_RUNTIME_ENTITY_PARTITION_BLOCKED`**，本轮不是重新验收。

## 1. 固定方法和真值范围

分析前已冻结[裁决方案](nero-gripper-v2-adjudication-preregistration.json)，SHA256 `8da86b225313d52a99f6bb500457231882471fa34198a1c8aa18d27f34a759a3`。

- membership直接计算346组保存半空间的union，单位m，不使用Boolean exterior作solid/void真值。三个固定法向偏移为 **`1e−6 / 1e−5 / 1e−4mm`**，初级容差 **`1e−10mm`**。
- 一般点在容差带外用binary64判定；带内只对附近cell使用longdouble及算术误差界，必要时以`Fraction.from_float`对保存系数的binary64字面值作精确有理数判号。没有挪点修面、snap候选或改系数。
- 对66个输出三角形取预注册的7个内部重心组合点。对268个missing记录，重现旧certifier**真正缺失的小区域**，再取每个连通片的稳定内部点；对21条occlusion记录同样检查其被指控的core patch。原大三角形的7点也保留，不能拿其多数内部点代替小缺失区。
- 旧Boolean mesh、Shapely二维运算只用于重现“当时究竟在怀疑哪里”。这些定位结果的solid/void仍由直接半空间独立决定。没有运行新的三维Boolean构造。
- 分类表按这些固定点/可疑patch统计。`TRUE_BOUNDARY`指三种尺度都稳定地一侧solid、一侧void；不是整个三角形/全部union的连续证明。`BOOLEAN_PSEUDOFACE`除双侧solid点证据外，另尽量给出**整个疑似patch所有顶点严格位于同一cell内**的连续证明。覆盖/尺度不一致和落在表示数值缝中的项保持`NUMERICALLY_UNRESOLVED`。

## 2. 全部失败记录的裁决

| 旧记录集合 | TRUE_BOUNDARY | BOOLEAN_PSEUDOFACE | NUMERICALLY_UNRESOLVED | 总数 |
|---|---:|---:|---:|---:|
| unattributed output triangles | 52 | 2 | 12 | 66 |
| missing true external faces | 2 | 264 | 2 | 268 |
| **certificate failures合计** | **54** | **266** | **14** | **334** |
| occlusion suspect records | 0 | **21** | 0 | **21** |

334条中，**173条**的可疑patch还取得了严格单cell内部覆盖的连续证明；21条occlusion则 **21/21** 均取得这种证明。因此误报并非仅由有限采样猜测。其余点分类不冒充全片证明。21个core patch的伪面结论也不表示其父三角形每一部分都是伪面；原先直接用父三角形中心会错过小遮挡区域。

14个未决记录：输出三角形`422,441,563,589,598,614,615,1210,1221,2769,2770,2773`；missing记录`V20177/face3`及`V20333/face6`。部分是同一面内boundary/interior混合，部分是两侧solid但面点处于系数微缝中；更多算术精度不能自动把整片变成一种语义类别。

逐条证据见[JSON](nero-gripper-v2-adjudication.json)和[CSV](nero-gripper-v2-adjudication.csv)，包括原大面点、真正失败patch点、每尺度两侧membership、局部高精度记录及连续覆盖证据。

## 3. triangle2940及0.619918918mm反例

**triangle2940最终分类：`NUMERICALLY_UNRESOLVED`。** 7个固定内部点与原反例点共8点均未满足可靠单一边界/伪面条件：三个固定偏移尺度上两侧都是solid，但保存系数字面值会在中间留下极微void。不能直接把原Boolean三角形宣布成真实外壳，也不能只因两侧solid就抹去该void。

在原反例点 `(6.599999964237212,−64.42487363125,7.798575050000002)mm` 的法向直线上，局部精确区间合并发现：

| 项目 | 精确字面半空间的结果 |
|---|---|
| 开区间起点t | 约`−1.72636810667e−15mm` |
| 开区间终点t | 约`+4.95765532966e−16mm` |
| void宽度 | **`2.222133639637184e−15mm`** |
| 左端solid owner | V20221 |
| 右端solid owner | V20218 |
| 距全部原始STL三角面的最近距离 | **0.619918917738mm**，最近面711（B） |

这不是只查附近3个cell就推断union：保存了**全部346个cell各一条分离半空间**的精确有理数证书。每个证书在两个端点的残差非负、至少一个严格为正，因此整个开区间不属于该cell；两个端点则分别满足其owner全部闭半空间。该开区间及端点确实属于保存系数字面union的void/boundary。

因此，**如果按本轮指定的保存halfspaces字面值作真值，仍存在超过所有source A/B预算的真实union boundary反例**。到全部原始STL面的距离已超过0.05mm，故任何A/B target子集的距离也不可能更小；没有扩大C豁免。距离查询最后转回binary64时的坐标误差远小于超预算余量。

但 **0.620mm是该端点到源表面的距离，不是裂缝宽度**。缝宽比原`1e−7mm`数值分辨率小约4500万倍。所有保存顶点相对保存半空间的最大外侧残差也仅约`1.362e−13mm`。本证据揭示的是闭合分区在极高精度字面解释下的数值级表示问题，**不能外推为宏观缺口、实物缺陷或抓取失败原因**。

其余固定尺度下的真实边界点到原STL面最大实测约 **0.021390342mm**，该最大点对应B类背袋侧壁；最近源面属于A的点最大约`9.612e−7mm`。没有发现这些点上的宏观A/B超预算反例。但这是有限记录的局部裁决，不是全域表面/法向重新验收，也不能排除尚未认证区域的问题。

## 4. central through-bore：直接原始STL证据

直接对冻结STL的原始三角面进行精确有理数线面交点与逐零件占据区间计算，未借用reference凸体来证明source。源面范围：mount `0–503`，contact `504–659`，backing `660–2085`。

| x,z(mm) | mount整条y线 | contact整条y线 | backing实体区间y(mm) | v2在规定路径内的实体区间y(mm) |
|---|---|---|---|---|
| 0,12.25 | clear | clear | −70.000000298..−64.000003040 | −70.000000298..−69.9999 |
| +0.8,12.25 | clear | clear | 同上 | 同上 |
| −0.8,12.25 | clear | clear | 同上 | 同上 |
| 0,13.05 | clear | clear | 同上 | 同上 |
| 0,11.45 | clear | clear | 同上 | 同上 |

规定路径为`y=−72.9999..−69.9999mm`；这些轴线均没有平行/边界交点未决。mount中央孔壁是原面449–479，位于约`y=−73..−70mm`，五条线穿过mount本身，无cap；backing原本从其出口后开始占据，完整装配不是自由贯通孔。v2占据者V20203保持同一前界，没有新增封堵。

源码装配检查还披露：mount/contact原STL的极小近零坐标不完全逐位共点；用此前已采用的`1e−12mm`分析索引等价检查后，两者均有向闭合，最大索引对应位移≤`1.323e−14mm`。**这只审查拓扑索引，没有写回或移动源顶点；本节的射线交点始终使用原坐标。** backing原始位级共点即有向闭合。

故输出 **`POLICY_SOURCE_CONTRADICTION_CONFIRMED`**。唯一修正规则提案是：

> 保留mount part中央孔本身；完整装配的connectivity必须与冻结source assembly一致。若backing原本封堵，不要求runtime人为挖通。

这只是v2.1 policy correction proposal，**没有修改8a5518b规则，没有追认旧候选PASS**。

## 5. 是否需要第二版几何

**目前不能判定需要第二版顶点/面几何，也不能证明只修certifier/policy就足够。** 已证明的两件事是：certifier有真实误报；系数字面半空间存在低于原分辨率许多数量级的微缝。14条未决、完整外壳/法向认证仍未完成。若坚持每个保存系数的无限精度字面集合都是最终几何，则这一微缝反例必须后续独立授权处理，不能通过忽略它来宣告PASS。

下一步唯一建议：ChatGPT独立审查上述全cell区间证书、14条未决及中央孔规则提案，确定表示精度问题应归于certifier还是需要几何修订；在此之前保持同一346体候选不动，不生成第二候选。本轮不实施修复或重新验收。

## 6. 复核与输入保持

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B \
  scripts/adjudicate_v2_partition.py
```

最终程序退出0仅表示只读裁决报告生成完成。Python3.11.9 / NumPy1.26.4；旧patch定位复用任务私有Shapely2.1.2，无Manifold/MuJoCo导入。脚本及所有实际项目import的SHA保存于结果JSON。分析修正和初始粗定位日志保留在 `debug/issue25/v2-adjudication/`，没有覆盖旧Run或6eba4f3证据。单独算术sanity和精确区间证书复核见[交付核对](nero-gripper-v2-adjudication-provenance.json)。

候选JSON文件逐字节与6eba4f3相同，包括全部halfspaces；几何SHA仍为 **`79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e`**。没有第二候选、MJCF、MuJoCo、grasp、调参、active model修改、规则改写或main修改。
