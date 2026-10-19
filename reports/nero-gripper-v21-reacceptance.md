# Issue #25：v2.1规则冻结与同一346体重新验收

**最终状态：`V2_RUNTIME_ENTITY_PARTITION_BLOCKED`。** 本轮已确认真实的 **Tier A法向对应失败**，同时仍有外壳/分域/轮廓验证器未决项。没有确认protected void的真实破坏；其完整验收尚未闭合。

任务：[评论6271873067](https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6271873067)。先以独立提交 **`30a028ce8868f602d077ab02630ed919e0e4fa21`** 冻结[v2.1规则](nero-gripper-v21-fidelity-protocol.json)及[说明](nero-gripper-v21-fidelity-protocol.md)，然后才执行重新验收。仅修central bore规则，并明确`1e−7mm`表示等价；其它A/B/C预算、protected void和0.1mm实体尺度要求全部继承`8a5518b`。

固定候选仍为`6eba4f34c5ff5029eb849804e3d2e9c535f58756`的 **346体/指**。候选JSON（包括保存halfspaces）逐字节未变，vertices/faces SHA仍为：

`79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e`

## 1. 可以独立确认的真实失败

不是从未认证外壳中的大距离直接推断失败。本轮另枚举全部冻结closed facets，在11个原始A侧壁点构造了独立局部证书：

1. 原始STL与源装配winding确认该点是外露A面。
2. 半径10µm匹配球被一个略加数值裕量的轴对齐cube包含。
3. 枚举所有可能提供外壳的closed facets，裁切到cube；对被其它凸体严格埋住的部分给出全部顶点不等式，从凸性证明整个局部片段均为内部。
4. 对剩余所有可能外露面的法向建立圆锥界，连边缘法向的正组合也纳入。11个点的法向偏差下界均超0.05°，最大已认证下界 **1.310005796°**，已保守扣除`1e−5°`角度计算裕量。这是全局最大值的下界，未声称求出了全域最大角度。

最直观的反例：原STL **face2006**（0-based，finger局部mm坐标），点
`(-27.072354965, -65.350124240, 19.501393102)`。原A面在 **V20278** 中仍存在，却被相邻 **V20280** 覆盖。沿源法向约 **0.000227913mm = 0.227913µm** 到达真实外壳，直接union查询两侧为solid/void；该外壳法向偏差 **0.366178608°**。局部全部可能外露法向的圆锥半径（含计算裕量）≤`1.09e−5°`，故不能靠选择另一个边缘法向满足0.05°。

所有证书中最小严格埋入余量约 **0.000032121mm = 32.121nm**，明显大于`1e−7mm = 0.1nm`数值支持尺度。这些真实覆盖不能用表示等价规则豁免。

证据：[独立法向证书](nero-gripper-v21-normal-witnesses.json)，[脚本](../scripts/verify_v21_normal_witnesses.py)。脚本先用 **0.1°的更宽候选superset**构造保守反证，再检查全部局部法向圆锥；0.1°只用于增加可能匹配者，**验收预算始终是0.05°**。没有改预算或retier。

工程含义：只保证某个单体仍含A支持平面，不保证该A面在完整union中继续外露。当前候选在并集层被相邻凸体覆盖，法向保真丢失；单修certifier/policy不能消除已经确认的失败。本轮不改分区。

## 2. Scale、A/B与C完整报告

**Scale：PASS。** 346个实体重新逐个LP求最大内切球，最小直径 **0.111617620211mm**，全部≥0.1mm。

下表是新分析外壳及当前A/B对应域上的**条件数值**，单位mm。全域exterior/target correspondence尚未完成认证，因此表中的诊断FAIL不能直接称作真实毫米级机械缺陷；真实A失败由上一节的独立证书确认。

| 测量 | 实测下界 | 连续保守上界 | 条件计算结果 |
|---|---:|---:|---|
| A surface：source→runtime | 0.008786633 | 0.009782993 | ≤0.01方向通过 |
| A surface：runtime→source | 2.000006357 | 2.007454444 | 诊断超限，分域/外壳未认证 |
| A contour：source→runtime | 0.012242120 | 0.013477491 | 诊断超限，轮廓集未认证 |
| A contour：runtime→source | 17.803643671 | 35.446642603 | 诊断超限，轮廓集未认证 |
| B surface：source→runtime | 0.044446321 | 0.049445574 | ≤0.05方向通过 |
| B surface：runtime→source | 3.763333173 | 14.223429460 | 诊断超限，上界较松，外壳未认证 |
| B contour：source→runtime | 1.749999821 | 3.499999692 | 诊断超限，轮廓集未认证 |
| B contour：runtime→source | 19.378720155 | 35.933695810 | 诊断超限，轮廓集未认证 |

A双向法向覆盖计算有37/1975个片段未认证；匹配成功子集最大0.048567336°不能当全域最大。**A法向最终FAIL，全域最大至少1.310005796°**，由独立局部证书给出。A/B完整双向surface/contour证书保持UNKNOWN，没有用上述未认证大数值冒充真实缺口。

两个诊断最大点直接查询仍是双侧solid，证实不能按其A/B标签距离宣称机械破坏。轮廓诊断的一个最大点`(0,-41.9454,2)mm`落在原main工作面内部，当前轮廓集尚未与真实机械轮廓完成对应。所有原始数值、求值节点、失败点和上界均保留于[完整结果](nero-gripper-v21-reacceptance.json)。

**Tier C：材料集合子检查PASS，完整C吸收资格UNKNOWN。** 原始STL实心并集与未改候选的双向固定offset差集均为空，材料集合变化保守上界 **0.04999mm < 0.05mm**。74个C旧实体身份重新核对均已退休；C源域、局部gap/thickness≤0.1mm及新外露封口责任仍未完成全域认证，有740个分析片段需要进一步溯源。当前法向cone检查只是C来源证明的充分条件，不是新增C法向预算；未证明时没有判真实C违规或擅自扩大C豁免。

## 3. 旧疑点与表示等价

验证表示从固定vertices/faces重新推导凸支持平面；没有回写候选halfspaces。内部mask使用连续凸支持条件；凹多边形与孔洞保持，不能再取凸包补回内部区域。另有105个分析三角片取得对向材料的连续小gap证书。输出是分析用boundary atlas，**不是第三版collision candidate**。

| 旧集合 | 连续内部 | ≤NUM表示等价 | 稳定边界点 | 稳定内部点 | 未决 |
|---|---:|---:|---:|---:|---:|
| 66 unattributed | 4 | 7 | 44 | 1 | 10 |
| 268 missing | 173 | 20 | 2 | 72 | 1 |
| 21 occlusion | 7 | 0 | 8 | 1 | 5 |

连续证明与有限点分类明确分开。旧21条是在旧字面halfspace真值下裁决的；本轮以vertices/faces为主重新检查，没有自动继承旧证明，也没有改写旧报告。

**triangle2940：表示等价通过。** V20218/V20221对同一三角形的冻结顶点距离、对向支持面及角度跨度给出连续gap上界 **`1.406250575e−8mm` < `1e−7mm`**，没有更大有效void。旧`2.22e−15mm`字面系数缝不再作为独立几何缺陷；旧0.6199mm是到源面的距离，不能据此判本轮几何失败。

**旧14项：3项表示等价、1项连续内部、10项仍未决。** 已解决的是triangle589、1210、2773及V20177/face3。其余10项详见[逐项JSON](nero-gripper-v21-rechecks.json)与[CSV](nero-gripper-v21-rechecks.csv)。

全域外壳还有4个facet覆盖证明未决；不能仅因残余面积很小就豁免。它们属于验证器未闭合，尚不是已确认几何缺口。分析中发现并修正了二维buffer拓扑错误、混合维度及凹patch被错误凸化的问题，初始错误日志保留。

## 4. Protected void与历史168点

| 结构 | 本轮已通过的子检查 | 完整验收 |
|---|---|---|
| 两条0.15mm side channels | 连续净宽下界各 **0.050019545mm**；source侵蚀50µm后的连通core保留 | UNKNOWN：完整壁面/口缘轮廓未认证 |
| 大背袋 | 9条连续开口线段通过，source void core保留 | UNKNOWN：全壁面/轮廓/loop关联未认证 |
| 五个backing孔 | 25条连续通路通过，source void core保留 | UNKNOWN：完整入口/出口轮廓未认证 |
| 四安装盲凹座 | 四轴深度通过，冻结占据点通过 | UNKNOWN：完整孔唇/壁面loop未认证 |
| central bore（v2.1） | mount孔core及五条轴线/偏置路径；source原backing阻挡与v2一致，通过 | UNKNOWN：完整壁面/孔缘仍依赖B认证 |
| root slot | 15个冻结slot占据见证及source void core通过 | UNKNOWN：完整开口/轮廓未认证 |
| 主要缺口 | 固定source void core保留；未改原资产 | UNKNOWN：完整A/B轮廓未认证 |

34条backing/背袋连续路径、131个mount相关占据见证全部通过。**没有确认protected void的真实破坏，也没有宣称全套void验收PASS。** 原source减去runtime的极小数值offset差集仍非空，因此不能仅靠材料距离把所有连接关系宣布通过。

中央孔规则的source contradiction已消除：新规则接受原装配backing阻挡，不要求挖通；并不豁免孔壁/口缘的B类50µm要求。

**旧168条A异常点：168/168直接验证通过，距离保守上界≤0.002930572mm（2.930572µm）。** 这次不只测到分析mesh的最近距离：以直接凸union的可靠solid/void端点证明10µm球中存在真实边界；端点clearance超过NUM，避免把系数微缝当物理外壳。原始查询值最大2.930170µm与直接存在性上界分开保存。证据见[168点证书](nero-gripper-v21-historical-certificates.json)。

## 5. 复现、完整性与停止点

Ubuntu22.04；Python3.11.9、NumPy1.26.4、SciPy1.15.3、任务私有Shapely2.1.2/Manifold3D3.5.4。本轮未安装系统依赖。在当前工作树可直接运行下列命令；独立checkout先准备这些版本及此前脚本所依赖的冻结source审计输入，并从随提交的provenance恢复本轮只读缓存入口：

```python
import json, pathlib, shutil
p = json.loads(pathlib.Path('reports/nero-gripper-v21-provenance.json').read_text())
task = pathlib.Path('debug/issue25/v21-reacceptance')
task.mkdir(parents=True, exist_ok=True)
(task / 'freeze-before-reacceptance.json').write_text(json.dumps(p['frozen_run_preregistration']))
shutil.copyfile('reports/nero-gripper-v21-target-surfaces.npz', task / 'tier-targets.npz')
```

随后在仓库根目录下依次执行：

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/v21_boundary_atlas.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/verify_v21_partition.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/verify_v21_normal_witnesses.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/verify_v21_partition.py --finalize-existing
```

完整表面/轮廓主计算实际约1810秒，最终退出0；随后补入独立法向与历史点证书。退出0表示检查及报告生成完成，验收状态仍BLOCKED。原始规则冻结、运行命令、分析修正、已执行主程序源码及SHA在`debug/issue25/v21-reacceptance/`；可携带证据见[来源与交付核对](nero-gripper-v21-provenance.json)。分析外壳、分域数组与未决证据均随本提交保存，旧证据没有覆盖。

没有修改候选vertices/faces、保存halfspaces、active model、正式MJCF或物理参数；没有生成第三版几何，没有导入MuJoCo、积分或grasp，没有修改main。

**本轮停止。下一步唯一建议：独立审查A侧壁并集覆盖反例（先看source2006的V20278↔V20280），再决定是否授权仅针对A外露/法向保真的分区修订。不要通过继续修certifier或放宽预算，把当前真实A失败追认为PASS。**
