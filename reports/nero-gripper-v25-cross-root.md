# Issue25：只读 cross-root feasibility audit

**最终状态：`V25_CROSS_ROOT_UNKNOWN`。12项已审计：1 FEASIBLE / 0 INFEASIBLE / 11 UNKNOWN。独立有理数证书复核 PASS。** 这不是抓取或runtime geometry验收；本轮未生成新候选。

任务：[comment6290781760](https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6290781760)。冻结来源：原346体 `6eba4f3`、v2.3 finite caps `87ef3b5`、root-local审计 `f32cab0`。旧12条只在root-local范围成立，未直接继承为跨root不可行。

## 1. 冻结范围

- V20157：本体及13个一阶邻居，共14 roots。
- V20291：本体及6个一阶邻居，共7 roots。
- V20329：本体及6个一阶邻居，共7 roots。
- 邻接严格取346体的 `reference_adjacency_v2_ids`，按无向边合并入边/出边；该字段来自原reference的共享边界及cross-part邻接。没有按距离扩大；没有使用二阶邻接。
- [preregistration](nero-gripper-v25-cross-root-preregistration.json) 列出全部ID、输入SHA及规则；冻结SHA `0b449111620f0fa8a83c0ee0799d61a6502dfda4e2149ddb09c95f60291b0aa2`。
- 材料是这些**原baseline roots的并集**，不是v2.3 successor mesh。减去**全部104个冻结finite caps**，因此所有与邻域相交的cap均被纳入；不相交者对集合无影响。没有利用数值LP筛选来排除caps，未改footprint或source plane。

346体geometry SHA保持 `79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e`；v2.3保持 `4050c488325b409532360ee9d8dc3311bd3a53bc36b75949af637bfc8b59fd96`。冻结v2.1 policy不变。

## 2. FP028：跨root后已证明可行

原required point，mm：

`(-16.590000202112005, -34.87594568965148, 4.000000190095088)`

找到一个解析闭凸半空间交集 H，其**全部材料严格位于直接邻居 V20007 内部**；无需让H仍归V20157所有，也无需假定BSP leaf必须独立实体化。球心，mm：

`(-16.534425023786824, -34.87594568965148, 3.9497700119352483)`

已认证内切球直径下界 **0.10000199998mm >=0.1mm**。完整H halfspaces、球约束和证书在 [cross-root archive](nero-gripper-v25-cross-root.json.gz) 的 `targets[entity=V20157_FP028].feasible_certificate`。

连续证明包括：

1. required point满足H全部不等式，不依赖只检查接近点。
2. H有界且闭凸；每条H约束用精确有理数验证球包含。
3. 对原V20007的全部29个精确支持面，保存并复核全H的nonnegative dual上界，**全部严格小于0**。因此是 `H ⊂ interior(V20007) ⊂ union(N)`；没有依赖扩张neighborhood或近似Boolean union。
4. 对全部104个原caps，各给一个反向cap面在**全H**上的dual上界；校正搜索余量后，这104个真实上界也**全部严格小于0**。因此H严格避开所有caps，未填回禁止材料；NUM例外都不需要用于这一最终可行证书。
5. H严格位于 `interior(R_N)`，不参与真正的A/B外露边界，因此不新增A/B越界或法向责任。本H只是内部材料cover存在性证书，不是完整runtime外露面交付。

所以FP028的旧root-local阻断**不能作为跨root不可行结论**。这个实例确认旧ownership限制制造了人工阻断；不能推广为其它11项也都如此。

## 3. FP090：更稳健，但尚未裁决

原点到neighborhood外边界的精确距离约 **6.13706764059e-06mm = 61.3706764 NUM**；到所有caps的最近距离约 **1.04384204161e-06mm = 10.4384204 NUM**。两者都>NUM，因此可作为本轮稳健required point。

已尝试允许落在各一阶邻居中的厚seed、连接required point的解析凸cover，并核验caps及连续union包含。部分搜索有厚seed，但**完整union包含证书没有闭合**；不能据此说真实几何越界或不可行。

另用包住全部14 roots的保守支持凸包，构造完整球心外包域并精确枚举90个顶点，尝试robust cap-shadow。没有一个已测试单cap能对全部连续球心闭合反证。这个外包包含可能不在真实neighborhood/可见域中的球心；没有取得多cap或外部void联合排除证书。

因此FP090是 **CROSS_ROOT_UNKNOWN**，不能继承旧root-local INFEASIBLE，也不能因为MILP或证明搜索失败而称不可行。

## 4. 全12项原required point的余量与裁决

NUM=`1e-7mm`。下表均是**相对NUM的倍数**；mm值=表值乘`1e-7`。这些原点的neighborhood距离都有相等的精确上下界，因此不是只报告旧root内部面的距离或采样估计。

| target | 到N外边界 / NUM | 到所有caps最小距离 / NUM | 两者均>NUM | 最终分类 |
|---|---:|---:|---|---|
| V20157_FP028 | 0.00999639542 | 0.0100000186 | 否 | FEASIBLE |
| V20157_FP090 | 61.3706764 | 10.4384204 | 是 | UNKNOWN |
| V20157_FP001 | 0.0108703855 | 4.17987276e-05 | 否 | UNKNOWN |
| V20157_FP002 | 0.0199712832 | 4.22248729e-05 | 否 | UNKNOWN |
| V20157_FP008 | 161.626454 | 4.01560483e-06 | 否 | UNKNOWN |
| V20157_FP009 | 0.015243654 | 3.7492664e-06 | 否 | UNKNOWN |
| V20157_FP088 | 0.00331031854 | 2.39359335e-06 | 否 | UNKNOWN |
| V20157_FP097 | 0.00253855937 | 2.49881464e-06 | 否 | UNKNOWN |
| V20157_FP099 | 0.00508593086 | 3.1721867e-06 | 否 | UNKNOWN |
| V20157_FP101 | 18.6257573 | 0.135041565 | 否 | UNKNOWN |
| V20291_FP006 | 0.000106569139 | 1.62143967e-06 | 否 | UNKNOWN |
| V20329_FP031 | 47.9856474 | 0.744964928 | 否 | UNKNOWN |

**11个旧点至少处于一种<=NUM带内，只有FP090通过原点稳健性guard。** 没有用这些带内点证明CROSS_ROOT_INFEASIBLE。FP028的FEASIBLE由实际集合包含证书建立，不要求用不稳定点证明不可行。

边界距离方法：下界由包含required point的某个真实root中的完整球证明；上界由穿过**全部neighborhood roots并集**的精确射线区间首次进入void证明。原12点上下界恰好相等。Cap距离用完整有限cap的精确投影/KKT及所有其它caps的连续下界证明；不是只取一个无限support plane的距离。

## 5. 更稳健的同区域点

在每个target原assigned retained leaf区域内都找到一个更稳健点；逐点精确核验原root、原leaf所有不等式及全部104 caps。Leaf保存坐标只用作搜索种子，未用作集合真值。对NUM带内点，没有将“没找到cover”提升为不可行。

| target | 新点到N外边界 / NUM（认证区间） | 新点到所有caps最近距离 / NUM |
|---|---:|---:|
| V20157_FP028 | [5.57893e+06, 6.52011e+06] | 6.51689e+06 |
| V20157_FP090 | [4497.06, 4497.06] | 56642.1 |
| V20157_FP001 | [10870.4, 10870.4] | 41.2254 |
| V20157_FP002 | [19971.3, 19971.3] | 42.2196 |
| V20157_FP008 | [13100.4, 13100.4] | 4.023 |
| V20157_FP009 | [15243.7, 15243.7] | 3.79339 |
| V20157_FP088 | [3310.3, 3310.3] | 2.39443 |
| V20157_FP097 | [2538.54, 2538.54] | 2.49703 |
| V20157_FP099 | [5085.91, 5085.91] | 3.17198 |
| V20157_FP101 | [1.56563e+07, 1.83421e+07] | 1.86764e+07 |
| V20291_FP006 | [106.638, 106.638] | 1.53039 |
| V20329_FP031 | [10430.8, 10430.8] | 56092 |

完整坐标、mm值、NUM倍数、精确margin证书和leaf身份见 [robust points JSON](nero-gripper-v25-robust-points.json)。FP090继续用原稳健点，其余10个未决对象用同区域新稳健点，进一步尝试了bounded cross-root单cap shadow；仍没有闭合任意凸cover不可行证书，见 [obstruction attempts](nero-gripper-v25-obstruction-attempts.json.gz)。稳健点存在不等于已证明整个target可行或不可行。

## 6. 实际验证、限制与交接

Ubuntu实际执行，四条命令均退出0：

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
PY=/data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python
$PY -B scripts/audit_v25_cross_root.py
$PY -B scripts/audit_v25_robust_points.py
$PY -B scripts/audit_v25_infeasibility.py
$PY -B scripts/verify_v25_cross_root.py
```

另一台机器在完整clone中可用 `python3.11 -B scripts/verify_v25_cross_root.py --certificates-only` 做纯只读证书复核；该选项跳过本机其它worktrees审计，也不写输出文件。本轮记录的是默认完整复核。

标准库独立复核器不导入优化器或分析helper；复用上一轮纯有理数replay工具，重新验证冻结支持面、邻接、全部原/替代点的距离、H严格包含与球约束。复核 24 组margin、133 条dual、22 条球约束，PASS。19项机械输入及11个受保护工作树均未变。结果和SHA见 [verification](nero-gripper-v25-verification.json) / [provenance](nero-gripper-v25-provenance.json)。

主要限制：构造搜索使用有限cap-facet separator和axis-aligned厚seed，并不完备；不可行搜索使用保守球心外包和单cap shadow，尚未覆盖任意多障碍/visibility组合。11项UNKNOWN代表证明能力不足，不代表真实失败。没有把1项可行和11项UNKNOWN叫作MIXED；按Issue定义整体是UNKNOWN。

工作树 `/data_robot/projects/.worktrees/nero-gripper-runtime-partition`；原日志 `debug/issue25/v25-cross-root/`。未处理343个UNKNOWN；未生成v2.4/v2.5 runtime geometry、MJCF，未运行MuJoCo/query/积分/抓取，未改policy、vertices/faces或main。

**本轮停止，交ChatGPT独立审查。** 下一步唯一建议：先审查FP028的严格V20007包含/104 cap分离证书，以及其余11项保留UNKNOWN的范围；不据此生成runtime候选。
