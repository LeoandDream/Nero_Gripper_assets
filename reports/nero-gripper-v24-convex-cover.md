# Issue25：只读 overlapping convex-cover feasibility audit

**最终状态：`V24_CONVEX_COVER_MIXED`；独立证书复核 PASS。** 本报告仅分析冻结 retained set，没有生成 v2.4 candidate、runtime mesh 或 MJCF。

任务：[comment6284738744](https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6284738744)。基线是原 `6eba4f3` 的346体和 `87ef3b5` 的有限 caps/leaf 身份；不是失败 v2.3 successor mesh 的并集。346体 geometry SHA `79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e`；v2.3 geometry SHA `4050c488325b409532360ee9d8dc3311bd3a53bc36b75949af637bfc8b59fd96`。两者 vertices/faces、finite footprints、v2.1 policy 全部不变。

## 1. FP028：无法用合法厚 cover 吸收

V20157 确认有9个 finite caps、144个 raw retained leaves、129个 saved successors。`V20157_FP028` 对应 raw node422，z宽约 **0.013787870503mm**。宽度本身只说明当前实体薄，**不是本次不可行证明**。

在 node422 中找到必须保留的点，单位mm：

`p = (-16.590000202112005, -34.87594568965148, 4.000000190095088)`

精确有理数检查证明：p 在原 root 凸包内、在 node422 的半空间区域内、在全部9个 deleted caps 外。阻断来自 **P0064 / source1985** 的有限 cap。证据见 [obstruction archive](nero-gripper-v24-obstructions.json.gz)，定位 `roots[baseline_entity=V20157].classifications[entity=V20157_FP028]`。

证明覆盖任意凸 cover，不限于选用 cap 原有面作分隔面的搜索：

1. 假设 H 合法且有半径 `r=0.05mm` 的内切球，球心 c 必须在 baseline root 的 inward-offset center polytope 中。分析还把 root 向外预留 NUM=`1e-7mm`，因此不会因 root 的表示容差错误排除合法球心。
2. 用冻结 root 顶点的**整数叉积**从冻结米制顶点精确乘1000后枚举全部真实支持面；用整数 Cramer determinant 枚举 center polytope 的全部7770组活动面三元组，得到 **58个精确顶点**。未依赖有限球心采样。
3. 对每个顶点 c，给出 shadow point s 和 `t>=1`，满足：
   - `k=(s+(t-1)p)/t` 位于 P0064 内，距每个 cap 面至少 `5 NUM`；
   - `||s-c||₁ + 3 NUM·t < r`。
4. 因而 `conv(p,ball(c,r))` 内含一个以 k 为中心、半径 **3 NUM** 的球，同时该球在 deleted cap 内。上述条件在 (c,s,t) 上是凸的；58个顶点的证书通过凸组合推广到**全部连续球心集合**。
5. 即使把 p 的覆盖位置允许偏移 NUM，仍留下半径至少 **2 NUM** 的真实禁止材料重叠，不能归入“<=NUM 且无更大有效 solid/void 区域”的 closure 表示等价。

所以：**不存在满足本轮 root-local、cap-disjoint、内切球直径>=0.1mm 条件的 H，能够覆盖 FP028 的全部必须保留材料。** 允许 cover 跨多个 leaves、允许 cover 互相重叠，都不能消除这个点的阻断。任意有限 cover 集合若覆盖 p，至少其中一个 H 必须覆盖 p，同样矛盾。

这只裁决**当前冻结 root-local retained-set oracle**，不证明 NERO 实物存在13.8µm功能薄壁，也不证明放宽 root-local 范围后仍不可能。

## 2. V20157整体：部分是碎片，但完整厚覆盖不可行

| 项目 | 结果 |
|---|---:|
| raw leaves | 144 |
| 原 saved successors | 129 |
| 合法厚 analysis covers | 19 |
| 被连续覆盖的 raw leaves | 92 |
| 其余尚未被该组 cover 覆盖的 leaves | 52 |
| 19个 cover 的最小已认证内切球直径 | 0.2465162542mm |

19个 cover 覆盖92个 leaves，证明“一个 BSP leaf 必须成为一个 runtime geom”确实过度约束。它们是闭凸半空间交集，能跨 leaves，允许内部重叠。对应完整 halfspaces、球心、内切球下界和逐叶连续证书均保存在 [cover archive](nero-gripper-v24-convex-cover.json.gz) 的 V20157 项。

但这不是“19个替换129个”的完整方案：FP028 等10个 V20157 successors 含有已认证不可行点。**在本轮固定 root-local / caps / 0.1mm 条件下，无论用多少个合法厚 cover，都不能覆盖全部144个 leaves。** 其余33个 scale-failing successors 仍 UNKNOWN，不能推断它们也不可行。

## 3. 全729个明确 scale-failing successors

| 分类 | 数量 | 含义 |
|---|---:|---|
| ABSORBABLE | 44 | 找到直接吸收其全部 assigned oracle leaves 的厚 cover |
| REPRESENTATION_FRAGMENT | 330 | 全部 assigned oracle leaves 已由其它厚 cover 连续覆盖，无需独立实体 |
| GEOMETRICALLY_INFEASIBLE | 12 | 必须保留点取得任意凸 cover 都无法回避的 robust cap-shadow 证书 |
| UNKNOWN | 343 | 未取得厚覆盖证书，也未取得真正不可行证书 |
| 合计 | 729 | 仅原报告 `scale.status=FAIL` 的729项 |

逐项结果见 [classification JSON](nero-gripper-v24-classification.json)。12条不可行项是：

- V20157：FP001、FP002、FP008、FP009、**FP028**（P0064）；FP088、FP090、FP097、FP099、FP101（P0067）。
- V20291：FP006（P0050）。
- V20329：FP031（P0071）。

跨32个原 roots 共取得 **165个安全厚 analysis covers**，最小已认证内切球直径 **0.14996738672mm**。它们只构成部分覆盖，不是可行 runtime 数量预测。因为存在已证明的必须保留点阻断，当前约束下**完整可行 cover 数量没有有限上界方案**；不能承诺“约165体即可交付”。

## 4. 连续集合证书及边界

**Cover 安全性：** 对冻结 baseline 顶点用精确整数谓词枚举全部支持面，再构建有显式向内舍入证明的 halfspaces。因此每个 H 严格在原 root 凸包内。对每个原 finite cap，H 包含至少一个反向 cap facet 不等式，所以不进入 cap 内部；边界相接允许。没有扩大 footprint、没有删除/合并 cap，也没有用 successor mesh 当集合真值。

**尺度：** 保存球心与保守半径下界。每条 H 约束用有理数验证 `margin>=0`、`margin² >= radius²·||normal||²`。这证明最大内切球直径至少达到报告下界，不假定 LP 最优值精确。

**覆盖：** 对整个 oracle leaf 区域作 LP，再保存并独立重算非负 dual weights 与有理数余项/bbox 上界。不是检查若干 leaf 顶点或采样点冒充连续认证。极小表示误差通过向已认证内切球心的 homothety 处理，逐叶给出全区域距离上界，均<=NUM；浮点减法的实际舍入量也纳入该上界。V20157 最大覆盖距离上界约 `3.31e-9mm`，远小于 NUM。

**未宣称通过的事项：** 没有冻结 runtime union，因而未重新宣称全部 A surface/contour/normal、protected void 或完整 retained terminal obligations 已通过。已有 cover 不进入任何 cap、不添加 root 外材料，但部分覆盖不等于完整 A 外露面验收；它们的新内部面也不能直接作为 runtime 外露面交付。搜索仅尝试原 cap facet separator 的方案，不完备；找不到方案始终 UNKNOWN。真实不可行的12项由独立的任意凸 cover 反证负责，不能由这个搜索的失败推导。

## 5. 复核与复现

在 Ubuntu，用原有 Python3.11 环境实际执行纯离线分析，未导入/调用 MuJoCo：

```bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
PY=/data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python
$PY -B scripts/analyze_v24_convex_covers.py
$PY -B scripts/extend_v24_obstructions.py
$PY -B scripts/verify_v24_certificates.py
```

[cover search](../scripts/analyze_v24_convex_covers.py) · [robust obstruction kernel](../scripts/certify_v24_point_obstruction.py) · [729项归类](../scripts/extend_v24_obstructions.py) · [独立复核器](../scripts/verify_v24_certificates.py)。复核器只用 Python 标准库，不导入 SciPy、优化器或本项目分析 helper；从原始输入重算全部支持面、球心顶点、dual bounds、内切球及 shadow 不等式。本次实际复核 165 个 cover、4563 条 dual bounds、5406 条球包含约束、592 条 shadow 顶点证书，退出码0。`certify_v24_point_obstruction.py` 的 standalone FP028 main 为调试定位入口，正式完整证书由 extend 脚本输出。

证据根目录：本工作树 `reports/nero-gripper-v24-*`；运行原日志 `debug/issue25/v24-cover/`。输入、脚本、环境、退出码及 SHA 见 [provenance](nero-gripper-v24-provenance.json)，独立复核计数见 [verification](nero-gripper-v24-verification.json)。main、机械资产和其它受保护工作树未修改；继续同一 Draft PR26，不合并。

**下一步唯一建议：** ChatGPT 先独立复核 FP028 的 robust cap-shadow 集合反证，尤其是 NUM 扰动余量及 root-local 前提。确认前不授权生成 v2.4；当前证据不能把问题仅归因于 BSP 碎片化。
