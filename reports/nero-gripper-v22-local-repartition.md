# Issue25：一次 bounded local repartition

**最终状态：`V22_LOCAL_REPARTITION_BLOCKED`。** 唯一候选修复了11条强制回归中的10条，但仍有真实A法向失败，并引入真实A surface退化；因此拒绝该候选。没有第二次几何尝试。

任务：[评论6273457243](https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6273457243)。基线`642d3b0` / 346体候选`6eba4f3`；先在`2d14b71`单独提交[冻结方案](nero-gripper-v22-local-preregistration.json)，之后才生成一次局部候选。

## 1. 实际修改范围

- **仍为346体/指，新增0、删除0、全局合并0。** 只裁剪32个直接覆盖冻结A sidewall patch的允许实体：15 outer_chassis、7 negative-x sidewall、10 positive-x sidewall。
- 其它**314个完整entity对象**的canonical JSON bytes及vertices/faces二进制数组逐字节相同。main7、tip4、ramp7、shoulder3、mount_root118全部未修改。
- v2/v2.1 policy、原346体候选、旧报告、mesh及正式模型均未改变。
- 新候选vertices/faces SHA：`2947f4513b3d9099907349cea82360cbf0748654e40949b7916dea8e293a7f74`。
- 新候选整文件SHA：`c48a76402c013ab21ba62a5649358178f2116d57b5eec5a5ab5a18fde31eb4d1`。

修改ID：

```text
V20140 V20143 V20154 V20157 V20160 V20166 V20175
V20205 V20207 V20245 V20259 V20263 V20267 V20273
V20274 V20280 V20284 V20291 V20294 V20297 V20301
V20304 V20317 V20322 V20324 V20329 V20330 V20333
V20334 V20337 V20339 V20344
```

[旧→新映射](nero-gripper-v22-local-mapping.json)：旧ID保持，32体的vertices/faces局部修订，其余314体不变。[唯一候选](nero-gripper-v22-local-partition.json)只作失败证据，**不可直接作为已通过runtime资产使用**；修订体继承的source-region/support/adjacency元数据只是基线来源，不能替代新union认证。

## 2. 冻结的单一构造及为何失败

构造覆盖全部326个A sidewall源三角面、777个冻结外露patch，而非仅11个点。在每个patch法向外侧2NUM做纯几何半空间相交，识别覆盖该patch的允许邻体；把相关**原始源平面**作为该邻体的内向裁剪平面。只重建这些邻体，未移动源平面，不做全局压缩。禁止实体上的潜在重叠保留为诊断，未越权修改。

这个单一策略的缺陷是：**识别覆盖时用了有限source patch，但随后把其支持平面扩展为整个邻体的裁剪约束。** 它恢复了一些目标A面，却切掉了同一实体在其它高度/位置原本正确的A面。只保证源平面位置不动，仍不足以保护相邻功能面。

可独立复核的新增反例：

- source face2030的外露点，finger局部mm坐标：`(19.434445381, -46.729538687, 6.169381551)`。
- 修订前：位于V20334的真实union边界，surface误差约`1.4e−15mm`，法向偏差约`8.54e−7°`；固定两侧查询为solid/void。
- 修订后：该点为runtime **VOID**；遍历全部closed triangles得到距离`0.325833955687mm`，扣NUM后真实union边界距离下界 **`0.325833855687mm`**，远超10µm，亦超全局材料50µm限制。
- V20334新增的source face608裁剪平面在该点的有符号外侧残差为`0.325833955687mm`。608真实三角面的z范围仅`[2.000000095, 4.000000190]mm`，而受损点z为`6.169381551mm`。这是有限patch被无限外推后的实际切除，不能归咎于数值接缝。
- 该反例已以**全closed-facet穷举距离**独立复算，不只依赖BVH、Boolean exterior或稀疏点到分析atlas的距离。

## 3. 验收结果

| 项目 | 实际结果 | 状态 |
|---|---|---|
| 11条强制normal witness | 10 PASS；source1987仍失败 | FAIL |
| source1987 | 位移0.441698µm；法向0.050264987°；全部局部潜在边界/边缘法向cone的偏差下界0.050254987° | 真实normal FAIL |
| 全部A sidewall | 326源面/777patch全部检查；16个点取得真实normal反证，其中15个点修订前通过、修订后失败，1个原失败仍存在；重复源面的不同patch点分开记录 | FAIL |
| 已确认A normal偏差 | 全域最大值至少3.377081117°，不是已经求出完整最大值 | FAIL |
| 全部A surface | 436个冻结A target triangles、3052个固定点；12个source外露且runtime VOID的点取得真实退化证书，修订前均通过；最大已确认距离下界325.833856µm | FAIL |
| A contour / 全域双向完整证书 | 同面覆盖子检查557/777、点回归704/777不能替代完整union contour/target责任证明 | UNKNOWN，不追认PASS |
| 全346体尺度 | 重新逐体LP，最小内切球直径0.111617620211mm | PASS |
| main/tip/ramp/shoulder/mount_root | 139体完整对象及几何不变；原工作面位置没有移动；新union不增加有效solid，不能新增遮挡已有工作面 | 原几何保持；不借此声称所有新边界通过 |
| Tier B | 1700 target triangles /11900固定点未检出>50µm的充分反例，样本最大41.577642µm；双向surface/contour完整证明未完成 | UNKNOWN |
| Tier C / 全局材料 | 不修改tier或C豁免；全局source-minus-runtime 49.99µm offset差集非空，A反例另证真实材料丢失>50µm；C域/closure责任未全域认证 | 全局材料FAIL；C源域UNKNOWN |

所有normal反证都枚举10µm匹配球外包cube内的全部冻结closed facets，严格内部片段给出连续凸覆盖证据；剩余法向以cone包络，包括边缘法向正组合，并预留`1e−5°`算术裕量。source外露性另用冻结STL装配两侧winding确认。source1987和其它真实超限没有用1e−7mm表示等价规则豁免。

这里没有把有限点无失败当成全域PASS，也没有用UNKNOWN掩盖已经确认的新增A normal/surface失败。完整数值、每patch结果、16条局部normal证书、12条实际surface退化及基线对照见[验收JSON](nero-gripper-v22-local-verification.json)。

## 4. Protected void及旧未决

| 结构 | 新候选检查 | 完整结论 |
|---|---|---|
| 两条side channels | 只有裁剪，没有增加有效solid；不能新堵塞原void。完整净宽/口缘仍依赖未闭合表面证明 | UNKNOWN，未确认真实退化 |
| rear pocket / five backing bores | 34条原连续路径均通过 | 全壁/口缘UNKNOWN |
| 四mount blind recesses | 原占据见证、四轴floor检查通过；mount_root不变 | 全壁/孔唇UNKNOWN |
| central bore，v2.1 | mount孔core及五条source/runtime路径通过，保留source原backing阻挡 | 完整孔缘UNKNOWN |
| root slot | 原slot点检查通过，mount_root不变 | 完整开口轮廓UNKNOWN |
| major notches | 原占据/路径相关检查保留；不以无新增solid豁免墙体损失 | 完整轮廓UNKNOWN |

131个mount占据见证均匹配。严格有理数凸组合证书另外证明每个新实体在原实体内、数值距离上界≤`1.347517e−9mm`，连续覆盖整个新凸体。因此没有>NUM的新void填充，但**这不能排除失去原壁面或开口变大**，没有把protected void完整不退化标PASS。

旧10条未决全部使用原历史坐标重查，仍10条未决：triangles422、441、563、598、614、615、1221、2769、2770，以及旧V20333/face6。**没有把修订后V20333的新face6偷换成旧面。** 原atlas的4条覆盖证明未决继续保留；本轮在真实失败已确立后没有追加一次全局atlas重新细化，更未把旧未知清零。

## 5. 运行与交付

Ubuntu实际执行，仅离线几何。Python3.11.9、NumPy1.26.4、SciPy1.15.3、任务私有Shapely2.1.2/Manifold3D3.5.4。没有系统依赖修改。

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/v22_local_repartition.py preflight
# 上一步先提交冻结方案。仅在尚无候选的工作树执行build；存在候选时脚本拒绝再次生成。
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/v22_local_repartition.py build
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/verify_v22_local.py
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /data_robot/projects/.worktrees/nero-gripper-grasp-diagnosis/.venv/bin/python -B scripts/finalize_v22_local.py
```

已有候选的checkout只重跑后两条验证命令，先创建`debug/issue25/v22-local/`并准备上述Python依赖即可。原始日志在`debug/issue25/v22-local/`；成功构造、主验证、独立证书及交付审计退出0，**不表示候选验收通过**。主验证约34.6秒。完整命令、日志、实际import项目模块SHA、314体bit identity、32体有理数凸包含证明及穷举反例复算见[provenance](nero-gripper-v22-local-provenance.json)。

没有修改原候选/政策/正式模型，没有MJCF、MuJoCo查询/积分、grasp、调参或新Issue。19项机械输入、另11工作树核对不变。继续同一Issue25 / Draft PR26，不合并。

**停止，不生成第二候选。下一步唯一建议：独立审查source608有限z域被外推后切掉source2030的反例；若后续获授权，应改用有限source-patch footprint约束的局部closure重分配，并同时保护相邻A面，不再使用整实体无限平面裁剪。**
