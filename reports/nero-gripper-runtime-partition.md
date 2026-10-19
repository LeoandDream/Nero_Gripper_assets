# Issue #25 — 唯一 runtime-oriented semantic partition 候选

**最终状态：`RUNTIME_ENTITY_PARTITION_BLOCKED`。** 已实际生成新几何、映射与误差证据，尚不能作为活动碰撞资产。对应[执行任务](https://github.com/LeoandDream/Nero_Gripper_assets/issues/25#issuecomment-6266903359)。生成前的预算已单独提交为 `7905a9542ee3fe7aae6ce5386a14b017688463e6`，本轮没有事后改预算或尝试第二版。

两个现有 finger-local frame 各使用同一 **753 体**模板，双指1506体；每指减少 **500体，39.90%**。接触层22、背体482、安装根部249。全部是闭合凸多面体，未生成MJCF。

| 语义族 | 每指候选数量 |
|---|---:|
| distal_tip_contact | 7 |
| inner_main_contact_plate | 14 |
| inner_ramp_contact | 1 |
| inner_transition_contact | 5 |
| sidewall_negative_x | 35 |
| sidewall_positive_x | 47 |
| outer_chassis | 395 |
| mount_root | 249 |

704个reference实体进入204个合并组，549个保持单独归属；参考材料删除量为零。9个最窄方向小于1nm、32个小于100nm的reference碎片均被吸收到较大实体，具体ID见[evidence的microcells](nero-gripper-runtime-evidence.json)，逐体映射见[JSON](nero-gripper-runtime-partition.json) / [CSV](nero-gripper-runtime-partition.csv)。每体包含源区域/三角形、完整vertices/faces/halfspaces、reference IDs、closure/邻接和吸收原因。旧reference文件保持不变。

## 预注册预算与实测

[冻结协议](nero-gripper-runtime-preregistration.json)：普通表面最大外扩/内缩各 **50µm**；关键工作面及轮廓 **10µm**、法向 **0.05°**；最小实体内切球直径 **0.1mm**。源格式最大坐标差约97.2nm，预算分别取已有1mm物理穿透尺度的5%和1%，不来自抓取结果。孔/槽/背袋/主要缺口必须保持，50µm侵蚀后的原空域不得填充。

| 指标 | 本次结果及适用范围 |
|---|---|
| 最大材料外扩 | 有限点集观测 **44.13744µm**；连续solid证书全域上界 **50µm**，因此精确最大值仅被夹在此区间内 |
| 最大材料内缩 | **0**：每个reference凸体完整包含于指定新hull；独立半空间包含残差≤6.5e-11mm。不是反向表面距离为零 |
| 接触层连续solid外扩上界 | **8.6603µm**，固定5µm cube Minkowski并集覆盖；此证书比10µm预算更严格，失败不能反推10µm预算不可行 |
| 安装根部双向source-part表面界 | **8.29878µm**，373个source seed的完整上包络/独立LP检查 |
| 新→旧外露网格距离 | 有限样本最大 **44.13744µm** |
| 旧→新外露网格距离 | 有限样本最大 **2.00000013mm**，发生于被合并填掉的装配微缝界面；不是毫米级材料膨胀 |
| 独立反向表面FAIL证据 | 旧E0362 / source689外露点周围±0.06mm完整cube被R0017∪R0193覆盖，故距新外露面 **至少60µm > 50µm**；不依赖外露网格提取 |
| runtime最小尺度 | 内切球直径 **1.844796µm**；最小facet-normal宽度 **1.845644µm** |

真正工作面单独按STL三角形核对：主面557–577、tip545–548、ramp549–556、shoulder578–581。最大连续支撑平面误差分别为 **0 / <0.00001 / 0.09355 / 0.005876nm**，最大法向差 **8.54e-7°**。这些是工作平面证书，不代替整个语义族的背面、边缘或双向union外露面验收；工作面到提取网格的有限采样结果另见[errors](nero-gripper-runtime-errors.json)。

## 保留特征与未通过项

保留主夹持面主要轮廓、中央根缺口和左侧非对称缺口、tip前缘、ramp和两段shoulder；保留大背袋及底/侧壁/根隔墙、背体四纵孔与一侧孔、四安装盲凹座及孔口、中央贯孔、根槽和0.15mm侧通道。接触层两缺口用完整凸空域检查；背体孔/背袋有连续线段见证及全域solid误差证书；安装根部131个见证、四盲孔深度及y=-70mm截面守恒。**不声称所有微缝保持**：装配界面的微缝确有被填掉，与上面的反向表面FAIL对应。

阻塞有三类，未回写任何旧结果：

1. **129体未满足0.1mm最小内切球直径**：接触层4、背体50、根部75；主要是薄肩、窄条、孔唇等，未删掉材料或放宽标准。
2. **装配微缝吸收与当前双向表面预算不兼容**。独立60µm下界由连续BSP和长双精度半空间对检查交叉证实；它不证明工作面或抓取失败。
3. **union外露面尚未认证**：逐面external/closure面积分区最大残差0.0384854mm²；当前输出是可复核的provisional polygon ownership。239666/81060个点的双向距离仅为该提取网格上的测量，未达到预注册5µm/1µm覆盖分辨率，不能称真实union的精确Hausdorff上界。几何JSON另保留R0299的1个未用于三角边界的辅助点；其实际闭合边界Euler=2。

## 重建与交接

几何代码实际使用Python **3.11.9**、NumPy **1.26.4**、SciPy **1.15.3**；绘图使用Python3.11.16、Matplotlib **3.10.3**，与运行模型无关。仅依赖本仓库已提交STL、reference JSON、functional regions、冻结协议及[373条mount seed](nero-gripper-runtime-mount-seeds.json)，不读取旧实验目录。以下命令重建同一固定规则的离线候选，不运行物理：

```bash
python scripts/runtime_contact_partition.py
python scripts/runtime_backing_partition.py
python scripts/runtime_backing_partition.py --verify-existing
python scripts/runtime_backing_partition.py --repair-triangulation
python scripts/runtime_backing_partition.py --verify-existing
python scripts/runtime_mount_partition.py
python scripts/assemble_runtime_partition.py --parts debug/issue25/runtime-partition/contact/partition.json debug/issue25/runtime-partition/backing/backing-runtime-entities.json debug/issue25/runtime-partition/mount/partition.json
python scripts/analyze_runtime_partition.py
python scripts/assemble_runtime_partition.py --finalize
python scripts/draw_runtime_partition.py
```

两处退化三角化只修复了同顶点凸包的faces表示，未改变分组或顶点，修复前FAIL及前后SHA保留在evidence。独立总装核验753个闭合、凸、定向边界，1253个reference各唯一归属一次、2086源片全部映射；分析工具另以相邻/重叠/重合盒子的解析表面积与已知三角形最近距离检查。详见[evidence](nero-gripper-runtime-evidence.json)、[误差报告](nero-gripper-runtime-errors.json)、[独立埋面反例](nero-gripper-runtime-buried-interface.json)和[对比图](nero-gripper-runtime-partition.png)。

**唯一下一步：交独立审查，确认129个残余薄体的机械语义和微缝表面口径，再批准局部重分区方案。** 当前不派生活动碰撞资产，不实施另一版。正式模型、源网格、控制/接触配置、main和用户README均未改；未导入MuJoCo、查询碰撞、运行mj_step或grasp。
