# Issue #25：v2.1规则冻结（验收前单独提交）

本文件与[结构化规则](nero-gripper-v21-fidelity-protocol.json)只冻结用户评论6271873067授权的两项修正，不包含重新验收结果，也不追认旧候选PASS。

继承`8a5518b37f15678893ca6f540f5ef2edea199ab8`原protocol blob（SHA256 `4ffb096f2519fed41d8d45c9c0b4ed3ec4dc781f959d4f0420905ce8b5df999b`）。固定候选仍为`6eba4f3`的346体，vertices/faces SHA `79bef01f443035c6db2213c0e73531c079cdb97762361e682d7c9d052edac84e`。

唯一protected-void改动：

> 保留mount part中央孔本身；完整装配connectivity必须匹配冻结source assembly。若backing原本阻挡，不要求runtime人为挖通。

其余protected void、A/B/C源域与预算全部继承，不改变A的10µm/0.05°、B的50µm、C材料集合50µm及每体0.1mm最小内切球直径。

数值表示等价规则：runtime几何真值以冻结vertices/faces为主，halfspaces只是分析表示。在已有 **`1e−7mm`** 数值支持尺度内，closure gap/overlap若不形成任何更大的有效solid/void区域，视为同一boundary representation。必须有局部支持/包含或区间证据约束其尺度；不能只看小残差就豁免，也不能用该规则掩盖较大边界偏差、protected void堵塞或真实A/B超限。未能约束的混合区域保持UNKNOWN。

本轮将在此规则提交之后，重新检查66 unattributed、268 missing、21 occlusion、14未决及triangle2940，并完整报告scale、A/B双向surface/contour/normal、C材料集合/源域、所有protected void与旧168点。新分析可以重建验证用半空间/外露面表示，但不改变候选顶点、面或生成新partition；分析中间产物不能称为新的runtime候选。
