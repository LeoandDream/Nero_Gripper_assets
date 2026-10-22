# Issue37 Release Candidate 整改与验收

本轮仅收敛发布交付树，当前工作树及无Git干净复制预检已通过；GitHub固定源码ZIP待最终收据补齐。正式发布仍为 **RELEASE_BLOCKED / BLOCKED_LICENSE_OWNER_DECISION**，不选择原创代码许可证。

## 来源与实际改动

从`main@7f01deedc434d1d09cf98a043fcd2c53894e835e`创建独立RC分支，按文件取自联合MVP `77bd26a58df4eda99f611de7be1013d714ddaa7f`和Standalone `0d944a2e384c838ea1d72c8655d5efb15e5897d0`，不导入两支研究历史。事先冻结的选择计划保留在服务器执行目录，最终逐文件来源/用途/SHA见根目录[矩阵](../RC_FILE_SELECTION.json)。main全部140个已跟踪路径保留。

最终树530文件：139项源自main、368项源自77bd26a、14项源自0d944a2、9项新增发布工程。`requirements.txt`采用77bd26a已有依赖；旧main实际139条SHA清单改为529条，覆盖全树除自身。无缺项、重复或循环自指。346个共享MVP mesh对应整机/独立组件每指346、共692个collision geom；没有复制或重新生成物理几何。

两处控制脚本仅改可选Git版本记录，源码ZIP记录`head=null`并显式列出原资产来源；Standalone manifest仅更新该脚本摘要。所有控制函数、模块常量、定时、插值、力度及验收逻辑的AST与冻结源完全相同。控制器、R1配置及472项核心机械/物理源文件保持原SHA。组件manifest移除脚本摘要单项后与原完整JSON相等。

## 实际零积分预检

锁定Ubuntu/Python3.11.9、MuJoCo3.2.3、NumPy1.26.4。工作树与无Git复制目录均退出0：529条整树SHA通过；871个XML引用、433个唯一资源均存在且位于交付树内；10项静态/接口测试通过。三个模型编译和`mj_forward`均time=0、warning=0；`mj_step/mj_step1/mj_step2`均被阻断，实际积分调用及尝试为0。

| 入口 | nq/nv/nu/neq | geom/mesh | SHA256 |
|---|---|---|---|
| 静态整机 | 9/9/0/1 | 74/74 | e5585f29fef505f48b90bbf2c7b1b035ee7eac68147d60147399db5993263b4e |
| 联合MVP | 16/15/9/1 | 764/416 | db18a44f72ac62d6cd06bc4f9d4a7ef7ea38d36c43b5813064eb51b2f3ca8408 |
| Standalone | 2/2/2/1 | 699/353 | 2632f9b7c1d3867966e4bc08f2f4cd4b1146a4a2aef75ba3d01b1462479dd0a1 |

独立只读Reviewer核对了全部来源和目标blob、472项冻结SHA、控制AST及资源闭包，没有运行引擎。其指出的外部收据绑定与历史Run源码版本说明已修正。初次跨Python版本AST指纹预检在引擎调用前失败，已固定Python3.11重新生成并验证；没有改动模型或任何物理参数。

## 保留的结论与限制

[VALIDATION](../docs/VALIDATION.md)区分Legacy抓取FAIL、原固定R1 PASS和原Standalone空载PASS；原报告、日志、NPZ/JSON保持原字节。本轮没有重跑上述物理实验。研究几何BLOCKED、长保持/实机UNKNOWN以及已知接触限制不变。

完整R1原始Run约102MB仍保留在服务器原目录，公共紧凑记录不能替代完整原始contact复核。历史verifier必须使用当时冻结源码和seal，RC版本元数据适配不冒充旧源码。独立组件仍依赖共享meshes，XML不是单文件分发资产。没有依赖私有官方派生包、OpenPI或DH116。

复现仅执行：

```bash
sha256sum -c SHA256SUMS
python3.11 -B scripts/rc_checks.py --report /tmp/nero-rc-check-new.json
```

执行环境需安装根requirements；报告输出必须在交付树外且不存在。内部预检与独立审查位置：`/data_robot/projects/.experiments/nero-gripper-rc-issue37-20261023/`。原17个Draft PR、main、原始Run保留；未创建Tag/Release，未合并PR。正式许可由所有者决定，完成后仅交付Draft PR等待审查。
