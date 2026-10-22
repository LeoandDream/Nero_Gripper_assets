# Issue37 Release Candidate 整改与验收

本轮仅收敛发布交付树；工作树、无Git干净复制及实际GitHub固定源码ZIP均通过零积分验收，状态 **RC_TECHNICAL_READY_LICENSE_PENDING**。正式发布仍为 **RELEASE_BLOCKED / BLOCKED_LICENSE_OWNER_DECISION**，不选择原创代码许可证。

## 来源与实际改动

从`main@7f01deedc434d1d09cf98a043fcd2c53894e835e`创建独立RC分支，按文件取自联合MVP `77bd26a58df4eda99f611de7be1013d714ddaa7f`和Standalone `0d944a2e384c838ea1d72c8655d5efb15e5897d0`，不导入两支研究历史。事先冻结的选择计划保留在服务器执行目录，最终逐文件来源/用途/SHA见根目录[矩阵](../RC_FILE_SELECTION.json)。main全部140个已跟踪路径保留。

最终树530文件：139项源自main、368项源自77bd26a、14项源自0d944a2、9项新增发布工程。`requirements.txt`采用77bd26a已有依赖；旧main实际139条SHA清单改为529条，覆盖全树除自身。无缺项、重复或循环自指。346个共享MVP mesh对应整机/独立组件每指346、共692个collision geom；未为组件重复复制网格，也未重新生成或修改几何。

两处控制脚本仅改可选Git版本记录，源码ZIP记录`head=null`并显式列出原资产来源；Standalone manifest仅更新该脚本摘要。所有控制函数、模块常量、定时、插值、力度及验收逻辑的AST与冻结源完全相同。控制器、R1配置及472项核心机械/物理源文件保持原SHA。组件manifest移除脚本摘要单项后与原完整JSON相等。

## 实际零积分验收

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

## GitHub源码ZIP实测与固定入口

实际下载固定RC快照`7ac058568c7f7bbf750b51579c53a2fbe33f9a20`，Git tree `98fa82e947572a74f5cdd4c4942bdbae473361e0`。从GitHub codeload下载，而非本地git archive替代：

- ZIP：33,917,265字节，SHA256 `6fb051820599c88faa0019a96872631d5ade3f4278786ccd9db03ea7c124a98c`。
- 解压后无.git，530个文件逐一与该commit Git blob一致。
- `sha256sum -c`为529/529 PASS；静态检查退出0，10项通过，3模型加载/forward，积分0。
- 该快照整体清单SHA256 `958747ee6a411af535c3b4509a18c3d26249bd618adb7f74aa8be6c96613462b`，来源矩阵SHA256 `d3fe1ff1e6ecd8c36f71b8af9b5885de86c9e837c95883ad63c44054165dfefa`。
- 原始静态验收JSON SHA256 `29ab72e6ef61a0b3f084a4dcd76bbbcb4ac9bacd5af6892e022efa72a8a25b08`；下载命令、环境、完整测试名、解压路径及退出码已保存在[机读记录](issue37-release-candidate.json)的`downloaded_github_snapshot`。

当前收尾提交仅更新README、两份Issue37报告、来源矩阵和整体校验；所有运行源码、XML、mesh、config与该已验证快照逐字节相同。README推荐该固定运行SHA，原快照内当时的预检标记由本报告后续收据覆盖。最终收尾提交的GitHub ZIP还会独立执行同一零积分检查，其commit/tree/ZIP/清单/矩阵SHA和退出码写入Issue37，避免向被校验报告写入自己的摘要产生循环。

外部验证产物：`github_snapshot_a.zip`、`github_snapshot_a-checks.json`、`github_snapshot_a-receipt.json`、`github_snapshot_a.log`，位于前述服务器执行目录。重现下载可使用README固定链接；解压后执行上面的两条零积分命令。仅用已安装的锁定环境，没有重新安装依赖或声称验证新平台兼容性。
