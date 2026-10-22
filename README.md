# NERO + Gripper — Release Candidate

本分支是 **Release Candidate 发布工程候选**，不是已合并main或已发布Release。研究级几何认证仍为BLOCKED，长保持及实机一致性为UNKNOWN。新增工程代码的发行许可尚未由所有者决定：**RELEASE_BLOCKED / BLOCKED_LICENSE_OWNER_DECISION**。

本候选从 `main@7f01deedc434d1d09cf98a043fcd2c53894e835e` 逐文件引入已验收 `77bd26a` 的联合MVP和 `0d944a2` 的独立Gripper；不是合并17个研究Draft PR。完整来源及适配摘要见 [RC_FILE_SELECTION.json](RC_FILE_SELECTION.json)，整改范围见 [RC说明](docs/RC_RELEASE.md)。

## 获取与安装

推荐的已验收RC运行快照：`7ac058568c7f7bbf750b51579c53a2fbe33f9a20`（下方固定SHA，非#26旧研究分支）。当前RC后续提交仅补充ZIP验收收据和获取说明，运行文件与该快照完全相同；最终文档树SHA及其下载验收见Issue #37。

```bash
RC_COMMIT=7ac058568c7f7bbf750b51579c53a2fbe33f9a20
git init Nero_Gripper_assets-rc
cd Nero_Gripper_assets-rc
git remote add origin https://github.com/LeoandDream/Nero_Gripper_assets.git
git fetch --depth 1 origin "$RC_COMMIT"
git checkout --detach FETCH_HEAD
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
sha256sum -c SHA256SUMS
```

无Git使用同一固定快照：[GitHub源码ZIP](https://github.com/LeoandDream/Nero_Gripper_assets/archive/7ac058568c7f7bbf750b51579c53a2fbe33f9a20.zip)。下载ZIP SHA256：`6fb051820599c88faa0019a96872631d5ade3f4278786ccd9db03ea7c124a98c`，本次下载33,917,265字节。GitHub可能改变自动压缩表示，最终文件真值以固定commit和解压后的SHA256SUMS为准。A快照中的Issue37报告标记为当时预检状态；ZIP已通过的后续收据在当前分支报告和Issue37中，不代表新增实验。

main目前仍是旧静态资产主线；普通 `git clone` 后不切换到RC并不能获得本页新增入口。源码ZIP解压后也可用，不要求 `.git`、ROS、旧worktree或网络获取模型；安装Python依赖可能需要包索引。共享 `meshes/` 是运行闭包的一部分，**XML单文件不是独立分发包**。

## 三类入口

| 入口 | 用途与计数 | 已验证范围 |
|---|---|---|
| [mjcf/nero_gripper.xml](mjcf/nero_gripper.xml) | 原静态参考整机；nq=nv=9，nu=0，neq=1 | 静态机械来源验证；nu=0合理，不是受控抓取入口 |
| [mjcf/simulation_mvp.xml](mjcf/simulation_mvp.xml) | 联合MVP与固定R1桌面/cube场景；nq16/nv15/nu9；764geom/416mesh | 原34,150步固定R1工程PASS；不是通用抓取成功率 |
| [assets/components/gripper/gripper.xml](assets/components/gripper/gripper.xml) | 固定安装座独立夹爪；nq=nv=nu=2，neq1；699geom/353mesh，含692个手指collision geom | 原一次9500步空载PASS；不追认独立抓物 |

[components / assemblies / scenes索引](assets/README.md) · [接口与安装](docs/RC_RELEASE.md#接口与安装) · [分模型验收状态](docs/VALIDATION.md)。未来NERO-only、DH116和双臂仍为PLANNED，未实现占位资产。

从仓库根做零积分加载：

```bash
.venv/bin/python - <<'PYLOAD'
import mujoco
for path in ['mjcf/nero_gripper.xml', 'mjcf/simulation_mvp.xml', 'assets/components/gripper/gripper.xml']:
    model = mujoco.MjModel.from_xml_path(path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print(path, model.nq, model.nv, model.nu, model.neq, data.time)
PYLOAD
.venv/bin/python -B scripts/rc_checks.py --report /tmp/nero-rc-check-new.json
```

校验报告须写在交付树外且文件不存在。该检查阻止`mj_step/mj_step1/mj_step2`，校验整树、全部XML资源、manifest、控制源码等价及静态测试。**不要用 `scripts/test.py` 无参或动态测试代替本轮检查。**

使用控制入口的命令如下；它们会启动新的物理积分，**不是RC静态验收命令，本轮没有执行**：

```bash
.venv/bin/python examples/gripper_control.py --output /tmp/gripper-empty-new
.venv/bin/python experiments/dynamics/simulation_mvp.py smoke --output /tmp/r1-new
```

独立夹爪接收1D绝对开口命令 `w∈[0,.1]m → (+w/2,-w/2)`；整机 `Controller.set_target` 接收8D `[joint1..joint7, w]`，前7维rad，末维m。内部9D ctrl是限力执行器努力，不是位置action。完整契约、固定安装座6mm偏移及静态查看方法见 [组件说明](assets/components/gripper/README.md)。

## 证据与发布限制

[RC整改报告](reports/issue37-release-candidate.md) · [RC机读记录](reports/issue37-release-candidate.json) · [原固定R1报告](reports/nero-gripper-simulation-mvp.md) · [原独立夹爪报告](reports/issue35-standalone-gripper.md)。原报告的历史分支/服务器路径是来源记录，不是RC下载入口。

约102MB的R1完整原始Run未公开，内部路径及SHA索引保留。公开的紧凑NPZ/JSON不能替代完整contact原始记录的独立复算，第三方复核限制详见 [VALIDATION](docs/VALIDATION.md)。历史Legacy抓取FAIL未改写。

[第三方来源说明](licenses/THIRD_PARTY_NOTICES.md) · [所有者待决许可清单](licenses/LICENSE_DECISION_REQUIRED.md)。厂商MIT文本不自动覆盖本仓库原创代码。当前不创建Tag/Release、不合并PR，也不声明整个仓库已按MIT/BSD/Apache发行。
