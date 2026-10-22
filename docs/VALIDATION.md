# 验证状态与复核边界

以下结论按模型和任务分开，不能互相替代。本轮Issue37只做文件和静态验证，新增物理积分为0。

| 模型/任务 | 冻结来源 | 状态与范围 |
|---|---|---|
| 原静态整机 `mjcf/nero_gripper.xml` | main/77bd26a相同字节 | PASS静态来源/运动学/惯性；9DOF、nu=0合理 |
| Legacy受控模型及旧手指代理抓取 | 原旧静态模型派生；[保存结果](../reports/validation.json) | **FAIL**，49,387步复验不通过；最大抬升0.138mm、穿透1.818mm，不能当成新MVP结果 |
| 联合Simulation MVP固定R1 | 77bd26a / PR26 | **PASS固定场景工程烟测**，34,150步，hold最低抬升96.146mm、漂移0.574mm、双指支持100%、机器人-cube最大穿透0.526mm |
| 独立Gripper | 0d944a2 / PR36 | **PASS静态+一次空载**，9500步；来源等价、开合/耦合/速度/限力通过。不表示独立夹持成功 |
| 严格研究几何认证 | 原v2.1及后续研究历史 | **BLOCKED**，局部A侧壁法向及certifier问题未被RC追认为通过 |
| 长保持、复杂manipulation、双臂、实机标定 | 无相应验收 | **UNKNOWN / UNVERIFIED** |

原始历史验收阈值保留：arm稳态≤.005rad、finger稳态≤.5mm、coupling≤.2mm、速度≤1rad/s/.1m/s。完整抓取还要求最低hold抬升80mm、2s hold、漂移≤10mm、双指支持≥95%、无其它支撑、机器人-物体穿透≤1mm；具体原协议见 [冻结R1配置](../experiments/dynamics/simulation_mvp.json)。RC未修改这些阈值，也没有新的物理实验。

## 本轮可重跑的零积分检查

```bash
sha256sum -c SHA256SUMS
python -B scripts/rc_checks.py --report /tmp/nero-rc-zero-step-new.json
```

必须使用MuJoCo3.2.3。检查包括三个模型编译/forward、共享文件及SHA闭包、源控制AST、6项Standalone静态、2项MVP静态来源、2项可选Git元数据测试；通过阻断函数禁止三个`mj_step` API。不要执行R1 smoke、空载入口、动态测试或会积分的viewer。此处静态通过不是重新执行原动态验收。

## 已公开证据与外部限制

- [原MVP报告](../reports/nero-gripper-simulation-mvp.md)、[来源/执行清单](../reports/nero-gripper-simulation-mvp-provenance.json)、[完整验收复核摘要](../reports/nero-gripper-simulation-mvp-run-verification.json)、[紧凑逐帧NPZ](../reports/nero-gripper-simulation-mvp-evidence.npz)、[部分contact原始记录](../reports/nero-gripper-simulation-mvp-contact-records.json.gz)均为77bd26a原字节。
- 原完整Run在 `/data_robot/projects/.worktrees/nero-gripper-runtime-partition/debug/issue25/simulation-mvp/smoke_001/`，约102MB；逐文件及raw-manifest SHA在上述provenance中保留。**这些路径不是外部下载地址，完整Run不在GitHub或源码ZIP中。** 未擅自上传大数据。
- 公开紧凑数据可检查保存指标及部分contact，但不能独立重做全部contact、旧helper和完整验收过程。需要完整Run及冻结执行源码才能重跑原零积分verifier；[原77bd26a verifier](https://github.com/LeoandDream/Nero_Gripper_assets/blob/77bd26a58df4eda99f611de7be1013d714ddaa7f/experiments/dynamics/verify_simulation_mvp.py)要求源封存精确一致。RC元数据适配不冒充原执行源码；原结论来自当时已保存证据。
- [独立组件报告](../reports/issue35-standalone-gripper.md)、[机读结果](../reports/issue35-standalone-gripper.json)、[完整9500步NPZ](../reports/issue35-gripper-trajectory.npz)、图和日志保持0d944a2原字节。原Run内部目录 `/data_robot/projects/.experiments/nero-gripper-standalone-issue35-20261023/empty_001/` 的其余seal文件由原报告SHA索引追溯。
- Legacy归档仍在 `56cfb6255825cc61f0fb46d3848c46c2b1f166c2:reports/dynamics-evidence.tar.xz`，SHA `fd12ba7206e79f1b0faaaa01391f86f0995d3fd66f32574249547b1cebab7d56`；可在完整历史中提取。未改写/删除研发分支、17个旧Draft PR或原始记录。

已知legacy MPR深探针法向异常、释放落桌穿透及局部工作面限制仍见原MVP报告。本轮未做新几何认证，不能从文件级RC验收推导高保真或新任务成功。

## 许可与发布

当前最多为 **RC_TECHNICAL_READY_LICENSE_PENDING**；正式发布仍是 **RELEASE_BLOCKED / BLOCKED_LICENSE_OWNER_DECISION**。新增工程代码发行许可必须由所有者决定，见 [待决清单](../licenses/LICENSE_DECISION_REQUIRED.md)。未授权合并、Tag或正式Release。
