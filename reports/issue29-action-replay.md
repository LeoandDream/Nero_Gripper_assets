# Issue #29 — 20Hz 示教位置动作的真实物理回放

**ACTION_RUNTIME：PASS；REPLAY_GRASP：PASS。** 仅执行 1 个新物理 Episode，34,150 个 1ms 步；后续均为零积分评估。它是高层开环 ReplayPolicy、底层反馈控制验收，不是 π0.5 推理或视觉闭环能力证明。

## 入口与冻结输入

- 入口：`scripts/replay_mvp_actions.py`；针对性测试：`scripts/test_replay_mvp_actions.py`。
- 物理输入只来自 `/data_robot/projects/.deliveries/nero-selfbuilt-mvp-77bd26a/`。直接加载原 XML、Controller、Recorder、reset、evaluate_grasp；没有派生模型、路径重写或物理参数修改。
- 动作来自 Issue #27 实际 Parquet 的 **`actions`** 列，682×8 float32，与源 `requested[50+50*i]` 逐项核对。9D `ctrl` 只作诊断，从未作为位置目标。
- 只读数据目录：`/data_robot/projects/.experiments/nero-mvp-lerobot-issue27-20261020/lerobot/local/nero_mvp_r1/`。来源版本 `7f4af8ee730c0ee7913427c12bc2f37dbe2093b7`，物理基线 `77bd26a58df4eda99f611de7be1013d714ddaa7f`。
- 原包、压缩包、原 Run、数据及来源文件共 467 个输入逐项核 SHA，运行及离线评估后均无变化。完整路径/摘要在新 Run 的 `frozen-plan.json`。

## 时序与接口

合法 reset 只在初始化写初态。前 50 步复用原 `requested[0:50]`，按整数步 0/10/20/30/40 更新；新旧前 50 步的 qpos、qvel、command、ctrl 完全一致。之后不再读取原专家插值或接触触发逻辑来控制机器人。

首个观察在真实物理 0.05s，第 i 条动作在整数步 `50+50*i` 之前传给 `Controller.set_target`，保持 50 步。原 `advance_command` 每 10 步调用，原 `apply` 与 `mj_step` 每步调用；原 Recorder 保留积分后的 `mj_forward` 和接触记录。没有二次插值或动态 qpos 写入。

- 高层目标调用 682 次，另有启动前缀 5 次；100Hz 更新 3,415 次；反馈/积分各 34,150 次。
- chunk 逻辑 shape 为 `(10,8)`，末块从 680 开始只消费 2 个有效动作。padding 消费 0 次；34.15s 后停止，不追加保持。
- `position_action` 拒绝维度、非有限值及真实越界。仅将 162 个 `float32(0.1)` 的端点表示误差还原到精确 0.1m，误差限为端点半 ULP；原值、还原值和误差界逐项记录，原 Controller 限位不变。
- 当前观察为真实积分得到的 9D 实际位置、同 Issue27 外部相机 256×256 RGB 和固定 prompt。保存 683 帧（0.05–34.15s，含无动作的末态）；所有 state 均与新轨迹对应步逐位相等。ReplayPolicy 按索引取动作，不依据观察决策。

## 原验收结果

| 指标 | 新积分结果 | 原标准 |
|---|---:|---:|
| 最低 hold 抬升 | 96.152778mm | ≥80mm |
| hold 时长 | 2.000s | ≥2s |
| hold 最大漂移 | 0.574253mm | ≤10mm |
| 双指支持比例 | 100% | ≥95% |
| hold 非指支撑 | 0 个样本 | 不允许 |
| 全程机器人—cube 最大穿透 | 0.526375mm | ≤1mm |
| 最大耦合残差 | 0.060422mm | ≤0.2mm |
| 最大 arm / finger 速度 | 0.203847rad/s / 0.028937m/s | ≤1rad/s / ≤0.1m/s |

原阶段末稳态跟踪检查全部通过，无数值告警、外力、时间重置或执行期 qpos 写入。**cube—table 穿透单列：最大 4.902505mm，发生于释放落桌；未将它混入原机器人—cube 1mm 判据，也不宣称该接触限制已解决。**

### 首版报告的评估适配错误及只读修正

`run_001` 首版原始报告仍保留 `control_health_status=FAIL`，其六项抓取检查已经全 PASS。唯一失败项是新 runner 错把源轨迹成功接触时刻 **20.483s** 当成 close 截止时间。

冻结 `execute_grasp` 实际允许从 8.372s 起最多关闭 **20s**，而非必须在 20.483s 达到双侧 0.2N。新轨迹在 **20.501s** 达阈值，延后 18ms，早于实际 lift 的 21.483s。只读修正使用原超时与“先接触、后抬升”的顺序约束；20.483s 仅保留为源诊断窗口边界，没有修改原阈值。`evaluation_003` 中原控制健康与抓取均 PASS；六项物理抓取指标未变。

首版执行源码提交为 `ba7d5f3f7601fff6a58332413a0120c6ccd53323`，完整脚本封存在 Run 内；交付脚本只修正评估及报告边界，所有产生物理积分的函数 AST 与实际执行版本一致。修正后**没有第二次物理回放**。只读 reviewer 确认本次 PASS，并指出的“抬升后才接触不应算 close 成功”和“无接触应正常输出 FAIL”边界均已修正，针对性测试通过。

## 新证据与可复现命令

- 实验根目录：`/data_robot/projects/.experiments/nero-mvp-action-replay-issue29/`。
- 实际 Run：`run_001/`，含每步 `trajectory.npz`、`contacts.jsonl`、683 帧 RGB、`observations.npz`、动作消费表、预先冻结方案及 SHA、执行源码、raw manifest。运行命令/退出码在根目录 `run_001-{command,execution}.json`，日志 `run_001.log`；进程退出码 0 只表示执行完成。
- 原始 `run_001/result.json`、`grasp-result.json` 的 FAIL 不覆盖；修正判定见 `evaluation_003/{control-health,grasp-result,runtime-result,timing-analysis}.json`。
- 零积分检查 `check_001` 在积分前通过；评估修正后检查保存在 `check_002`。6 项针对性测试覆盖非法位置、端点误差、chunk 尾部、整数调度、原关闭超时和接触先于 lift 的边界；不新增 CI 或质量门槛。
- 便于审查的索引：[issue29-evidence.json](issue29-evidence.json)；新积分视频：[issue29-replay.mp4](issue29-replay.mp4)。视频 SHA256：`fd934aca3be385f85ab8303d113c43050282f308e55f8b466fd7f917544393f4`，122,285 bytes。
- 环境：Python3.11.9、MuJoCo3.2.3、NumPy1.26.4、SciPy1.15.3、PyArrow20.0.0、Pillow11.2.1；完整版本见实验根目录 `environment.txt`。

仓库根目录使用该环境：

```bash
PY=/data_robot/projects/.experiments/nero-mvp-action-replay-issue29/env311/bin/python
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 "$PY" -B scripts/test_replay_mvp_actions.py
# 零积分：输出目录必须不存在。
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 "$PY" -B scripts/replay_mvp_actions.py check --output /tmp/issue29-check-new
# 只读重新评估现有新 Run；不积分，不覆盖原始 Run。
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 "$PY" -B scripts/replay_mvp_actions.py evaluate-only \
  --run-directory /data_robot/projects/.experiments/nero-mvp-action-replay-issue29/run_001 \
  --output /tmp/issue29-evaluation-new
```

本轮实际执行过的物理命令如下，仅供复核；**本轮额度已使用 1 次，未再次执行**：

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 "$PY" -B scripts/replay_mvp_actions.py run \
  --output /data_robot/projects/.experiments/nero-mvp-action-replay-issue29/run_001 \
  --checked /data_robot/projects/.experiments/nero-mvp-action-replay-issue29/check_001
```

实际消费目标从第 61 步（0.061s）起出现 float32 表示误差之外的预期采样差异；手指位置差异超过 1µm 的首个样本为第 62 步。不是逐点无损重放，却在本次固定任务通过原判据。一次确定性成功不能证明重复性、长保持、扰动泛化或学习策略能力；这些仍未验证。

下一步唯一建议：由 ChatGPT 独立核对动作消费时间表、close 评估修正与新视频；本轮不训练、不接权重、不调参、不继续资产研究。保持 Draft，不合并，停止等待审查。
