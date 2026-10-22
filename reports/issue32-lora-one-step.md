# Issue #32 — 官方 π0.5 LoRA 单步实测

**PI05_NERO_LORA_ONE_STEP_PASS。真实本地预训练模型、真实 NERO batch，恰好一次官方 JAX/Optax 更新，step 0→1。** 这只验证轻量微调入口；不证明训练收敛、策略有效或全参数训练可行。原 `c24a546` / Draft PR #33 的全参数 AdamW+EMA 显存阻断报告原样保留。

| 实际验收 | 结果 |
|---|---|
| checkpoint 基座恢复 | PASS；51 个原参数条目 key/global shape 匹配，恢复初值与官方转换后的 checkpoint 值逐项字节一致；只允许新增20个官方 LoRA 条目 |
| NERO loader | PASS；673个合法起点，实际消费0/1，真实归一化、完整10步、无时间 padding |
| forward/backward/optimizer | PASS；官方 `train_step` 实际调用1次，step 0→1，Adam状态变化 |
| loss / grad_norm / param_norm | **0.5817274451255798 / 3.3095920085906982 / 1833.281005859375**，全部有限，梯度非零 |
| LoRA 更新 | 20/20路径有非零变化，共48,434,710个元素；最大绝对变化 `2.60770320892334e-08` |
| 冻结权重 | 20个参数条目、2,936,464,384个元素，更新前后SHA全部一致 |
| 参数和优化器状态 | 全部有限；51条可训练路径均有更新，共389,563,229个元素，最大变化 `2.9802322387695312e-08` |

## 固定输入与最小改动

独立工作树 `/data_robot/projects/.worktrees/nero-gripper-lora-step`，分支 `feature/nero-lora-one-step`，基于 `c24a546`；实际执行代码提交 `3061906ce2dd04cc83e5986979fac63780aac881`。OpenPI锁定 `215abfb217dbac7d5f1273282331b9b1866c0479`，Python3.11.9 / JAX0.5.3 / Flax0.10.2 / Optax0.2.4，完整版本见机读报告。

唯一配置为 Issue 指定的 `Pi0Config(pi05=True, action_horizon=10, action_dim=32, paligemma_variant="gemma_2b_lora", action_expert_variant="gemma_300m_lora")`，保留 `discrete_state_input=True`；使用官方 `model.get_freeze_filter()`、`ema_decay=None`、seed42、FSDP2、batch2、默认AdamW与CosineDecaySchedule。按官方配置，warmup起始学习率为 `2.5e-5/(1000+1) ≈ 2.4975e-8`，所以更新幅度小；未为放大变化改学习率。

**官方filter冻结LLM/动作专家的非LoRA基座，并不冻结整个视觉主干。** 466,957,072个float32可训练元素包括49,987,584个LoRA元素及416,969,488个视觉/辅助头元素；后者31条路径本次也有更新。不能把这次配置描述为“只有LoRA可训练”。冻结LLM基座沿官方初始化转bfloat16。

复用 `bec08f9` 数据入口、全部682帧原始数据及673起点训练视图。norm stats SHA仍为 `59b12d0aa10aa116018cd8d8c0d1d0615fdb85e7a45ac3bc61d647c33a31ff26`。本次 batch：state `(2,32)`、actions `(2,10,32)`、RGB `(2,224,224,3)`，浮点输入为float32，全部有限。真实动作仍为前8维绝对位置，后24维仅模型特征padding；9D状态后23维为0。原官方32D flow-matching loss不改、不新增mask。

本地权重来源：

```text
/data_robot/projects/official_pi05_libero/cache/openpi/openpi-assets/checkpoints/pi05_libero/params
```

15个文件共12,439,083,567 bytes；逐文件SHA在机读报告。旧SHA清单摘要 `890eb06c040e8765bb530656f4425a38b9dab63db8bcac244c19e1a9af3ad7ea`。CPU计划和实际恢复前均校验文件；真实恢复委托官方 `CheckpointWeightLoader`。没有下载、保存或上传更新后权重。

项目改动仅有：新增一次性runner及5项CPU适配测试；既有 `create_training_loader` 增加可选 `sharding` 透传，673起点过滤、Normalize和动作标签不变。直接复用官方 `init_train_state`、`train_step`；不调用 `train.main`。

## 显存：预算与实测分别报告

CPU抽象状态：参数7,740,757,056 bytes，Adam状态3,735,656,584 bytes，EMA=None，总持久状态约10.69GiB。按照实际官方FSDP分片（含小数组复制）计算：

| 项目 | GPU2 / logical cuda:0 | GPU3 / logical cuda:1 |
|---|---:|---:|
| 运行前可用 bytes | 25,231,884,288 | 25,228,738,560 |
| 预计常驻训练态 bytes | 5,763,861,708 | 5,763,861,708 |
| 扣除常驻后的余量 bytes | 19,468,022,580 | 19,464,876,852 |
| 保守初始化预算，含2GiB预留 | 15,452,152,076 | 15,452,152,076 |
| 编译单步缓冲区估计，另留2GiB | 9,375,430,987 | 9,375,430,987 |
| `nvidia-smi` 采样峰值 | **17,331MiB（16.925GiB）** | **17,076MiB（16.676GiB）** |
| JAX allocator active peak bytes | 13,425,080,832 | 13,312,358,912 |
| JAX allocator pool peak bytes | 17,454,596,096 | 17,186,160,640 |

编译估计为 arguments + outputs + temporary − donated aliases，不是整个进程/初始化/allocator峰值。`nvidia-smi`每轮请求间隔0.25s，共1975轮、跨度733.24s，采样不代表无遗漏瞬时峰值；JAX allocator峰值另列。两卡全程未OOM。退出后的只读快照显示GPU2/3仅剩19/22MiB、已无本次计算进程，GPU0/1两个初始任务PID仍存在；本次未终止或复用它们。GPU2/3 UUID、初始/退出快照、逐阶段采样及原始JAX统计均保留。

## 适配失败记录与验证

这些是同一候选的接口修正，没有扫描配置或重复optimizer step：

1. `plan_001`：CPU阶段发现锁定Flax `flat_state()` 返回dict，修正为 `.items()`；GPU和optimizer调用均为0。
2. `run_001`：显式JAX `.lower` 的内部 `ArgInfo` 触发类型检查，发生在真实权重恢复和更新之前，optimizer调用0。改用上游已有 `array_typing.disable_typechecking()`，仅作用于lower/compile；成功退出后恢复检查，不改变loss/模型计算。上游该helper缺少异常路径finally；编译失败时本runner封存错误并退出，不继续训练。
3. 预编译和真实初始化分别创建的Optax闭包，Flax静态metadata身份不同。复用相同配置抽象prototype的 `tx`，检查model graph及完整pytree结构相等；不替换任何numeric optimizer state。

最终 `plan_003` 继承 `plan_002` 已算出的同一配置/参数预算，仅更新编译适配代码SHA；AST核对证明配置、计划和checkpoint映射函数不变。修订记录在实际恢复前冻结。runner源SHA为 `6a1ecb960d3fa457bda61851a7032ab1ae7827f4dc63223e3b36f0399249d0ac`。

5项CPU适配测试通过，包括真实安装Flax API、类型化state lowering及静态Optax身份。最终 `run_002` 退出码0，墙钟758.41s（包含编译、恢复、参数逐项核验，不是单步吞吐）；仅该run实际执行1次更新。日志中未能donate部分初始化buffers及rematerialization提示原样保留；它们没有被当作OOM或被隐藏。封存manifest和全部计划输入SHA在整理时再次核对通过。

## 证据与可复现命令

- [机读证据](issue32-lora-one-step.json)：冻结配置/输入SHA、51条基座与20条新增映射、逐路径参数差异及前后SHA、设备预算、显存、命令、退出码、早期失败索引及所有原始文件摘要。
- [实际运行日志](issue32-lora-one-step.log)：与原 `run_002.log` 字节一致。
- 原始记录：`/data_robot/projects/.experiments/nero-lora-step-issue32-20261023/`，保留 `plan_001/002/003`、`run_001/002`、完整失败traceback、commands/execution、gpu-samples及manifest；不覆盖旧run。
- 既有全参数报告仍为 [issue32-one-step.md](issue32-one-step.md)，没有被此次PASS追认为全参数训练可用。

以下是复现命令，**会产生一次新的更新；本轮完成后未再次运行**。新目录必须不存在，先CPU计划确认，GPU2/3仍无其它任务时才执行run：

```bash
cd /data_robot/projects/.worktrees/nero-gripper-lora-step
PY=/data_robot/projects/official_pi05_libero/server-venv/bin/python
export CUDA_VISIBLE_DEVICES=2,3 CUDA_DEVICE_ORDER=PCI_BUS_ID
export XLA_PYTHON_CLIENT_PREALLOCATE=false TF_FORCE_GPU_ALLOW_GROWTH=true
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 WANDB_MODE=disabled OPENBLAS_NUM_THREADS=1
export PYTHONDONTWRITEBYTECODE=1
export HF_LEROBOT_HOME=/data_robot/projects/.experiments/nero-mvp-lerobot-issue27-20261020/lerobot
export OPENPI_DATA_HOME=/data_robot/projects/official_pi05_libero/cache/openpi
JAX_PLATFORMS=cpu "$PY" -B experiments/dynamics/test_pi05_lora_step.py
JAX_PLATFORMS=cpu "$PY" -B experiments/dynamics/pi05_lora_step.py plan --output /tmp/nero-lora-plan-new
JAX_PLATFORMS=cuda "$PY" -B experiments/dynamics/pi05_lora_step.py run --plan /tmp/nero-lora-plan-new --output /tmp/nero-lora-run-new
```

没有正式训练循环、第二次更新、WandB、MuJoCo、权重保存或PR合并。当前阶段停止，等待ChatGPT独立审查；不自动扩展到多步训练。
