# Issue #32 — 单次真实 optimizer step 的 Gate 0 阻断

**TRAIN_STEP_BLOCKED / BLOCKED_GPU_MEMORY。目标未达成：有效 optimizer step 为 0 次。** 已完成只读结构、真实 batch 和显存预检，按 Issue 的资源阻断规则停止。没有 forward/backward、真实训练状态初始化、OOM 尝试、权重保存或 WandB run。

| 验收项 | 结果 | 精确范围 |
|---|---|---|
| CHECKPOINT_MODEL_COMPATIBILITY | PASS | 本地 Orbax metadata 与官方目标模型51个参数条目的 key/global shape/dtype 一致；**未实际 restore 或 forward** |
| NERO_MODEL_BATCH_DTYPE | PASS | 已审查的673起点 loader，真实2个样本，原生JAX batch float32/NHWC；原norm stats及动作语义保留 |
| REAL_PI05_FORWARD_BACKWARD | FAIL（未运行） | 资源 Gate 未通过；不是已观察到 loss/gradient 数值失败 |
| ONE_OPTIMIZER_STEP | FAIL（未运行） | 实际 update 0次，不能标 PI05_NERO_ONE_STEP_READY |

## 冻结输入及实际检查

独立分支 `feature/nero-one-optimizer-step`，基于 `bec08f9dbb8433dc4d34819443f98b29e9603d76`。复用该提交的 `create_training_loader`，未修改其673起点逻辑、原始682帧数据或统计。统计SHA仍为 `59b12d0aa10aa116018cd8d8c0d1d0615fdb85e7a45ac3bc61d647c33a31ff26`。

使用锁定 OpenPI `215abfb217dbac7d5f1273282331b9b1866c0479`、现有 server-venv 和本地 checkpoint：

```text
/data_robot/projects/official_pi05_libero/cache/openpi/openpi-assets/checkpoints/pi05_libero/params
```

来源为服务器此前下载的官方 `gs://openpi-assets/checkpoints/pi05_libero`，没有新增下载或加载其 LIBERO norm stats。已读取本地 Orbax metadata，并对 params 目录全部文件计算 SHA256；逐文件清单在机读报告。清单文件本身 SHA256：

```text
890eb06c040e8765bb530656f4425a38b9dab63db8bcac244c19e1a9af3ad7ea
```

该清单位于实验目录 `checkpoint-sha256.json`，摘要不是某个单独大权重文件的摘要。结构兼容 PASS **只限定 metadata 核对，不冒充本轮完整权重恢复或真模型前向通过**。

模型配置为 `Pi0Config(pi05=True, action_horizon=10, action_dim=32)`；保留既有 NERO `discrete_state_input=True`，未套用 LIBERO 的 False。官方原始32维 flow-matching loss 不改：仅前8维对应7个绝对关节rad及夹爪总开口m，后24维是模型特征 padding，未新增 loss mask。

固定配置：seed42、batch2、拟使用空闲GPU2/3及FSDP2；全 `nnx.Param` 可训练（freeze_filter=`nnx.Nothing`），无LoRA。沿用官方默认 AdamW（b1=.9,b2=.95,eps=1e-8,weight_decay=1e-10,clip_norm=1）、CosineDecaySchedule（warmup1000,peak_lr2.5e-5,decay30000,end2.5e-6）、EMA=.99。没有为了通过资源检查改变优化器、冻结策略或模型。

## 显存阻断证据

GPU0/1由既有进程3669327/987212占用，各约21GB；没有终止、释放或复用它们的显存。GPU2/3无计算进程，实际快照合计空闲 **50,460,622,848 bytes（约46.995GiB）**。

直接调用官方 `init_train_state(..., resume=True)` 获得抽象训练状态，仅执行 CPU shape trace，**不是恢复训练、不生成实际随机参数、不创建真实优化器 buffers**。它给出3,353,433,872个全参数float32权重：

| 持久状态组成 | bytes |
|---|---:|
| 参数 | 13,413,735,488 |
| Adam状态 | 26,827,470,984 |
| EMA参数 | 13,413,735,488 |
| 合计 | **53,654,941,960（约49.970GiB）** |

仅该配置的持久状态总量已经超过两张空闲卡合计可用显存。这还是乐观总量下界，未计FSDP小数组复制、激活、attention、collective、编译临时buffers或额外梯度。未发起GPU实际初始化和OOM试错，也未改用CPU执行训练、SGD、LoRA、更多冻结或其它模型来获取成功结果。

## 真实 NERO batch 与 dtype 衔接

直接使用 `create_training_loader(..., batch_size=2, shuffle=False, num_batches=1, framework='jax')`，内部仍是原训练视图→Normalize（skip_norm_stats=False）→模型transforms→官方TorchDataLoader/DataLoaderImpl。673个合法起点保留，本次消费起点0/1，两个chunk均10个真实动作、无时间padding。

- Normalize之后的物理/模型特征样本 state/actions 为float64，符合已审查的原路径。
- 原生JAX loader边界已将state/actions变为float32；额外项目侧转换器对本次batch没有再次改变dtype，只做有限值/浮点类型保证。state `(2,32)`、actions `(2,10,32)`、RGB `(2,224,224,3)`，tokens `(2,200)` 为int32，mask为bool。
- 没有混入PyTorch的NCHW图像布局。图像、整数tokens、mask的值保持正确，所有输入有限；state尾23维及action尾24维全部为0。动作前8维与值域、完整叶子shape/dtype均保存，没有改写stats或动作标签。
- 上述是**CPU JAX上的输入路径验证**，不是GPU模型前向验收。

loss、grad_norm、param_norm、真实训练step起止、optimizer状态变化及参数更新均为 **N/A（未执行）**，没有编造0→1或非零参数更新。

## 交付、命令与下一步

- 最小预检脚本：`experiments/dynamics/pi05_one_step_preflight.py`。它明确只做 Gate0，不调用 `train_step` 或正式 `train.main`；资源阻断后没有扩展编写/执行未验证的训练runner。
- [机读证据](issue32-one-step-preflight.json)：实际GPU、权重路径/各文件SHA、抽象shape预算、真实batch、命令、退出码和全部结果。
- 原始证据：`/data_robot/projects/.experiments/nero-one-step-issue32-20261023/preflight_001/`；根目录含 `preflight_001.log`、命令和执行记录。预检退出码0表示成功形成阻断报告，不表示训练成功。
- 2项针对性测试通过；不依赖GPU或模型权重分配。原数据、stats、项目适配器和实际上游文件摘要运行前后保持一致。

复现只读预检（新输出目录；不是训练命令）：

```bash
PY=/data_robot/projects/official_pi05_libero/server-venv/bin/python
export JAX_PLATFORMS=cpu CUDA_VISIBLE_DEVICES=2,3 XLA_PYTHON_CLIENT_PREALLOCATE=false
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 WANDB_MODE=disabled OPENBLAS_NUM_THREADS=1
export HF_LEROBOT_HOME=/data_robot/projects/.experiments/nero-mvp-lerobot-issue27-20261020/lerobot
export OPENPI_DATA_HOME=/data_robot/projects/official_pi05_libero/cache/openpi
"$PY" -B experiments/dynamics/test_pi05_one_step_preflight.py
"$PY" -B experiments/dynamics/pi05_one_step_preflight.py --output /tmp/nero-issue32-preflight-new
```

唯一下一步建议：待足够且均衡的GPU资源可用后，重新冻结设备/FSDP及可整除batch预算，再恢复同一真实模型的单步任务。当前不修改已有任务进程、模型结构或冻结策略。保持独立Draft，不合并；停止等待ChatGPT审查。
