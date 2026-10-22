# Issue #27 — NERO norm stats 与完整 action chunk 训练输入准备

**NERO_TRAINING_DATA_PREP_READY**。`VALID_CHUNK_SAMPLING=PASS`、`NERO_NORM_STATS=PASS`、`NORMALIZED_OPENPI_LOADER=PASS`。本轮只准备现有单条示教的训练输入；没有训练、权重下载、仿真、重渲染、数据导出、资产修改或重新打包。

## 可复用入口与采样

基线为 `7f4af8ee730c0ee7913427c12bc2f37dbe2093b7`，独立分支 `feature/nero-training-data-prep`。新增 `experiments/dynamics/nero_training_data.py`：

- `create_training_view` 用原 OpenPI `create_torch_dataset` 创建真实 LeRobot 数据集，依据原 LeRobot Episode 边界构造 `CompleteActionChunks`。
- 仅过滤**采样起点**，不修改任何帧：原始 682 帧、20Hz；horizon=10，合法起点 0–672，共 673 个。起点 673–681 被视图拒绝，最后合法 chunk 为动作 672–681。
- 所有读取都验证真实 `actions_is_pad` 全 false，并验证 chunk shape `(10,8)`。短 Episode 和多 Episode 边界有针对性测试，不能跨 Episode。
- **后续训练实际入口为 `create_training_loader(stats_directory, model_config, ...)`**。它自行重新创建上述过滤视图，再调用原 `transform_dataset(skip_norm_stats=False)`、`TorchDataLoader`、`DataLoaderImpl`，返回原 `(Observation, actions)` 接口。过滤不依赖本报告的验证循环，没有修改/monkeypatch 共享上游安装。
- shuffle 只作用于 673 个过滤后的索引，已核验索引全集。默认 batch size=1，本轮遍历全部 673 批，包含最后起点 672；其它 batch size 保留上游 `drop_last=True` 行为，不能据整批数量声称完整遍历。

原有通用 OpenPI factory 本身没有 NERO 过滤配置，未来 NERO trainer 必须使用本项目入口；本轮未创建或启动 trainer。

## 真实统计

统计和训练使用相同视图构造、相同合法起点全集。统计位置严格在 **repack + NeroInputs 之后，Normalize、tokenization、32维 padding 之前**。

复用锁定 `scripts/compute_norm_stats.py` 的 `RemoveStrings`、原 `TorchDataLoader` 和 `openpi.shared.normalize.RunningStats/save/load`。上游统计 factory 无过滤视图注入接口，因此仅在项目侧构造过滤视图后接入原统计流程；batch size=1、shuffle=false、673 批，避免原 `floor(len/batch)` 丢尾问题。

| 统计量 | 计数 | 物理维度与单位 |
|---|---:|---|
| state | 673 | 7 arm rad + 2 finger 实际位置 m，共9D |
| actions | 6,730 | 10步×673窗口；7 arm绝对目标rad + gripper总开口目标m，共8D |

6,730 是重叠窗口中动作位置的计权数，不是独立示教数量。保留逐帧 multiplicity：首动作权重1，内部最高10，末动作权重1；673个状态起点各计一次。

每维 mean/std/q01/q99 有限，官方 JSON 保存并加载后数值一致；没有常量维度，也没有原始范围 ≤1e-6 的近常量维度。各维范围、单位、quantile span 和计权清单见 [统计审计](issue27-training-stats-audit.json)。分位数为原 RunningStats 的 5000-bin 在线直方图近似；沿用原 epsilon=1e-6，不加噪声或改写标签。原 float32 RunningStats 相对独立 float64 参考的最大 mean/std 差异分别约 1.15e-6 / 1.85e-5，未改算法。

统计文件：[`experiments/dynamics/nero_norm_stats/norm_stats.json`](../experiments/dynamics/nero_norm_stats/norm_stats.json)。这是原实验 `prep_001/stats/norm_stats.json` 的逐字节副本，SHA256：

```text
59b12d0aa10aa116018cd8d8c0d1d0615fdb85e7a45ac3bc61d647c33a31ff26
```

统计仅适用于本条示教的该训练视图。数据、horizon 或定义改变后应重算；未使用 LIBERO 统计。

## 归一化与真实 batch 结果

明确由官方 `normalize.load` 加载本轮 stats 并注入 DataConfig；`use_quantile_norm=True`、**skip_norm_stats=False**，缺失统计直接报错，没有空统计回退。

真实 `DataLoaderImpl` 完整遍历 673 批，并逐批与原 Parquet 经官方 Normalize 的结果核对。全部状态、动作、图像有限；无时间 padding，无非法起点。起点 0 / 336 / 672 的实际 batch：

- state `(1,32)`，actions `(1,10,32)`，RGB `(1,3,224,224)`，prompt tokens `(1,200)`；逻辑物理维度分别仍为9和8，额外维度为0。
- base 相机 mask=true，两个缺失 wrist mask=false，prompt 沿用 Issue27。
- 原库从 JSON 加载统计后，当前 state/actions batch 为 torch.float64，RGB 为 torch.float32；如实保留官方路径结果，没有在项目适配中额外改 dtype。尚未运行训练或验证优化器路径。
- 官方 `Unnormalize` 加 `NeroOutputs` 后，每个动作恢复 `(10,8)`；对全部6,730个动作位置，8维最大误差均为 **0**（当前原生 float64 loader 路径）。运行前预设容差为 arm `1e-6rad`、width `1e-8m`。
- 归一化动作范围约 `[-1,1.000394]`，确实不同于物理标签；没有 clip 到 `[-1,1]`。32维特征 padding 与时间尾部 padding 明确区分，后者为0。

完整三组 batch 数值、shape、dtype、动作索引、统计SHA、版本及输入摘要见 [真实执行结果](issue27-training-prep.json)。

## 复现与证据

锁定 OpenPI `215abfb217dbac7d5f1273282331b9b1866c0479`、LeRobot `0cf864870cf29f4738d3ade893e6fd13fbd7cdb5`，使用原 server-venv、缓存 tokenizer 和离线模式，未安装/升级依赖。

原数据只读，全部原 manifest 条目运行前后 SHA 一致，682帧保留；共享上游实际模块与 tokenizer 也核验未变。物理 Run、碰撞资产、便携包和 Issue29 回放没有作为本轮运行任务打开或执行。

新实验目录：`/data_robot/projects/.experiments/nero-training-data-issue27-20261023/prep_001/`。包含执行源码、预先冻结方案、完整合法/拒绝索引、stats、统计审计、result和manifest；根目录另有 `prep_001.log`、命令和退出码记录，进程退出码0。3项新边界测试及3项旧接口语义测试通过。

在本分支仓库根目录执行（输出必须为全新目录）：

```bash
PY=/data_robot/projects/official_pi05_libero/server-venv/bin/python
export HF_LEROBOT_HOME=/data_robot/projects/.experiments/nero-mvp-lerobot-issue27-20261020/lerobot
export OPENPI_DATA_HOME=/data_robot/projects/official_pi05_libero/cache/openpi
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 JAX_PLATFORMS=cpu OPENBLAS_NUM_THREADS=1
"$PY" -B experiments/dynamics/test_nero_training_data.py
"$PY" -B experiments/dynamics/prepare_nero_training_data.py --output /tmp/nero-prep-new
```

后续 NERO 训练代码直接复用（本轮未执行训练）：

```python
from pathlib import Path
import sys
from openpi.models.pi0_config import Pi0Config
sys.path.insert(0, str(Path("experiments/dynamics").resolve()))
from nero_training_data import create_training_loader

loader = create_training_loader(
    Path("experiments/dynamics/nero_norm_stats"),
    Pi0Config(pi05=True, action_horizon=10, action_dim=32),
    batch_size=1, shuffle=True,
)
# loader.data_config() 含实际NERO stats；迭代返回原OpenPI Observation/actions。
```

只完成本单条示教的训练输入准备，不声称训练已经启动、策略学会抓取或泛化通过。保持独立 Draft，不合并；完成后停止等待 ChatGPT 审查。
