# 验证范围与复现

## 验证层级

| 层级 | 当前结论 | 范围 |
|---|---|---|
| 厂商机械描述一致性 | 通过 | 29 项冻结源文件的 Git blob/SHA；展开描述与源语义一致 |
| 静态模型 | 通过 | 11 姿态 FK、安装、质量/完整惯性、轴/限位、mimic 和几何边界；`nu=0` |
| 空载控制 | 通过指定动作验收 | 21 阶段、37,000 次 `mj_step`；七轴小动作、多轴组合、夹爪开合与保持 |
| 物理抓取 | 未通过，实验性 | 当前源码固定条件复验 49,387 步失败；历史两次完整尝试也均失败；偏移条件未进入物理执行 |
| 实机动力学一致性 | 未验证 | 无实机标定或物理对照 |

当前源码的复验命令、环境、通过数量和能力状态保存在 [validation.json](../reports/validation.json)。原始静态字段/几何核对见 [static-validation.json](../reports/static-validation.json)，转换来源面归属见 [mesh-conversion.json](../reports/mesh-conversion.json)。

## 自动化验证

```bash
.venv/bin/python scripts/test.py
.venv/bin/python scripts/test.py --layer static
.venv/bin/python scripts/test.py --layer dynamic
.venv/bin/python scripts/test.py --control --run-id my_acceptance
sha256sum -c SHA256SUMS
```

静态层 9 项含原有 8 项机械资产测试及统一入口验证；动态层 12 项包含非法目标、力限、双指映射、自由物体、抓取判定反例、中断证据保存、静态/受控配置字段相等，以及无 Git/旧工作目录依赖的 37,000 步控制。

空载控制判定阈值由 `experiments/dynamics/config.json` 固定：稳态机械臂误差 ≤0.005 rad、每指误差 ≤0.5 mm、双指耦合残差 ≤0.2 mm、实际速度 ≤1 rad/s 和 0.1 m/s，且无非有限值、数值警告、时间重置或额外外力。这些阈值和指定动作定义仿真验收范围，不是厂商性能指标。

抓取成功需要物体抬升 ≥80 mm、保持 2 s、漂移 ≤10 mm、保持期双指有效接触比例 ≥95%、最大机器人/物体穿透 ≤1 mm，且带载耦合满足上述阈值。当前失败不能解释为已支持的抓持能力；单元测试中的判定器反例也不替代完整物理任务验收。

当前源码复验的最大抬升为 0.138 mm、最大穿透 1.818 mm、双指最大耦合残差 0.213 mm，保持期双指接触比例为 0，物体由桌面支持。重复失败未通过修改物理参数或放宽判定标准消除；具体接触根因尚未用受控对照证实。

## 引擎兼容性

运行与完整回归基于 Python 3.11 / MuJoCo 3.2.3。MuJoCo 2.3.7 曾通过模型编译和参考姿态 `mj_forward`，记录在 [兼容性报告](../reports/libero-engine-compatibility.json)；其范围只覆盖静态加载，不保证动态控制、数值接触或其他引擎版本的行为。

## 必要证据追溯

研发审查、迁移过程和原始 Run 归档保留在 Git 历史及开发证据存储，不随当前源码文件集重复打包。重要设计结论已归纳在 `MODEL.md`，普通使用无需阅读研发过程。

| 证据 | 冻结来源 |
|---|---|
| 静态来源和转换 | 当前 `SOURCE_MANIFEST.json`、`reports/static-validation.json`、`reports/mesh-conversion.json` |
| 相邻过滤、控制和失败抓取的原始记录 | `56cfb6255825cc61f0fb46d3848c46c2b1f166c2:reports/dynamics-evidence.tar.xz` |
| 历史结果与审查 | 同一提交的 `reports/dynamic-validation.md`、`reports/dynamic-review.md`；原控制开发提交 `ef15cd524d1d72ff13ad6564c7e681f6ec15f60c` |
| 迁移审计与环境复验 | 同一提交的 `docs/MIGRATION.md`、`reports/migration-validation.json` 与相关日志 |
| 官方碰撞/限位对照 | 当前 `experiments/dynamics/references/manifest.json` 的冻结 URL、Git blob 与 SHA256；原快照在上述提交同路径 |

原始归档 SHA256 为 `fd12ba7206e79f1b0faaaa01391f86f0995d3fd66f32574249547b1cebab7d56`，含 10 个 Run 的 129 个文件，包括两次失败抓取及空载控制。需要维护复查时，在有完整历史的克隆中可用 `git show <提交>:<路径>` 导出；浅克隆可先 `git fetch --unshallow origin`。导出到独立临时目录，校验上述哈希后再解包，避免覆盖已有记录。

本地原始 Run、原开发 worktree 及整理前的逐字节副本继续保留。移出当前文件集只精简检出内容和源码包；完整 Git 历史仍含原始大归档，本次未重写历史。候选范围为静态资产与已复验空载控制；抓取保持实验性。合并稳定主线或创建 Release 需按该范围审核。
