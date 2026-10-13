# 单仓库迁移独立只读 Review

审查者 `/root/consolidation_review`，未参与实现，2026-10-14。审查对象为main@8e8b805到feature/dynamic-control的迁移候选。

**工程迁移PASS，无剩余Important问题；物理抓取仍FAIL，不能合并main。** 根依赖缺少trimesh/pycollada、clone未指定Feature分支两项问题均已修正；全新临时环境已实测先复现缺包错误，修正声明后完整测试通过。

| 检查 | 结论 | 独立证据 |
|---|---|---|
| 单仓库、统一说明/入口/依赖 | PASS | README、requirements.txt、scripts/test.py；动态代码读取根资产 |
| 机械资产和原静态测试 | PASS | 103个机械文件及tests/test_assets.py与8e8b805逐字节相同 |
| 静态/动态派生边界 | PASS | nu0/nu9，机械字段相同，唯一新增排除base_link/link1 |
| 统一测试 | PASS | 独立21/21，静态9项、动态12项，退出码0 |
| 可搬迁控制 | PASS | 无.git/旧worktree临时副本完成37000步、21阶段 |
| 原Run和归档 | PASS | 117原文件一致，9清单108内容哈希匹配，归档129文件完整恢复 |
| 防覆盖 | PASS | 临时重复tar --keep-old-files返回2，预置文件原字节保留 |
| 校验及历史 | PASS | 审查候选151项SHA一致；4原提交保留，本地及实查远程main均8e8b805 |
| 原截图 | PASS | 两张根截图匹配迁移前记录；受控报告图像与基线相同 |
| 完整物理抓取/带载耦合 | FAIL | 两份grasp-result均FAIL，完整尝试0/2 |
| 真机保真度/具体接触根因 | UNKNOWN | 本轮未新增这类证据 |

Reviewer实测：`PYTHONDONTWRITEBYTECODE=1 MUJOCO_GL=egl MUJOCO_EGL_DEVICE_ID=1 .venv/bin/python scripts/test.py`，静态22.759s、动态44.754s。另执行SHA、基线字节、临时归档恢复与防覆盖核对。未修改正式目录、Git或原Run。

证据：migration-validation.json、migration-tests.log、migration-fresh-tests.log、dynamic-status.json、dynamics-evidence.tar.xz及experiments/dynamics/runs。归档SHA256 `fd12ba7206e79f1b0faaaa01391f86f0995d3fd66f32574249547b1cebab7d56`。最终补写报告后，交付SHA另行刷新与复核，未再次修改控制实现或物理参数。
