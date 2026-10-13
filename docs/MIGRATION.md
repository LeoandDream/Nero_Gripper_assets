# NERO + Gripper 单仓库收敛

本轮只统一工程与交付入口，不重建资产、不调整物理参数。唯一仓库为 `LeoandDream/Nero_Gripper_assets`，开发分支 `feature/dynamic-control`；原main在静态基线8e8b805，抓取未验收前不合并。

## 只读审计

| 目录 | 分支 / 原HEAD | Git关系与未提交内容 |
|---|---|---|
| `/data_robot/projects/nero_gripper_assets` | main / 8e8b805 | 正式checkout；原有未跟踪closed.png、preview.png保留 |
| `/data_robot/projects/nero_gripper_dynamics` | dynamics-control / ef15cd5 | 同一Git的linked worktree；tracked文件干净，runs/debug被ignore |

两者remote均为 `https://github.com/LeoandDream/Nero_Gripper_assets.git`，共用git-common-dir，merge-base为8e8b805；远程main实查仍为8e8b805。不是两个不相关的Git历史，采用相关提交的fast-forward，不使用unrelated-histories。

来源4个提交完整保留：f295939、2b8b2a8、44f497d、ef15cd5。相对main仅新增experiments/dynamics下18文件，无机器人资产改动。两个目录的127项旧SHA和103个mjcf/urdf/mesh基线blob一致。原资产nu=0；动态build_model读取当前checkout根mjcf/nero_gripper.xml，资源为相对路径，无旧worktree运行依赖。官方sim references仅用于来源审计，不作为另一套运行资产。

原始9个Run共249023051字节，9清单/108内容文件SHA全部匹配；含清单117文件无遗漏。控制、抓取002/003有执行源码快照；更早记录并非全部有快照，保留来源提交和当时的哈希，不补写伪造历史。debug156KiB及失败、IK诊断、中断修复记录保留在开发区和原worktree。

## 最小迁移清单与验证计划

- [x] 正式目录从main创建feature/dynamic-control，fast-forward来源4提交；原source分支与worktree保留。
- [x] 实验README并入根README，结果/审查/状态进入已有reports；保留运行代码和配置位置，避免大量重构。
- [x] 根运行依赖补齐SciPy；统一scripts/test.py入口，保留原8项静态断言，增加nu0/nu9及共享机械字段/可搬迁验证。
- [x] 原始Run保持同相对层级、逐字节保留；在reports提供一个可解包的证据归档与哈希清单，远程克隆也能获得必要历史证据。
- [x] 原SHA清单按授权文档/依赖/新增文件更新；URDF、mesh、MJCF、惯性/安装/限位、旧静态报告逐字节不变并单独核验。
- [x] 实际重跑统一静态/动态测试、37s/21阶段控制及静态validate；只做既有能力复验，不新增抓取调参。
- [x] 独立只读Reviewer核对迁移完整、统一入口、资产派生和验收边界。

## 合并门槛与原目录保留

历史静态与空载控制PASS；完整抓取0/2、加载耦合FAIL，偏移条件未执行，硬件保真度与具体接触根因UNKNOWN。本轮工程迁移通过不改变这些物理结论。Feature可提交供审查，main保持原基线。

原动态worktree作为历史来源保留，不删除、不清理Git历史。统一功能和证据完整核对后，只能提出将其标记只读/归档的建议；任何删除仍需要用户确认。

实际复验：统一入口21/21测试PASS（原静态8项全部保留），另保存37000步/21阶段控制PASS；静态validate11姿态PASS。临时可搬迁副本也完成37000步控制PASS。原始117文件复制后逐字节一致，103权威机械文件保持基线原字节，两个原有截图哈希一致。命令、环境和原始输出见reports/migration-validation.json及migration-tests.log。

独立迁移审查PASS（reports/migration-review.md）。根依赖漏列trimesh/pycollada已在干净环境复现并修复；根README clone明确Feature分支。现有环境、全新临时环境与独立Reviewer均21/21 PASS；归档129文件解包内容一致，keep-old-files重复解包保持原文件。源4提交、source worktree和main基线均保留。本轮只发布Feature供复现，不合并main。
