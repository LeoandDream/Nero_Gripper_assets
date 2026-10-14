# 来源与许可

原始Xacro/URDF、DAE、STL和上游README来自 `agilexrobotics/agx_arm_urdf` 提交 `a68b8e876a2c6fcb0ecee7a7943129e4d48dd864`，逐项Git blob ID、SHA256及相对路径见 `SOURCE_MANIFEST.json`。

该提交的完整MIT许可原字节保存在 `AGX_MIT.txt`，上游README在 `UPSTREAM_README.md`。派生可视OBJ、PNG颜色调色板和凸碰撞体均由这些源文件生成，构建脚本与转换报告保留其来源。SolidWorks导出器说明保留在原描述文件中。

`AGX_MIT.txt` 覆盖上述厂商来源及其派生资产。新增工程代码未单独声明发行许可证，不能将厂商版权声明自动套用于本仓库全部内容；创建公开 Release 前需要仓库维护者确认新增代码的许可。

碰撞关系和 Joint 2 范围的比较依据另来自 `agilexrobotics/agx_arm_sim@f8cd8b147c75d59e14f90fb0646770eefa268ed0`。该版本 README 声明 MIT，但 `Moveit2/nero_gripper_moveit_config/package.xml` 声明 BSD，未提供完整版权/许可文本。当前检出文件集仅保留冻结链接、Git blob 和 SHA256 索引，不包含这三个比较 XML/SRDF 快照。它们不是运行模型依赖，原字节仍保留在 Git 历史及开发证据存储；不能由现有 `AGX_MIT.txt` 推定其许可范围。
