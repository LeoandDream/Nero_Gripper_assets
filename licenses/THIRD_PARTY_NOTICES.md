# 第三方来源与边界（非整仓许可证声明）

| 类别 | 来源/保留文本 | RC用途及待决边界 |
|---|---|---|
| 厂商原始Xacro/URDF、DAE、STL和README | agilexrobotics/agx_arm_urdf@a68b8e876a2c6fcb0ecee7a7943129e4d48dd864；SOURCE_MANIFEST.json，AGX_MIT.txt，UPSTREAM_README.md | 原始厂商文本按原字节保留；不将其许可自动扩展到全部原创工程代码 |
| 派生XML、OBJ、PNG、凸碰撞资产 | 上述源 + main/77bd26a/0d944a2；矩阵逐文件SHA | 按来源归属整理。衍生发布范围及必要版权保留由所有者确认，本文件不作新许可选择 |
| 原创控制、验证、导出与发布工程代码 | 本仓库历史提交，RC_FILE_SELECTION记录 | 未确定发行许可；正式Release被所有者许可决策阻断 |
| 非运行研究对照 | agx_arm_sim@f8cd8b147c75d59e14f90fb0646770eefa268ed0；experiments/dynamics/references/manifest.json | 只保留URL/Git blob/SHA，不包含三个比较XML/SRDF快照；上游README MIT与Moveit2包BSD信息需独立确认，AGX_MIT不能自动代替 |
| Panda/robosuite等研究对照 | 原Issue21/PR22及研究分支 | 不作为本RC模型或运行依赖，没有复制其模型网格/代码进入运行闭包 |
| Python运行依赖 | requirements.txt中的MuJoCo/NumPy/Pillow/SciPy/trimesh/pycollada | 独立安装，不随本RC捆绑其二进制或覆盖其许可证 |

本清单用于归属整理，不声明整个仓库MIT/BSD/Apache授权。没有添加原创代码LICENSE。所有者待决事项见LICENSE_DECISION_REQUIRED.md。
