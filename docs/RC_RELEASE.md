# RC交付树与发布边界

本轮只从main选择性引入77bd26a/0d944a2文件，不合并研究历史，不改冻结物理。最终来源见根目录RC_FILE_SELECTION.json，逐项source commit/path/SHA与目标SHA可无Git核对。main全部140个已跟踪文件保留；替换文件逐项解释，不删除旧资产。

## 目录与完整性

- `meshes/`、`urdf/`、原静态XML保留main原字节；新增346个共享MVP mesh与两个已验收XML保持来源SHA不变。
- `assets/`仅索引已有组件、组合和场景，不再复制整机。独立组件XML仍需要共享mesh及纹理。
- `experiments/dynamics/controller.py`、`run.py`、config/R1协议为原验收字节，必要import闭包只有controller/run/simulation_mvp及外部requirements。历史研究/算法PR不是运行依赖。
- 少量已验收报告、NPZ/JSON与346体源几何记录属于审计证据；其余研究留在原分支、Issue、PR和Git历史，未删除唯一原始记录。

SHA256SUMS vRC覆盖最终交付树所有文件，**只排除SHA256SUMS自身**；不包含`.git`、本地虚拟环境、缓存、debug运行输出。RC_FILE_SELECTION.json与SHA256SUMS不在自身/互相的target_sha256字段中形成循环；整体SHA清单负责封存来源矩阵，其自身摘要由外部验收记录提供。实际main为140文件/139条旧校验，不覆盖新增资产，不能将其或更早153项快照当作完整RC清单。

组件manifest不是整体校验表：它仍绑定原机械来源及358个XML/控制脚本/353 mesh/3纹理文件；manifest自身另封存。RC只同步元数据适配后控制脚本的hash，**control配置对象、XML、mesh、所有物理参数未变**。原manifest SHA及新SHA关系在来源矩阵中记录，不伪称manifest整文件原字节相同。

## 接口与安装

- 静态参考整机：9个实体DOF，nu0，适合来源/运动学查看。
- 联合受控MVP：用户位置action为8D `[q1..q7,w]`（rad、m）；实体目标9D `[q1..q7,+w/2,-w/2]`；9D原生ctrl是努力（前7Nm、后2N），不能当位置动作。
- 独立组件：状态只有2个slide，用户命令1D绝对 `w∈[0,.1]m`；w是命令坐标而非标定指尖净距，`q2=-q1`。原kp800/kv8、10N限力、.03m/s限速、100Hz/1kHz保持。
- `flange_mount`表示原`flange_link`坐标，无NERO法兰碰撞；到`gripper_base`仍为z+.006m，姿态无旋转。未来`left_`/`right_`及两个mount只是规划，双臂未实现。

## Git与源码ZIP

两种方式均支持直接加载、控制对象准备及记录版本信息。Git存在时记录真实HEAD；源码ZIP或Git不可用时`head=null`，另记录冻结asset/component来源，不伪造Git版本。SHA256SUMS及各manifest仍可离线核验。

本轮仅把两个记录入口的版本探测改为可选Git，未改控制、计时、插值、solver、接触或成功标准；源函数与模块常量AST受静态检查保护。源码ZIP中的新物理Episode没有在本轮获授权执行，因此只验证其零步准备路径，不宣称重新通过动态实验。

已预建模型无需执行build/extract。原生成器属于来源审计，可能需要原Git历史；ZIP不承诺复现研究构建过程。原动态报告的历史路径与命令是证据，不代表RC当前下载入口。

## 尚未发布

许可证门为BLOCKED_LICENSE_OWNER_DECISION；技术检查通过也不能称RELEASE_READY。没有原创代码LICENSE选择、Tag、Release、合并或关闭旧PR。用户决定许可及正式发布方式后，另行处理，不在本轮自动推进。
