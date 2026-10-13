# 动力学与抓取验收结果

受保护资产 `8e8b805e28bd07a6799d3d838574912ab2280912`；独立工作树 `dynamics-control`。实际环境Python3.11.9/MuJoCo3.2.3。**静态复验PASS，空载动态控制PASS，物理抓取FAIL；完整抓取基线尚未成立。** 实机限位、动力学保真度UNKNOWN。

## 静态、来源和碰撞

127项SHA、原8项测试、11姿态核对实际重新执行PASS；命令、依赖和原始日志在 `runs/static-recheck/`。原资产完整，原main未修改。

官方 `agx_arm_sim@f8cd8b1` 的 `nero.xml` Joint2范围±1.74，与本资产一致；`nero_arm.xml` 为[-0.261799,3.31613]。两文件七轴零点/运动方向在10组FK中与本资产匹配到约1.6e-8m/8.6e-8旋转矩阵误差，不支持90度零点偏移解释。实机有效限位UNKNOWN；源范围不改，实验命令限定[-0.2,1.5]rad。证据 `runs/static-recheck/agx-comparison.json`；原官方文本及blob/SHA在 `references/`。

未过滤模型的具体底座接触为 `base_link_collision_004/link1_collision_000` 和 `base_link_collision_007/link1_collision_000`。约5mm代理重叠在积分中生成最高1.45655e6N内部法向反力，峰值法向Jacobian范数约1.65e-12，不能通过实体自由度消除。峰值代理边界距原底座STL表面约17.75mm，支持凸包填充中空区域的解释；无符号表面距离不是实心材料穿透证明。详见 `runs/risk_20261014_001/source-surface-contact-review.json`。

官方两个MuJoCo XML均排除该相邻对，SRDF标记Adjacent。同条件原/候选各5000真实步的对照只添加这一个exclude，21类机械/几何/接触/actuator/equality数组及全局参数核对相同，内部反力降为0。用户在具体候选及确认请求后指示“继续任务”，据此仅采用该单对过滤，授权记录 `base-filter-approval.json`。历史诊断comparison中的adopted=false保留原样；随后正式控制与抓取采用批准配置。其它碰撞掩码、源网格、接触参数、机械阻尼及equality数值参数未改。抓取002按持久名称比对的21项原机器人数组完全相同，见 `fixed/robot-physical-field-comparison.json`。

## 控制

8D绝对目标映射9个实体关节及9个内部Nm/N servo；夹爪只有一个用户命令。全部参数、单位和来源边界见README及 `config.json`。10项实验测试实际通过，含非法形状、NaN/Inf、复杂数、越界/不可达、力限、无状态写入、自由物体无执行器/外力、单指支撑与过大穿透拒绝，同步目标轨迹以及真实短积分后的Ctrl+C证据保留。日志 `runs/verification_20261014_001/experiment-tests.log`；独立审查发现的中断收尾缺口已修复并回归，取消前的已执行轨迹/计数以INTERRUPTED保存，然后继续传播中断。

采用单对过滤后 `runs/control_approved_20261014_002/approved-filter-control/` 完成21阶段、37000次mj_step/37s，保持、七轴逐轴小动作、多轴组合和夹爪开/半开/闭合/重开全部PASS。最大稳定机械臂误差0.0002161rad；最大空载双指耦合误差0.08235mm；实际最大机械臂/手指速度0.21781rad/s、0.01691m/s，底座异常反力0。完整逐物理步状态、目标、ctrl/actuator力、外力数组与接触在trajectory.npz/contacts.jsonl，执行源码快照在Run根 `executed-source/`。三个主要Run的全部117148行还通过原生integratePos状态连续性复核，最大残差见 `runs/verification_20261014_001/trajectory-audit.json`，包括自由立方体位置/四元数。之前未过滤及首轮过滤控制记录保留。这些结果只证明已检查的空载动作。

## 抓取：两次完整尝试均FAIL

执行前冻结40mm规则立方体、质量100g、摩擦[1,0.005,0.0001]、解析均匀实体惯量、桌面0.13m、最低抬升80mm、保持2s、漂移≤10mm、双指有效接触≥95%、机器人/物体最大有符号接触穿透≤1mm。立方体为自由关节，无物体actuator或weld；robot和物体外力数组全0。合法reset将机器人放到预抓取姿态，随后张开、接近、闭合、抬升、保持和释放全部真实积分，执行期间不写qpos。

| 实际Run | 物理步/时间 | 最大物体抬升 | 最大接触穿透 | 最大加载双指耦合误差 | 结果 |
|---|---:|---:|---:|---:|---|
| grasp_20261014_002/fixed，v1 | 30761 / 30.761s | 3.681mm | 1.333mm | 0.3525mm | FAIL |
| grasp_20261014_003/fixed，v2 | 49387 / 49.387s | 0.138mm | 1.818mm | 0.2127mm | FAIL |

两次保持阶段双指接触比例均0，立方体由桌面支持；保持漂移接近0是物体停在桌面，不能解释为稳定抓持。加载耦合阈值0.2mm也未通过。统计为完整物理尝试0/2成功；两组预定±3mm/±2度偏移按第一固定条件通过后才执行的冻结规则均未执行，不计为测试成功或失败。`grasp_20261014_001`在scratch IK800次迭代预算耗尽时准备失败，0次抓取积分，interruption保留；增加求解预算而不放宽位姿精度后回归通过。

v1实际稳态夹持力约0.70N/指，抬升初段TCP横向摆动约1.3mm，物体滑落。v2只改专家位置目标生成：同步余弦轨迹、目标峰值速度0.05rad/s、闭合位置压缩等效量1N→1.5N；实际夹持约1.07N/指，物体/接触/增益/判定均未变，原冻结IK路径直接复用。失败未删除，也未使用更轻物体、更大摩擦或更松阈值追求成功。

002的16个抽样接触代理边界点距原始夹指STL三角面0.028～0.124mm，未发现该样本范围内的大空腔假接触；这不是整个抓取空间的几何保真证明。`gripper-source-surface-contact-review.json`、`support-force-analysis.json`、`tcp-vs-cube-failure.json`保留核查。003的 `failure-analysis.json`记录实际夹持力、世界坐标支撑力与TCP/物体分离：接触持续滑动、竖直支持不足，随后失去接触。软接触/摩擦正则化及每侧单个凸网格接触点可能是主要阻碍，但各因素贡献尚未用受控对照证明，根因归属UNKNOWN。

MuJoCo3.2.3官方说明指出，平面凸碰撞的单点接触可能缺少支撑，软接触模型也允许缓慢滑动；可分别研究多点检测及摩擦求解参数。[官方防滑说明](https://mujoco.readthedocs.io/en/3.2.3/modeling.html#preventing-slip)。当前仍是默认pyramidal/impratio1/noslip0，未自动采用新碰撞算法、摩擦锥、接触/equality硬化或Noslip。下一步应先提出单因素派生候选、具体diff、复验方案并取得关键物理变更确认；本轮已按停止条件结束自动扩展。

视频为逐帧读取真实积分状态的离线回放，回放本身0次mj_step；replay.json给出源SHA及帧→物理行映射，不用静态图证明成功。两个Run的 `fixed/replay.mp4`、完整trajectory.npz/contacts.jsonl、model.xml、protocol.json、grasp-result.json均可复查；源码快照和冻结配置在Run根。当前grasp.json为v2，v1原协议保留在002冻结记录中。

## 独立审查与交付边界

独立只读Reviewer已完成，报告 `review.md`：资产、参数、映射、真实积分、无捷径、碰撞授权、记录一致及异常测试等10项PASS；抓取能力FAIL，加载耦合FAIL，实机保真度及具体接触根因UNKNOWN。10项当前代码测试独立重跑PASS，中断保存缺口修复已复核，无剩余Important代码问题。控制器与稳定运行入口可复现；失败抓取候选留在独立开发工作树，未作为正式抓取能力合并/推送。没有真机动作、训练、LIBERO/OpenPI、DH116或大型框架。本轮完整物理抓取及工程验收不能标为PASS。
