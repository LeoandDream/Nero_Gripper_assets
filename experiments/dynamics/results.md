# 当前验收状态

原始资产版本 `8e8b805e28bd07a6799d3d838574912ab2280912`，实验在独立 `dynamics-control` 工作树。全部物理执行仅使用MuJoCo3.2.3/Python3.11.9；没有真机、LIBERO/OpenPI或训练。

## 已完成

- 静态复验：127项SHA、8项原测试、11姿态独立核对PASS。原始127项交付文件保持原字节。命令、环境与日志在 `runs/static-recheck/`。
- Joint2来源比较：官方 `agx_arm_sim@f8cd8b1` 的 `nero.xml` 范围±1.74，与资产一致；`nero_arm.xml` 为[-0.261799,3.31613]。两文件的七轴零点/运动方向在10组静态FK中与本资产匹配到约1.6e-8m/8.6e-8旋转矩阵误差，不支持用90度零点偏移解释范围差异。实机有效限位UNKNOWN；原URDF范围不改，实验命令采用[-0.2,1.5]rad保守候选包络。证据在 `runs/static-recheck/agx-comparison.json`，原官方文本和blob哈希在 `references/`。
- 控制接口：8D绝对目标（7rad+1m）映射9个实体目标和9个内部Nm/N控制通道；夹爪两指目标由同一宽度确定。三项契约/异常输入/力限测试PASS，见 `debug/controller-tests-green.log`。
- 未过滤模型的保持、七轴逐轴小幅运动、多轴组合、夹爪开/半开/闭合/重开21阶段全部通过已冻结的控制阈值。真实37000次mj_step/37s，完整状态、命令、力、接触记录在 `runs/control_unfiltered_20261014_001/`。最大末段机械臂误差约0.000216rad，手指误差约7.97e-8m，动态耦合最大误差约8.24e-5m；关节最大速度0.218rad/s、手指0.0169m/s。这只证明本次测量范围的受控跟踪，不替代下述接触物理审查。

## 需要确认的接触候选

base_link/link1 的具体接触对为base_link_collision_004/link1_collision_000和base_link_collision_007/link1_collision_000；约5mm代理重叠在实际积分中生成最高约1.45655e6N内部法向反力，峰值在第一对。峰值接触法向Jacobian范数仅约1.65e-12；固定底座及铰链已经限制这一方向，代理重叠无法通过实体自由度消除。控制跟踪虽然稳定，不能把这项虚假内部反力包装为可信接触。

峰值代理边界点到底座原STL三角面的最近距离约17.75mm，link1侧约0.295mm，支持凸包填充中空区域产生该接触的解释；这是源表面的无符号距离，不是实心材料穿透深度或全空间表面等价证明。详见 `runs/risk_20261014_001/source-surface-contact-review.json`。

官方MuJoCo两份XML显式排除base_link/link1，官方SRDF将其标为Adjacent。仅在独立动态候选中增加该单对exclude的同条件对照，前后各真实5000步/5s：该内部反力由约146万N变为0，测量的保持/Joint1动作仍通过；21类机械、几何、接触、执行器、夹爪约束数组完全相同。其它碰撞掩码、非相邻对、物体接触和源资产不变。

此候选尚未采用为正式动态/抓取基线，按照用户任务第3.3节等待确认。具体diff与前后全轨迹见 `runs/risk_20261014_001/candidate-diff.xml`、`comparison.json`、`unfiltered/` 与 `single-pair-exclusion-candidate/`。

## 尚未完成

物理抓取/抬升/保持未执行；独立最终Reviewer尚未执行。需要先确认上述单对过滤，随后在采用的动态模型上重新完成控制验收，再执行事先冻结的立方体任务。已有抓取FK可达性仅为scratch MjData计算，不当作动作或成功证据。没有擅自改变原几何、惯性、机械阻尼、摩擦、equality数值参数或成功条件。
