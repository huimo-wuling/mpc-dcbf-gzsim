# 复现说明（方法、指标定义与结果）

本文件说明如何在本机复现论文的仿真实验、指标如何计算，以及结果与论文的对照。
**注意：本仓库只复现论文提出的方法本身**（MPC-D-CBF + KF 预测），不包含论文的 4 个对比
基线——它们的代码并未随论文开源，本项目也不自行臆造基线。

## 1. 被复现的内容

论文（Jian et al., ICRA 2023）的仿真实验是：机器人在包含静态与动态圆柱的场景中，从
(-10, 0) 驶向 (9, 0)，用 LiDAR → 局部高程图 → DBSCAN → 最小包围椭圆 → 匈牙利跟踪 →
卡尔曼滤波预测 → D-CBF/MPC 的完整链路完成安全避障，并报告 Table I 的四项指标。

论文 Table I（仿真）中 "Ours" 的数值：

| Min dist [m] | Cons time [s] | Reac time [s] | Speed var |
|---|---|---|---|
| 0.828 | 21.52 | 0.353 | 0.0172 |

## 2. 实验方法

* **场景**：直接沿用原仓库 `scene/worlds/world.world` 转换来的
  `src/mpc_dcbf_bringup/worlds/mpc_dcbf_world.sdf`：平坦地面 + 8 个红色圆柱
  （r = 0.25 m，h = 0.5 m），初始位姿与速度取自
  `src/mpc_dcbf_bringup/config/obstacles.yaml`。机器人出生在 (-10, 0)，目标 (9, 0)，
  全局路径是 190 个间隔 0.1 m 的直线路点。
* **障碍物运动**：`movetest` 以 100 Hz 通过 gz-transport `/world/mpc_dcbf/set_pose` 把圆柱
  按 `p += v` 绝对设定（0.5–1.0 m/s），与原实现用 `/gazebo/set_model_state` 的语义一致；
  同时以 20 Hz 发布 `/obstacles/true_poses` 供评估。
* **执行**：`scripts/run_experiment.sh` 启动 `gz sim`（headless）+ 全链路节点，4 s 后开始
  记录 60 s，然后关闭并分析。每次运行使用独立的 `ROS_DOMAIN_ID` 与 `GZ_PARTITION`。
* **数据**：`scripts/record_run.py` 记录真值位姿、`/cmd_vel`、`/obs_predict_pub`
  （约 1 Hz 抽样）与 `/local_planner/solve_time`。
* **代码路径**：与论文/上游实现一致——MPC 每个周期重新构建 CasADi 问题、IPOPT 参数不变、
  椭圆不确定性膨胀不变；只做"能在 ROS 2 + Gazebo Sim 上运行"所必需的移植
  （见 `docs/port_notes.md`）。

## 3. 指标定义（本仓库实现）

| 指标 | 定义 |
|---|---|
| **Min dist** | 机器人中心到任意圆柱表面（r = 0.25 m）的最小距离，逐里程计样本计算 |
| **Cons time** | 机器人首次进入目标点 0.2 m 范围内的时间 |
| **Reac time** | 首次有障碍物进入 7 m 局部地图范围 → 之后第一次出现 \|ω\| > 0.05 rad/s 的时间差 |
| **Speed var** | 有障碍物处于 3 m 以内时，`/cmd_vel` 线速度的方差 |
| 附加 | 路径长度、终点误差、重规划频率、KF 预测频率、MPC 求解耗时（均值/p95/最大） |

论文未给出这四项的精确公式，以上是按论文文字描述实现的等价定义，因此与论文的绝对值存在
定义层面的细微差别。

## 4. 结果

见 README 第 6 节（由 `results/metrics_r*.json` 得到），以及：

* `results/trajectory.png`：本次运行的轨迹 + 最近时刻的 CBF 预测椭圆；
* `results/speed_clearance.png`：线速度与最小间距随时间变化；
* `results/run_r*.json`：原始记录（体积较大，已被 `.gitignore` 排除，可用
  `scripts/run_repeats.sh` 重新生成）。

### 观察

1. **控制链路完全复现**：局部地图 11 Hz、KF 预测 ~10.7 Hz、重规划 15–19 Hz、控制器 50 Hz；
   机器人每次都到达终点（终点误差 0.10 m），路径长度 ≈ 19.3 m（直线 19 m）。
2. **实时性与论文同量级**：MPC 单次求解均值 ≈ 19 ms（p95 ≈ 22 ms），重规划 ≈ 19 Hz，
   论文在 Intel NUC 上报告 10 Hz 规划。
3. **安全间距没有达到论文的 0.828 m**：本机重复运行最小间距在 0.28–0.55 m 之间（见 README
   表）。除了场景/传感器配置差异外，主要原因是**估计误差与延迟**：CBF 约束作用在 10 Hz
   更新、带不确定性膨胀的*估计*椭圆上，两次更新之间预测会滞后；此外 `h()` 把机器人当作
   质点（未计车体外廓），因此"中心到圆柱表面"的真实间距会比设计值 `d_safe = 0.5 m` 更小。
4. **速度方差比论文大**：论文 0.0172，本机 0.08–0.13。原因是本移植沿用原实现的椭圆膨胀
   `a ← a + pos_conf + ab_conf`，膨胀量偏大、CBF 约束更早生效，速度在 0.1–1.2 m/s 间反复
   调节。这属于参数/标定差异，未做任何"调参美化"。
5. **未复现论文的 4 个基线**：其代码未开源，本项目不做对比。
