# 项目指南（GUIDE）

这份文档回答四个问题：

1. 这个仓库由什么组成，怎么把仿真实验跑起来；
2. 跑它需要哪些**前置软件知识**；
3. 要读懂里面的开源代码，需要补哪些**算法与工程知识**，以及按什么顺序读；
4. 读 / 改这份代码需要哪些**编程知识**（语言、库、消息约定、调试、动手示例）。

> 面向三类读者：想"跑一遍看结果"的（第 2 章）；想"改参数/换场景"的（第 2.6 节）；
> 想"读懂算法、动手改代码"的（第 3、4、5 章）。
> 论文本身的公式与翻译见桌面上的《论文翻译与注释》，本仓库的移植细节见
> [`port_notes.md`](port_notes.md)，实验方法与结果见 [`reproduction.md`](reproduction.md)。

---

## 1. 项目组成

### 1.1 一句话概括

一个**移动机器人动态避障**的完整闭环：LiDAR 点云 → 局部 2.5D 栅格 → 聚类成椭圆 →
卡尔曼滤波预测未来轨迹 → 带 D-CBF 安全约束的 MPC 生成轨迹 → 差速底盘执行。
本仓库是上述官方实现从 ROS 1 + Gazebo Classic 到 **ROS 2 + Gazebo Sim** 的移植版。

### 1.2 目录与职责

```
mpc-dcbf-gzsim/
├── src/
│   ├── mpc_dcbf_bringup/   ← 顶层：launch、世界文件、桥接配置、RViz、URDF
│   ├── jackal_gz/          ← Jackal 的 SDF 模型（差速驱动 + 32 线激光雷达）
│   ├── mpc_scene/          ← 位姿真值转 TF、动态障碍物驱动
│   ├── local_map/          ← 点云 → 局部高程图 → DBSCAN → 最小包围椭圆 → 跟踪
│   ├── obs_param/          ← 卡尔曼滤波 + 25 步椭圆预测
│   ├── local_planner/      ← 核心：MPC-D-CBF 规划器 + 50 Hz 控制器
│   └── global_path_publisher/ ← 直线参考路径
├── scripts/                ← 依赖导入、启动/停止、实验、记录、指标、绘图
├── results/                ← 复现产物（图 + 指标）
├── patches/                ← grid_map 的 ament 补丁
├── mpc_dcbf.repos          ← 第三方仓库清单
└── docs/                   ← 移植笔记、复现说明、本指南
```

### 1.3 六个 ROS 2 包逐个看

| 包 | 语言 | 关键文件 | 做什么 | 主要输入 → 输出 |
|---|---|---|---|---|
| `mpc_dcbf_bringup` | Python/YAML/SDF | `launch/simulation.launch.py`、`worlds/mpc_dcbf_world.sdf`、`config/bridge.yaml`、`config/obstacles.yaml`、`rviz/simulation.rviz`、`urdf/jackal_tf.urdf` | 把下面所有东西串起来启动；定义世界、8 个圆柱、gz↔ROS 话题映射 | — |
| `jackal_gz` | SDF | `models/jackal/model.sdf` | 机器人本体：`DiffDrive`（订阅 `/cmd_vel`，发布 `/model/jackal/odometry`）、`gpu_lidar`（32 线、720 方位、10 Hz、z=0.5 m）、四轮关节 | `/cmd_vel` → 轮速 / `/velodyne_points/points` |
| `mpc_scene` | C++ | `src/odom_tf.cpp`、`src/movetest.cpp` | ① 用里程计 + 出生位姿发布 `world → base_link` 的 TF；② 100 Hz 用 gz 服务把 8 个圆柱按速度平移，并发布真值位姿 | `/base_pose_ground_truth` → `/tf`；参数 → `/obstacles/true_poses` |
| `local_map` | C++ (PCL + grid_map + CGAL) | `src/local_map_pub.cpp`、`include/{DBSCAN,ellipse,KM}.hpp` | 点云裁剪/滤波 → 世界系 2.5D 高程图 → Sobel/Laplace/台阶阈值 → DBSCAN 聚类 → 最小包围椭圆（MBE）→ 匈牙利跨帧跟踪 → 形状方差 | `/velodyne_points` → `/local_map_pub/{for_obs_track,gridmap,local_pcd,ellipse_vis}` |
| `obs_param` | C++ (Eigen) | `src/obs_kf.cpp`、`src/kalman.cpp`、`include/kalman.h` | 每个障碍物一个 9 维卡尔曼滤波（位置/速度/加速度/椭圆三参数），外推 25 步，并按协方差把椭圆放大 | `/local_map_pub/for_obs_track` → `/obs_predict_pub`、`/obs_predict_vis_pub` |
| `local_planner` | Python (CasADi/IPOPT) | `local_planner/local_planner.py`、`controller.py` | ① MPC：25 步滚动优化，代价 = 跟踪 + 控制量 + 控制变化率，约束 = 差速模型 + 速度/角速度界 + D-CBF；② controller：50 Hz 把规划第一步变成 `/cmd_vel` | `/curr_state`、`/global_path`、`/obs_predict_pub` → `/local_plan`、`/local_path`；`/local_plan` → `/cmd_vel` |
| `global_path_publisher` | C++ | `src/global_path_publisher.cpp` | 从 (-10,0) 到 (9,0)、间隔 0.1 m、190 个路点，10 Hz 发布 | 参数 → `/global_path` |

### 1.4 数据流（话题级）

```
                        ┌──────────────────────── gz sim ────────────────────────┐
 /cmd_vel ──bridge──▶   │ DiffDrive ──▶ /model/jackal/odometry ──bridge──▶ /base_pose_ground_truth
                        │ gpu_lidar ──▶ /velodyne_points/points ─bridge──▶ /velodyne_points
                        └────────────────────────────────────────────────────────┘
 /base_pose_ground_truth ──▶ odom_tf ──▶ TF(world→base_link) ──▶ controller ──▶ /curr_state
 /velodyne_points ──▶ local_map_pub ──▶ /local_map_pub/for_obs_track ──▶ obs_kf ──▶ /obs_predict_pub
 /global_path ────────────────────────────────────────────────┐
 /curr_state ─────────────────────────────────────────────────┤──▶ local_planner(MPC-D-CBF)
 /obs_predict_pub ────────────────────────────────────────────┘        │
                                                                      ▼
                                                        /local_plan ──▶ controller ──▶ /cmd_vel
```

关键频率（实测值，括号内为代码设定）：

| 环节 | 频率 |
|---|---|
| LiDAR | 10 Hz |
| 局部地图 | 11 Hz（定时器 91 ms） |
| KF 预测 | ~11 Hz（跟随上游消息） |
| MPC 重规划 | 17–19 Hz（定时器 50 ms，单次求解 ≈20 ms） |
| 控制器 | 50 Hz |
| 障碍物遥操作 | 100 Hz |

### 1.5 一次实验的闭环

`/cmd_vel` → DiffDrive 转成四轮速度 → 车在物理世界里动 → 里程计积分 →
`odom_tf` 发布 `world→base_link` → controller 读出位姿发 `/curr_state` →
规划器据此重新优化。**这条环里任何一个话题断了，车就不动**（排查表见第 2.7 节）。

---

## 2. 跑仿真实验

### 2.1 环境检查（30 秒）

```bash
cat /etc/os-release | head -2          # 期望 Ubuntu 24.04/26.04 一类
ls /opt/ros/                          # 期望有 lyrical（或其他已适配的发行版）
gz sim --versions                      # 期望 10.x
python3 -c "import casadi; print(casadi.__version__, casadi.__file__)"
```

最后一条要看到 **3.8.x 且路径在 `~/.local/...`（pip 安装）**，不能是 `/usr/lib/python3/dist-packages`
（apt 版本与 numpy 2.x ABI 不兼容，会直接段错误）。

### 2.2 装依赖

```bash
sudo apt install -y libcgal-dev ros-lyrical-ros-gz ros-lyrical-rviz2 \
                    ros-lyrical-pcl-ros ros-lyrical-nav2-msgs ros-lyrical-rosbag2-cpp
pip3 install --user --break-system-packages casadi==3.8.1   # 自带 IPOPT
```

### 2.3 构建

```bash
cd ~/mpc-dcbf-gzsim
bash scripts/import_repos.sh              # 拉 grid_map(ros2) + filters，并打 ament 补丁
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 的好处：改 Python/launch/YAML/SDF **不用重编译**；改 C++ 才需要
`colcon build --packages-select <包名>`。

### 2.4 启动与停止（可反复执行）

```bash
bash scripts/start.sh                              # Gazebo GUI + RViz
bash scripts/start.sh headless:=true               # 只跑 Gazebo 服务端 + RViz
bash scripts/start.sh headless:=true rviz:=false   # 纯无头（做实验用）
bash scripts/stop.sh                               # 手动清理（一般 Ctrl+C 就够）
```

> 为什么要 `start.sh`：Gazebo 服务端经常在 `Ctrl+C` 后残留，**点窗口 × 只关界面**，
> 不清理就重启会撞 `Another world of the same name is running`。
> `start.sh` 会先清理再启动。

### 2.5 跑一次完整实验并出图

```bash
bash scripts/run_experiment.sh 60 results/run.json "MPC-D-CBF"   # 单次 60 s
bash scripts/run_repeats.sh 3 60                                  # 重复 3 次
```

脚本做的事：清理 → 无头启动 → 记录 60 s 真值/指令/求解耗时 → 关闭 → 计算指标 → 出图。
产物：

| 文件 | 内容 |
|---|---|
| `results/metrics_r*.json`、`repeats.md` | Table I 式指标 |
| `results/trajectory.png` | XY 轨迹 + 最近时刻的预测椭圆 |
| `results/speed_clearance.png` | 线速度、最小间距随时间 |
| `results/run_r*.json` | 原始记录（默认被 `.gitignore` 排除） |

指标定义（见 `scripts/analyze_run.py`）：`Min dist` 机器人中心到圆柱表面最小距离；
`Cons time` 首次进入终点 0.2 m 的耗时；`Reac time` 障碍物进入 7 m 到 |ω|>0.05 的时间差；
`Speed var` 有障碍物在 3 m 内时线速度方差。

### 2.6 想改点什么

| 想改 | 改哪里 |
|---|---|
| 障碍物初始位置/速度 | `src/mpc_dcbf_bringup/config/obstacles.yaml`（`movetest` 读它） |
| 世界里的圆柱数量/尺寸 | `src/mpc_dcbf_bringup/worlds/mpc_dcbf_world.sdf` |
| 起终点、路点间隔 | `src/mpc_dcbf_bringup/launch/simulation.launch.py` 里 `global_path` 的 `start_x/end_x` |
| 重规划频率 | launch 里 `replan_period`（默认 0.05 s） |
| 地图尺寸/分辨率/聚类参数 | launch 里 `local_map` 的 `localmap_x_size`、`resolution`、`DBSCAN_R/N`、`obs_height`、`step_height` |
| 安全距离、速度上限、γ | `src/local_planner/local_planner/local_planner.py` 的 `safe_dist`、`v_max/v_min`、`omega_max`、`gamma_k` |
| KF 噪声/膨胀 | `src/obs_param/src/obs_kf.cpp` 的 `P0_cov/Q_cov/R_cov`，以及 `a = m_x(2) + pos_conf + ab_conf` |

### 2.7 常见故障速查

| 现象 | 先查什么 | 常见原因 |
|---|---|---|
| 第二次启动报 `Another world of the same name` | `ps -eo cmd \| grep gz` | 上次 gz 服务端没退 → `bash scripts/stop.sh` |
| 车不动、`Linear Speed: 0.0` | `ros2 topic hz /local_plan` | 规划器没出解 / `/curr_state` 或 `/obs_predict_pub` 没数据 |
| 有 `/local_plan` 但车不动 | `ros2 topic info /cmd_vel -v` | gz 侧 `DiffDrive` 没订阅（`<topic>` 配错）或桥接断了 |
| 没有任何椭圆 | `ros2 topic hz /local_map_pub/for_obs_track` | 点云没进 `local_map`、参数把点全过滤了、地图全局量未赋值 |
| RViz 里没机器人 | `ros2 topic echo /robot_description --once` | `robot_state_publisher` 没起或 URDF 没加载 |
| RViz 里没黄线 | `ros2 topic info /global_path -v` | QoS 不匹配（必须是 Volatile） |
| 规划器一启动就崩 | 看终端是否 `segfault` | 用了 apt 的 casadi |
| `colcon build` 找不到 CGAL / grid_map | — | 装 `libcgal-dev` / 先跑 `import_repos.sh` |

调试三板斧：

```bash
ros2 topic list | sort                 # 有哪些话题
ros2 topic hz /local_plan              # 有没有数据、频率对不对
ros2 topic info /cmd_vel -v            # 类型、QoS、发布者/订阅者数量
ros2 run tf2_ros tf2_echo world base_link   # TF 通不通
gz topic -l                            # gz 侧有哪些话题
```

---

## 3. 前置软件知识（"能跑起来"需要的）

按优先级排，前 4 条是硬门槛。

1. **Linux 命令行**：`cd/ls/grep/ps/kill/pkill`、`source` 环境变量、`|` 管道、后台进程 `&`。
   本仓库大量使用 `source /opt/ros/.../setup.bash`，不理解它会遇到 90% 的"找不到包"。
2. **ROS 2 基本模型**（半天可入门）：
   - 节点 / 话题 / 服务 / 参数 / launch；
   - 消息类型（`sensor_msgs/PointCloud2`、`nav_msgs/Odometry`、`visualization_msgs/MarkerArray`）；
   - **QoS**（reliability、durability、history）——本项目 RViz 的坑就出在 durability；
   - **TF2**（坐标系树，`world→base_link→velodyne`）；
   - 命令行：`ros2 run/launch/topic echo|hz|info|list/param`；
   - 构建：`colcon build`、`source install/setup.bash`、`package.xml`/`CMakeLists.txt`。
3. **Gazebo Sim 与桥接**：
   - SDF 与 URDF 的区别（SDF 描述仿真世界与插件，URDF 描述机器人运动学，本仓库两者都用）；
   - 常用插件：`DiffDrive`、`gpu_lidar`、`JointStatePublisher`；
   - gz 的传输层（`gz topic -l/-e`）与 `ros_gz_bridge` 的"方向 + 类型映射"；
   - 记住：gz 的点云在 `<topic>/points`、里程计在 `/model/<名>/odometry`。
4. **Python 3 + numpy**：本项目的规划器是 Python；`numpy` 数组、`np.diag`、切片广播；
   以及 ROS 2 的 Python 绑定 `rclpy`（节点、订阅、定时器、回调组、执行器）。
5. **C++ 能读懂即可**：`local_map`/`obs_param` 是 C++；需要看得懂类、模板、Eigen 表达式、
   PCL/grid_map 的 API 调用；能根据编译错误定位缺依赖就行。
6. **CMake/包管理**：`find_package`、`ament_target_dependencies`（新版移除，见 `patches/`）、
   `pip --user` vs `apt`、`COLCON_IGNORE`。
7. **调试工具**：`rviz2`（可视化）、`tf2_echo`、`ros2 topic hz/echo`、`py-spy`（看 Python 卡在哪）、
   `gz topic -e`、`ps/pkill`（进程残留是 Gazebo 的日常）。

> 如果这些还不熟：建议先用 `turtlesim` 和官方 ROS 2 教程过一遍节点/话题/TF/launch，
> 再回来看本项目，能省掉大量"为什么没数据"的困惑。

---

## 4. 读懂开源代码需要的知识

### 4.1 控制理论（对应 `local_planner.py` 的约束部分）

| 概念 | 在这里指什么 | 代码位置 |
|---|---|---|
| 控制屏障函数 CBF | 一个标量函数 `h(x)`，`h≥0` 为安全集，靠约束 `ḣ ≥ -γ(h)` 保证前向不变 | `h()` 函数 |
| 离散时间 CBF | `h(x_{k+1}) ≥ (1-γ)·h(x_k)`，`0<γ≤1` | `opti.subject_to(h(...) >= (1-gamma_k)*h(...))` |
| D-CBF（本文创新） | 把障碍物的**位置和形状**也放进 `h`，于是 `h` 依赖预测的椭圆 | `h(curpos, ob)` 里的 `a,b,theta,cx,cy` |
| 椭圆距离公式 | 从椭圆中心沿某方向到边界的距离（论文式 6） | `h()` 内的二次型开方 |
| 安全距离 `d_safe` | `h = 距离 - 椭圆边界 - d_safe`，代码里 0.5 m | `safe_dist = 0.5` |
| 椭圆膨胀（Minkowski 和） | 用不确定性把椭圆放大 | `obs_kf.cpp` 里 `a = m_x(2) + pos_conf + ab_conf` |

需要的数学：多元微积分（李导数 `∇h·f`、`∇h·g`）、集合不变性、K∞ 类函数的直觉即可。

### 4.2 MPC（对应 `local_planner.py` 的目标与约束）

| 概念 | 在这里指什么 |
|---|---|
| 滚动时域 | 每 50 ms 解一次、只执行第一步（`input_sol[0]`） |
| 预测时域 | `N=25` 步 × `T=0.1 s` = 2.5 s |
| 代价函数 | 跟踪误差（`Q`，随步数增大）+ 控制量（`R`）+ 控制变化率（`S`）+ 终端代价 |
| 动力学约束 | 离散差速模型 `x_{k+1} = x_k + T·[v cosθ, v sinθ, ω]` |
| 输入约束 | `v∈[0.1,1.2]`（离终点 <1 m 时放开到 `[-0.1,1.2]`）、`ω∈[-1.2,1.2]` |
| 可行性/无解 | `except` 里打 `Infeasible Solution`，并用上一帧轨迹平移一格顶上 |

需要的数学：线性代数（二次型 `xᵀQx`）、最优化基础（约束优化、KKT 的直觉）、
数值优化器 IPOPT 是"内点法非线性求解器"，知道它会迭代到收敛即可。

### 4.3 优化建模（CasADi）

本项目用 CasADi 的 `Opti` 接口描述问题：`opti.variable()` 是决策变量，
`opti.subject_to()` 是约束，`opti.minimize()` 是目标，`opti.solver('ipopt')` 求解。
读代码时把符号变量当作"矩阵里的未知数"，逐行对照论文式 (8)(9) 即可。

需要知道：符号计算/自动微分的概念（为什么约束里可以写 `cos/sqrt`）、
IPOPT 的 `max_iter`/`acceptable_tol` 含义。

### 4.4 状态估计（`obs_param/`）

| 概念 | 在这里指什么 |
|---|---|
| 卡尔曼滤波 | 预测（用 A 推状态、P 推协方差）+ 更新（用观测和 R 修正） |
| 状态向量 | 9 维：`[x, y, ẋ, ẏ, ẍ, ÿ, a, b, θ]`（匀加速模型 + 椭圆形状） |
| 观测向量 | 5 维：`[x, y, a, b, θ]`（只能测到位置和形状） |
| 协方差 → 椭圆 | `0.95·sqrt(P_ii)` 当作不确定性半径，加到 `a,b` 上 |
| 数据关联 | 上游 `local_map` 用匈牙利算法给椭圆编号，KF 按编号分别滤波 |

需要的数学：概率论（高斯分布、协方差传播）、矩阵运算（`A P Aᵀ + Q`）。

### 4.5 点云与栅格（`local_map/`）

| 概念 | 在这里指什么 |
|---|---|
| PCL 滤波 | `PassThrough`（裁剪范围）、`VoxelGrid`（降采样）、`transformPointCloud`（转到世界系） |
| 2.5D 高程图 | `grid_map`：每格存一个高度值 |
| 边缘检测 | Sobel（坡度）、Laplace（凹凸）、邻域台阶差 → 三者阈值化判障碍 |
| DBSCAN | 基于密度的聚类，`R`（邻域半径 5 格）、`N`（最少点数 4） |
| 最小包围椭圆 | CGAL 的 `Min_ellipse_2`，把点簇压成一个 `(cx, cy, a, b, θ)` |

需要知道：点云的基本概念（无序点集、体素）、卷积核的直觉、聚类的基本思想。

### 4.6 几何与跟踪（`ellipse.hpp`、`KM.hpp`）

- 最小包围椭圆：为什么对共线点会退化（代码里有 fallback 分支）；
- 匈牙利算法：二分图最小代价匹配，用于"上一帧的 3 号椭圆对应这一帧的哪一个"；
- 跟踪标签的生命周期：**这是本项目最脆弱的一环**——每次重新检测可能产生新标签，
  新标签意味着新 KF（速度初值为 0），预测会滞后；复现结果里偶发的间距不足就源于此。

### 4.7 ROS 2 工程知识（读代码时的"工程噪声"）

- `rclpy` 的回调组与执行器：为什么本项目用 `SingleThreadedExecutor`（见 `port_notes.md` 2.5）；
- QoS：`/local_map_pub/gridmap` 是 transient local（latched），`/global_path` 是 volatile；
- `ros_gz_bridge` 的 YAML 配置：`ros_topic_name` 与 `gz_topic_name` 可以不同名；
- launch 的 `IfCondition/UnlessCondition`、`DeclareLaunchArgument`、参数传递。

### 4.8 建议的代码阅读顺序

1. `src/mpc_dcbf_bringup/launch/simulation.launch.py` —— 先建立"谁发布什么"的全局图；
2. `src/global_path_publisher/src/global_path_publisher.cpp` —— 30 行，热身；
3. `src/local_planner/local_planner/controller.py` —— 50 行，看清 `/local_plan → /cmd_vel`；
4. `src/mpc_scene/src/odom_tf.cpp` —— 明白 TF 是怎么来的（和出生位姿补偿）；
5. `src/local_map/src/local_map_pub.cpp` 的 `mainLoop()` —— 先只看主干调用顺序，
   细节（Sobel、DBSCAN）再逐个进去；
6. `src/obs_param/src/obs_kf.cpp` —— 对着 `kalman.h` 看预测/更新；
7. `src/local_planner/local_planner/local_planner.py` 的 `MPC_ellip()` —— 核心，
   拿论文式 (8)(9) 一行行对照；
8. 回到论文的 Fig. 2 / Fig. 4，用 RViz 把每一级画出来验证你的理解。

---

## 5. 编程与代码知识（读 / 改这份代码需要的）

这一章不讲算法，只讲**代码本身**：用了哪些库、每种语言在本项目里的固定写法、
消息怎么打包、要改东西该动哪几行、编译和调试怎么做。

### 5.1 语言与库清单

| 语言 / 库 | 用在哪 | 用来干什么 | 需要掌握到什么程度 |
|---|---|---|---|
| Python 3 + `rclpy` | `local_planner`（规划器、控制器） | 节点、回调、定时器、CasADi 建模 | 能写"类继承 Node + 订阅 + 发布 + 定时器" |
| `numpy` | 规划器 | 状态/代价矩阵、切片、`np.diag`、`np.argmin` | 数组形状与索引 |
| `CasADi`（`casadi`） | 规划器 | 符号优化建模 + 调 IPOPT | `Opti` 的五个 API 就够（见 5.2.5） |
| `threading` | 规划器 | 保护跨回调共享的状态 | `Lock` 的 acquire/release 与 `finally` |
| C++17 + `rclcpp` | `local_map`、`obs_param`、`mpc_scene`、`global_path_publisher` | 高频处理点云/滤波/预测 | 能读懂类、lambda、智能指针、模板调用 |
| `Eigen` | `obs_param`、`local_map` | 矩阵/向量/位姿运算 | 会写 `MatrixXf`、`Vector3d`、`Isometry3d` |
| `PCL` | `local_map` | 点云滤波、坐标变换、消息转换 | 三个滤波器的调用顺序 |
| `grid_map` | `local_map` | 2.5D 高程栅格 | 建立地图、写/读层、取世界坐标 |
| `CGAL` | `local_map` | 最小包围椭圆 | 会调用 `Min_ellipse_2` |
| `tf2` | `mpc_scene`、`local_map`、`controller` | 坐标变换查询与广播 | `lookupTransform` / `TransformBroadcaster` |
| `ros_gz_bridge` | `mpc_dcbf_bringup/config/bridge.yaml` | gz ↔ ROS 话题/类型映射 | YAML 五个字段的含义 |
| CMake + `ament_cmake` + `colcon` | 所有 C++ 包 | 构建 | 看得懂 `find_package` / `target_link_libraries` |

### 5.2 Python 侧（`src/local_planner/local_planner/`）

#### 5.2.1 一个 rclpy 节点的固定骨架

```python
class Controller(Node):
    def __init__(self):
        super().__init__('control')                       # ① 节点名
        self.N = 10
        self.local_plan_sub = self.create_subscription(   # ② 订阅
            Float32MultiArray, '/local_plan', self.local_planner_cb, 10)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)   # ③ 发布
        self.timer_localization = self.create_timer(0.01, self.get_current_state)  # ④ 定时器
        self.control_timer = self.create_timer(0.02, self.control_loop_tick)       # 50 Hz
```

四件事：`super().__init__` 给出节点名 → `create_subscription` / `create_publisher` 声明接口 →
`create_timer` 给周期 → 回调函数里干活。所有节点的结构都是这四段的排列组合。

#### 5.2.2 参数

```python
self.declare_parameter('replan_period', 0.05)              # 声明 + 默认值
self.replan_period = self.get_parameter('replan_period').value   # 取出来存成员
```

launch 里传参就是覆盖默认值：`parameters=[{'replan_period': 0.05, 'use_sim_time': True}]`。
**注意**：`use_sim_time` 是 ROS 2 内置参数，设成 `True` 后 `create_timer` 用的是仿真时间。

#### 5.2.3 执行器与回调组（本项目踩过的坑）

```python
executor = SingleThreadedExecutor()   # 串行执行所有回调
executor.add_node(node); executor.spin()
```

* `rclpy.spin(node)` = 单线程执行器；`MultiThreadedExecutor` 会让不同回调**并发**执行。
* 本项目的 MPC 单次求解 ≈20 ms，用多线程执行器时会被订阅回调抢 GIL，实测从 0.03 s 恶化到
  几十秒（详见 `port_notes.md` 2.5）。所以规划器用单线程执行器。
* `ReentrantCallbackGroup` / `MutuallyExclusiveCallbackGroup` 决定"同一组内的回调能否并发"。
  只有当你确实需要"求解时仍更新订阅"才用 reentrant。

#### 5.2.4 numpy 惯用法（本仓库出现的写法）

```python
self.local_plan = np.zeros([self.N, 2])        # 二维数组，[步数, (v, ω)]
self.local_plan[i, 0] = msg.data[2 * i]        # 行、列索引
Q = np.diag([1.0, 1.0, 0.02])                  # 对角矩阵
num = np.argmin(np.array([...]))               # 取最小值的下标
state_sol[i, 0]                                # 第 i 步的 x
```

**类型陷阱（必看）**：ROS 2 新的 rosidl 在把 Python 对象转成消息字段时用
`PyFloat_Check` / `PyBool_Check` 断言：

```python
cmd_move.data = distance > 0.1        # ✗ numpy.bool_ → 断言失败，进程直接 abort
cmd_move.data = bool(distance > 0.1)  # ✓
pose.orientation.x = 0                # ✗ Python int → 断言失败
pose.orientation.x = 0.0              # ✓
pt.x = float(state_sol[i, 0])         # ✓ 显式转 float 最稳妥
```

`np.float64` 是 `float` 的子类所以通常没事，但 `np.float32`、`np.bool_`、Python `int`
都会触发断言——这类崩溃没有 Python traceback，只有一行 C 断言，非常难查。

#### 5.2.5 CasADi 的编程模型（读 `MPC_ellip()` 的关键）

CasADi 里所有的量都先当**符号**写进一个优化问题，再交给求解器：

```python
opti = ca.Opti()                              # 一个空问题
opt_x0   = opti.parameter(3)                  # 参数：每次求解前用 set_value 填
opt_U    = opti.variable(N, 2)                # 决策变量：求解器来定
opti.subject_to(opti.bounded(0.1, opt_U[:, 0], 1.2))     # 约束
opti.minimize(ca.mtimes([x, Q, x.T]))                     # 目标
opti.solver('ipopt', {'ipopt.print_level': 0})            # 选求解器
opti.set_value(opt_x0, self.curr_state)                   # 填参数（数值）
sol = opti.solve()                                        # 求解
u = np.array(sol.value(opt_U))                            # 取数值解
```

对应到本仓库：`opt_x0` ← `/curr_state`，`opt_U` = 25 步的 `(v, ω)`，约束是差速动力学 +
输入界 + D-CBF，目标见 4.2 节。**读代码时把 `opt_states/opt_controls` 当成未知矩阵**即可。

`opti.solver()` 每次调用都会**重新构建**一份 NLP 并交给 IPOPT，所以上游是"每个控制周期
重建一次"，单次 ≈20–30 ms——这是它能跑到 18 Hz 的原因，也是为什么它不能被多个回调并发调用。

#### 5.2.6 锁

规划器里三个锁分别保护 `/curr_state`、`/global_path`、`/obs_predict_pub` 写入的缓存：

```python
self.curr_pose_lock.acquire()
try:
    ...读 self.curr_state...
finally:
    self.curr_pose_lock.release()     # 必须放 finally，否则异常后永久死锁
```

### 5.3 C++ 侧（`local_map` / `obs_param` / `mpc_scene`）

#### 5.3.1 一个 rclcpp 节点的骨架

```cpp
class LocalMapPub : public rclcpp::Node {
public:
  LocalMapPub() : Node("local_map_pub") {
    obs_height = declare_parameter<float>("obs_height", 0.4F);        // 参数
    for_obs_track_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/local_map_pub/for_obs_track", 1);                           // 发布
    velodyne_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points", 1,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {   // lambda 回调
          pcl::fromROSMsg(*msg, velodyne_cloud_);
        });
    loop_timer_ = create_wall_timer(std::chrono::milliseconds(91),     // 11 Hz 主循环
        std::bind(&LocalMapPub::mainLoop, this));
  }
};
```

要点：① 用 `declare_parameter<T>` 而不是 `nh.param`；② 回调里**只做拷贝/缓存**，重活放定时器；
③ `std::bind(&类::方法, this)` 或 `[this](...){}` 两种绑定方式本项目都有；④ 节点必须
`rclcpp::spin(std::make_shared<LocalMapPub>())` 才会开始收消息。

#### 5.3.2 指针与所有权

* `std::unique_ptr<T>`：独占，`tf_buffer_ = std::make_unique<tf2_ros::Buffer>(...)`；
* `std::shared_ptr<T>`：共享，消息回调收到的是 `Msg::SharedPtr`；
* PCL 的 `cloud.makeShared()`：把栈上的 `pcl::PointCloud` 包成 `Ptr` 交给滤波器——
  因为 PCL 的 `setInputCloud` 只收 `Ptr`。

#### 5.3.3 Eigen 常用写法（`local_map` 里到处都是）

```cpp
Eigen::Vector3d pos;
localmap_obs.getPosition3("elevation", pt, pos);        // 栅格索引 → 世界坐标
robot_position2d_ << tf.translation().x(), tf.translation().y();   // 逗号初始化
Eigen::Isometry3d lidar_tf_ = tf2::transformToEigen(lookupTransform(...));
pcl::transformPointCloud(in, out, lidar_tf_.cast<float>());        // double → float
```

#### 5.3.4 PCL 四步

```cpp
pcl::fromROSMsg(*msg, cloud);                 // ROS 消息 → 点云
pass_x_.setInputCloud(cloud.makeShared());    // 裁剪
pass_x_.filter(cloud);
sor_.filter(filtered);                        // VoxelGrid 降采样
pcl::toROSMsg(cloud, out_msg);                // 点云 → ROS 消息
```

#### 5.3.5 grid_map 与 CGAL

```cpp
map_.setGeometry(Length(14.0, 14.0), 0.1);    // 尺寸 + 分辨率
map_.add("elevation", matrix);                 // 写一层
auto m = map_.get("local_lidar");              // 读一层（矩阵）
```

```cpp
Min_ellipse me(pts.begin(), pts.end(), true);  // 点集 → 最小包围椭圆
me.ellipse().double_coefficients(r, s, t, u, v, w);  // 二次曲线系数
// 再由系数解出半轴 a,b、中心 cx,cy、旋转角 theta（ellipse.hpp 里那段公式）
```

CGAL 对**共线点**会返回退化椭圆（`is_degenerate()`），所以代码里有 fallback 分支。

#### 5.3.6 编译期常识

* 头文件里定义**函数/全局变量**会被多个 `.cpp` 包含 → 重复定义（ODR）。本项目 `ellipse.hpp`
  就是这么写的（上游如此，里面还有 `float localmap_x_size;` 这些全局量），所以只能有一个
  `.cpp` 包含它——目前正是 `local_map_pub.cpp`（它再包含 `KM.hpp`）；
* `const&` 传参避免拷贝（点云/矩阵很大）；
* 日志用 `RCLCPP_INFO(get_logger(), "...")`、`RCLCPP_WARN`，不要用 `printf`（`printf` 也能出来，但
  不会进 ROS 日志系统）；
* 头文件里 `#include <eigen3/Eigen/Eigen>`（本机 Eigen 装在 `eigen3/` 子目录下）。

### 5.4 消息打包约定（改代码前必须知道）

| 话题 | 类型 | 布局 |
|---|---|---|
| `/local_map_pub/for_obs_track` | `Float32MultiArray` | 每 **7** 个一组：`cx, cy, a, b, theta, label, variance` |
| `/obs_predict_pub` | `Float32MultiArray` | 每个障碍 **25 步 × 5** 个数：`cx, cy, a, b, theta` |
| `/local_plan` | `Float32MultiArray` | 每步 **2** 个：`v, ω`，共 N=25 步 |
| `/curr_state` | `Float32MultiArray` | `[x, y, yaw]` |
| `/obstacles/true_poses` | `Float32MultiArray` | 每障碍 2 个：`x, y`（评估用） |

所有话题名都是**硬编码字符串**，改名要全局搜索：`grep -rn "'/local_plan'\|\"/local_plan\"" src/`。

### 5.5 三个可以照抄的最小改法

**例 1：给 MPC 加一条走廊约束（`|y| ≤ 5`）**

在 `local_planner.py` 的 `MPC_ellip()` 里，动力学约束那段循环之后加：

```python
        for i in range(self.N):
            x_next = ... # 原有动力学
        # 新增：横向走廊约束
        opti.subject_to(opti.bounded(-5.0, opt_states[:, 1], 5.0))
```

**例 2：把安全距离做成参数**

```python
# __init__ 里
self.declare_parameter('safe_dist', 0.5)
self.safe_dist = self.get_parameter('safe_dist').value

# h() 里把常量替换成
safe_dist = self.safe_dist
```

然后 `ros2 launch ... --ros-args -p safe_dist:=0.8`，或在 launch 的 `planner` 节点
`parameters` 里加 `{'safe_dist': 0.8}`。

**例 3：新增一个订阅 `/local_plan` 的调试节点**

```python
# src/local_planner/local_planner/plan_logger.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class PlanLogger(Node):
    def __init__(self):
        super().__init__('plan_logger')
        self.create_subscription(Float32MultiArray, '/local_plan', self.cb, 10)

    def cb(self, msg):
        v, w = msg.data[0], msg.data[1]
        self.get_logger().info('v=%.2f w=%.2f' % (v, w))

def main():
    rclpy.init(); rclpy.spin(PlanLogger()); rclpy.shutdown()
```

再在 `src/local_planner/setup.py` 的 `console_scripts` 里加一行：

```python
'plan_logger = local_planner.plan_logger:main',
```

`colcon build --symlink-install --packages-select local_planner` 之后
`ros2 run local_planner plan_logger` 即可（`--symlink-install` 下新增 Python 文件也需要重新
install 一次，因为 `console_scripts` 是在安装阶段生成的）。

### 5.6 构建与调试的代码级知识

**colcon 的目录语义**

| 目录 | 内容 | 说明 |
|---|---|---|
| `src/` | 源码 | 你的改动都在这里 |
| `build/<包>/` | CMake 中间产物、Python 包的拷贝 | 报错信息里的路径常在这里 |
| `install/<包>/` | 安装结果 | `--symlink-install` 下多为符号链接指向 `build/` 或 `src/` |
| `log/` | 每次构建的日志 | `log/latest_build/<包>/stdout_stderr.log` |

改 Python/launch/YAML/SDF → 不用重编译；改 C++ → `colcon build --symlink-install --packages-select local_map`。

**读编译错误**

| 报错 | 含义 | 处理 |
|---|---|---|
| `Could not find a package configuration file provided by "Eigen3"` | 缺依赖 | `apt install libeigen3-dev` 或在 `package.xml` 加 `<depend>` |
| `undefined reference to ...` | 链接时缺库 | `CMakeLists.txt` 的 `target_link_libraries` |
| `ament_target_dependencies` 未定义 | 新版 ament 移除 | 见 `patches/grid_map_ros_ament_target_deps.patch` |
| `fatal error: grid_map_core/GridMap.hpp: No such file` | 没 source 工作空间 | `source install/setup.bash` |

**调试**

| 手段 | 命令 | 适用 |
|---|---|---|
| 话题频率/内容 | `ros2 topic hz /local_plan`、`ros2 topic echo /curr_state` | 任何"没反应" |
| QoS/连接关系 | `ros2 topic info /cmd_vel -v` | 有话题但收不到 |
| TF | `ros2 run tf2_ros tf2_echo world base_link` | 位姿相关 |
| Python 断点 | `python3 -m pdb`，或临时 `print(..., flush=True)` | 规划器逻辑 |
| 看 Python 卡在哪 | `py-spy dump --pid <pid>`（`pip install --user py-spy`；若报 Permission Denied 需 `sudo` 或放开 `kernel.yama.ptrace_scope`） | 卡死/极慢 |
| C++ 日志 | 代码里加 `RCLCPP_INFO(get_logger(), "x=%.2f", x)` | 中间量 |
| 录包复现 | `ros2 bag record /velodyne_points /local_plan /cmd_vel` | 复现偶发问题 |
| 进程残留 | `ps -eo cmd \| grep gz`、`bash scripts/stop.sh` | Gazebo 的日常 |

### 5.7 自测清单

能回答下面这些，读改本项目基本没有障碍：

1. 一个 rclpy 节点要成为节点，最少要写哪几行？`spin` 是谁调用的？
2. `create_subscription(类型, 话题, 回调, 10)` 里的 10 是什么？
3. 为什么 `cmd_move.data = a > b` 会让进程直接退出（没有 traceback）？
4. `ReentrantCallbackGroup` 和 `MutuallyExclusiveCallbackGroup` 的区别？本项目为什么用单线程执行器？
5. CasADi 里 `opti.parameter` 和 `opti.variable` 的区别？`set_value` 什么时候调用？
6. `sol.value(opt_controls)` 返回的是什么类型？为什么后面要 `np.array(...)`？
7. 为什么 `--symlink-install` 下改 Python 不用重编译，但新增 `console_scripts` 要重新 build？
8. `install/` 里的文件为什么是符号链接？对调试有什么好处？
9. `/obs_predict_pub` 里 500 个数怎么解读成一个障碍的 25 步预测？
10. `pcl::PointCloud::makeShared()` 为什么必要？
11. `Eigen::Isometry3d` 与 `Eigen::Matrix4f` 的关系？`cast<float>()` 在做什么？
12. 如果 `ros2 topic hz /local_plan` 有 18 Hz，但车不动，下一步查什么？

## 6. 学习路线与时间估计

| 目标 | 需要补 | 预计 |
|---|---|---|
| 把仿真跑起来看到车动 | 第 3 章 1、2 条 | 1–2 小时（含装依赖） |
| 会调参数、换场景、跑出指标图 | 第 3 章 2、3 条 + 第 2.6 节 | 半天 |
| 看懂并小改 Python 节点（规划器/控制器） | 第 5.2、5.4、5.5 节 + 例 1/2 | 1 天 |
| 看懂并小改 C++ 节点（local_map/obs_param） | 第 5.3、5.6 节 | 3–5 天 |
| 讲清楚算法在做什么 | 第 4 章 4.1–4.4 | 2–3 天（配合论文） |
| 能改算法（换约束、换预测、加障碍） | 第 4 章全部 | 1–2 周 |

---

## 7. 术语表

| 中文 | 英文 / 记号 | 一句话 |
|---|---|---|
| 控制屏障函数 | CBF, `h(x)` | 定义"安全集"的标量函数，用导数约束保证不越界 |
| 动态 CBF | D-CBF | 把移动障碍物的位置/形状并入 `h` 的 CBF |
| 模型预测控制 | MPC | 每步解一个有限时域最优控制问题，只执行第一步 |
| 最小包围椭圆 | MBE | 包住一堆点的最小面积椭圆，用来参数化障碍物 |
| 卡尔曼滤波 | KF | 用模型预测 + 观测修正来估计状态 |
| 前向不变 | forward invariant | 一旦在安全集内，就永远不出来 |
| 预测时域 | horizon, `N` | 往前看多少步（这里 25 步 = 2.5 s） |
| 差速驱动 | differential drive | 靠左右轮速差转向的底盘 |
| 体素降采样 | VoxelGrid | 把空间切成小立方体，每格只留一个点 |
| 二分图匹配 | Hungarian | 给两帧的椭圆做一一配对（跟踪） |
| 回调组 | callback group | 决定同一节点内回调能否并发执行 |
| 执行器 | executor | 决定由几个线程/以什么顺序派发回调 |
| 服务质量 | QoS | 可靠性/持久性/队列深度，收发两端必须兼容 |
| 符号变量 | symbolic variable | CasADi 里代表未知数的占位符，求解后才有数值 |
| 符号安装 | symlink install | `colcon --symlink-install`，源码直接以符号链接进 install |

---

有具体想深入的部分（比如"MPC 那段代码逐行讲"或"KF 的矩阵怎么来的"），
可以在 `docs/` 里继续补，或直接在 issue 里问。
