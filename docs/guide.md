# 项目指南（GUIDE）

这份文档回答三个问题：

1. 这个仓库由什么组成，怎么把仿真实验跑起来；
2. 跑它需要哪些**前置软件知识**；
3. 要读懂里面的开源代码，需要补哪些**算法与工程知识**，以及按什么顺序读。

> 面向两类读者：想"跑一遍看结果"的（第 2 章），和想"改它/读懂它"的（第 3、4、5 章）。
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

## 5. 学习路线与时间估计

| 目标 | 需要补 | 预计 |
|---|---|---|
| 把仿真跑起来看到车动 | 第 3 章 1、2 条 | 1–2 小时（含装依赖） |
| 会调参数、换场景、跑出指标图 | 第 3 章 2、3 条 + 第 2.6 节 | 半天 |
| 讲清楚算法在做什么 | 第 4 章 4.1–4.4 | 2–3 天（配合论文） |
| 能改算法（换约束、换预测、加障碍） | 第 4 章全部 | 1–2 周 |

---

## 6. 术语表

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

---

有具体想深入的部分（比如"MPC 那段代码逐行讲"或"KF 的矩阵怎么来的"），
可以在 `docs/` 里继续补，或直接在 issue 里问。
