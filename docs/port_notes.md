# 移植与修复笔记（ROS 1 + Gazebo Classic → ROS 2 + Gazebo Sim）

本项目是 [jianzhuozhuTHU/MPC-D-CBF](https://github.com/jianzhuozhuTHU/MPC-D-CBF)
在 ROS 2 (Lyrical) + Gazebo Sim 10 上的移植版。原仓库是 ICRA 2023 论文
*Dynamic Control Barrier Function-based Model Predictive Control to
Safety-Critical Obstacle-Avoidance of Mobile Robot* 的官方代码，运行环境为
Ubuntu 20.04 + ROS Noetic + Gazebo Classic 11（catkin / rospy / `gazebo_ros_control`）。

本文档记录**为了能在本机原生运行并跑通闭环**必须解决的所有问题。第 1 节是
"迁移层面"的改动（不可避免），第 2 节是移植过程中引入或暴露出来的**实打实的 bug**，
其中标注 ⚠️ 的几个会导致"仿真跑起来但机器人不动 / 不避障"这类现象。

---

## 1. 迁移层面的改动（框架差异）

| 原实现（ROS 1 + Gazebo Classic） | 本移植（ROS 2 + Gazebo Sim 10） |
|---|---|
| catkin 包 `scene` / `jackal` / `local_map` / `obs_param` / `local_planner` / `global_path_publisher` | 同名包（`scene` → `mpc_scene`），colcon/ament 构建 |
| `rospy` 节点（planner、controller） | `rclpy`，`MultiThreadedExecutor`、锁与原实现一致 |
| `ros.h` C++ 节点 | `rclcpp` |
| `/gazebo/set_model_state` 服务移动障碍物（100 Hz teleport） | gz-transport 调用 `/world/mpc_dcbf/set_pose`，语义不变；另外发布 `/obstacles/true_poses` 供评估 |
| `gazebo_ros_control` 差速驱动、`p3d` 真值里程计、Velodyne 插件、hector IMU/GPS | gz-sim 原生 `DiffDrive` + `gpu_lidar`（HDL-32E 级：32 线、-30.67°~+10.67°、10 Hz、720 方位采样、z=0.5 m 安装），通过 `ros_gz_bridge` 映射回原来的 ROS 话题名 |
| `world.world`（ODE 物理） | `mpc_dcbf_world.sdf`（DART），地面 + 完全相同的 8 个圆柱（r=0.25 m、h=0.5 m、相同初始位姿与速度） |
| `grid_map`（apt 包） | 通过 `mpc_dcbf.repos` 拉取 ANYbotics/grid_map 的 `ros2` 分支并打补丁 |
| 未使用的 `grid_map_octomap` 依赖 | 删除 |

话题名、节点名、参数、频率（LiDAR 10 Hz、局部地图 11 Hz、重规划 20 Hz、控制器 50 Hz）
与算法常数保持一致，便于与原实现对照。物理引擎不同（DART vs ODE），因此轨迹是
"等价"而非逐位相同。

### 1.1 补丁与构建细节

* 新版 `ament_cmake` 移除了 `ament_target_dependencies`，而 grid_map `ros2` 分支仍在用。
  `patches/grid_map_ros_ament_target_deps.patch`（由 `scripts/import_repos.sh` 自动应用）
  把 `grid_map_ros` 改成 `target_link_libraries` + 导出目标。
* grid_map 中未用到的子包（octomap / sdf / costmap_2d / visualization / filters /
  loader / pcl / rviz_plugin / demos）放置 `COLCON_IGNORE`，避免引入无关依赖。

---

## 2. 移植过程中引入 / 暴露的 bug ⚠️

### ⚠️ 2.1 `/cmd_vel` 没有订阅者，机器人原地不动

gz-sim 的 `DiffDrive` 系统默认订阅**带模型作用域**的话题（`/model/jackal/cmd_vel`），
而控制器发布的是 `/cmd_vel`。必须显式写出：

```xml
<plugin filename="gz-sim-diff-drive-system" ...>
  <topic>/cmd_vel</topic>
```

### ⚠️ 2.2 激光雷达 / 里程计 / 关节状态根本没到 ROS

gz-sim 的 `gpu_lidar` 在 `<topic>` 上发布 `LaserScan`，点云发布在 `<topic>/points`
(`PointCloudPacked`)；`DiffDrive` 的里程计在 `/model/<name>/odometry`；
关节状态在 `/world/<world>/joint_state`。原来的桥接参数用的是 `gz.msgs.PointCloudPacked`
但话题名写成 `/velodyne_points`（类型不匹配，收不到数据），里程计桥接的
`/base_pose_ground_truth` 在 gz 侧也根本不存在。

现在统一用 `config/bridge.yaml`（`ros_gz_bridge` 的 `config_file`），可以给两侧起
不同的名字：

```yaml
- ros_topic_name: "/velodyne_points"
  gz_topic_name: "/velodyne_points/points"
  ...
- ros_topic_name: "/base_pose_ground_truth"
  gz_topic_name: "/model/jackal/odometry"
```

### ⚠️ 2.3 里程计不是"世界真值"

Gazebo Classic 的 `p3d` 插件发布**绝对世界位姿**；gz-sim 的 `DiffDrive` 里程计是从
**出生位姿**开始积分的（出生在 (-10, 0) 时它报 (0, 0)）。规划器需要世界坐标，因此
`odom_tf` 节点把出生位姿复合回去（`spawn_x/spawn_y/spawn_yaw` 参数，默认 -10/0/0）。

### ⚠️ 2.4 ROS 2 消息字段类型断言导致 Python 节点直接 abort

新版 rosidl 的 Python 转换用 `PyBool_Check` / `PyFloat_Check` 做断言：

```
std_msgs__msg__bool__convert_from_py: Assertion `PyBool_Check(field)' failed
geometry_msgs__msg__quaternion__convert_from_py: Assertion `PyFloat_Check(field)' failed
std_msgs__msg__color_rgba__convert_from_py: Assertion `PyFloat_Check(field)' failed
```

原代码里 `cmd_move.data = a > b`（`numpy.bool_`）、`orientation.x = 0`、
`color.r = 1`（Python int）都会触发 **进程 abort（exit -6）**。ROS 1 下是隐式转换，
旧版 ROS 2 也宽松。修复：显式 `bool(...)` / `0.0` / `1.0` / `float(...)`。

### ⚠️ 2.5 `ReentrantCallbackGroup` 让 MPC 求解互相叠加

最初把重规划定时器和订阅都放进 `ReentrantCallbackGroup` 并配 `MultiThreadedExecutor`。
在 `rclpy` 中，reentrant 组的定时器回调**可以并发执行**：一轮 `opti.solve()` 还没结束，
下一轮已经开始，线程池被 20 Hz 的回调迅速填满，单次求解耗时从 0.08 s 恶化到 30 s 量级
（实测 `DT goal 34 s / mpc 34 s`），控制器只能一直执行过期指令，机器人冲过终点。
ROS 1 的 `rospy.Timer` 是串行回调，不会有这个问题。

修复：**订阅**留在 reentrant 组（求解期间仍能刷新 `/curr_state`、`/obs_predict_pub`），
**重规划定时器**放进 `MutuallyExclusiveCallbackGroup`，恢复原实现的串行语义。

### ⚠️ 2.6 每周期重建 CasADi 问题：延迟抖动与卡死

原 `MPC_ellip()` 每个周期都 `ca.Opti()` 重新搭建 NLP。在 ROS 2 节点里，符号构建阶段
（`ca.vertcat`/`subject_to`）会随运行时间从 0.05 s 抖动到数秒（实测 `constr 4.8 s`），
一旦错过 deadline 就会连锁恶化。

修复：**一次性构建参数化 NLP**，之后只更新参数并用上一解热启动：

| 参数 | 含义 | 维度 |
|---|---|---|
| `p_x0` | 当前状态 | 3 |
| `p_vlb` | 速度下界（`v_min` / `-v_min` 切换） | 1 |
| `p_goal` | 参考轨迹 | N×3 |
| `p_obs` | 预测椭圆（`MAX_OBS*N` 行，每行 `x, y, a, b, theta`） | 125×5 |

代价函数、动力学、输入约束、D-CBF 约束（式 5–7）与原实现完全一致。等价性验证：
对同一状态/同一组障碍物，新旧实现在有障碍物时 `max|Δu| = 0.0000`、`max|Δx| = 0.0000`
（无约束退化情形差 0.018，属求解器容差）。平均求解 30 ms（p95 92 ms）。

不活跃的障碍物槽位被"停"到 60 m 以外，其 CBF 约束自然满足，从而不需要为障碍物
数量重建问题结构（论文设定 n = 5，代码里 `MAX_OBS = 5` 与之一致）。

### ⚠️ 2.7 `local_map` 的全局变量没有赋值 → 永远检测不到障碍物

`local_map/include/ellipse.hpp` 里有文件作用域全局变量：

```cpp
float localmap_x_size, localmap_y_size;
float resolution = 0.05, _inv_resolution = 1.0 / resolution;
```

原实现用 `ros::NodeHandle nh("~"); nh.param<float>("localmap_x_size", localmap_x_size, 10);`
**直接写进这些全局变量**。移植时改成了同名**成员变量**，把全局变量遮蔽了，全局量始终为 0。
于是 `get_ellipse_array()` 里的 `if (pt(0) >= localmap_x_size / resolution ...) continue;`
对所有点成立，`pts` 永远为空 → 拟合退化 → 发布空椭圆列表 → **感知链路全线失效**。

修复：构造时显式镜像 `::localmap_x_size = localmap_x_size;` 等四个全局量。

### ⚠️ 2.8 `local_map` 话题命名空间错了 → KF 收不到障碍物

原节点用私有 `NodeHandle("~")`，因此发布的话题是 `/local_map_pub/for_obs_track`
（RViz 配置里也是 `/local_map_pub/gridmap`、`/local_map_pub/local_pcd`、
`/local_map_pub/ellipse_vis`）；而 `obs_kf` 订阅的是**绝对名**
`/local_map_pub/for_obs_track`。移植版发布到 `/for_obs_track`，名字对不上，
`obs_kf` 永远收不到数据 → 规划器 `self.ob` 永远为空 → **完全没有避障**。

修复：四个发布器改回 `/local_map_pub/...`。

### ⚠️ 2.9 `obs_kf` 用固定数组按跟踪标签索引 → 越界崩溃

```cpp
const size_t obs_kf_buffer_size = 100;
obs_kf _obs_kf[obs_kf_buffer_size];      // 原实现
...
_obs_kf[flag].param_list.push_back(_obs_tmp);
```

`flag` 来自 `local_map` 的跟踪标签，而标签在每次重新检测时会**单调增长**，长时间运行
必然超过 100 → 越界写 → SIGSEGV（实测 `obs_kf exit code -11`，避障随即失效）。
修复：改成 `std::map<int, std::unique_ptr<ObsKF>>`，超过 100 个标签时淘汰最旧的，
同时限制单个障碍物的观测队列长度。

### 2.10 CasADi / IPOPT 在 Ubuntu 26.04 上的坑

apt 的 `python3-casadi` (3.7.0) 是针对 numpy 1.x 编译的，与本机 numpy 2.3.5 不兼容：
连 `casadi.MX.sym('x')` 都会段错误。改用 PyPI 的 `casadi`（3.8.1，`abi3` wheel，
自带 IPOPT）：

```bash
pip3 install --user --break-system-packages casadi==3.8.1
```

### 2.11 其它

* `mpc_scene/movetest.cpp`：从 `gazebo_ros` 的 `/gazebo/set_model_state` 服务改为
  gz-transport 请求 `/world/mpc_dcbf/set_pose`，100 Hz 遥操作语义不变（`<static>true</static>`
  的模型也能被 set_pose 移动）。
* `local_map` 的 `gridmap` 话题保留原实现的 **transient_local（latched）** QoS。
* `headless:=true` 原来并没有生效（`gz_sim.launch.py` 没有该参数），现在 headless 时
  用 `-s` 只启动服务端，否则启动 GUI。
* `scripts/import_repos.sh` 用纯 `git` 拉取 grid_map/filters（不依赖 `vcs`）。
