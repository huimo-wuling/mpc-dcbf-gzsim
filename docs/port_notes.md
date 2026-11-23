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

### ⚠️ 2.5 rclpy 多线程执行器把 0.03 s 的回调拖成几十秒

最初沿用 ROS 2 常见写法：`MultiThreadedExecutor` + `ReentrantCallbackGroup`。结果是
同一个 `MPC_ellip()` 在离线脚本里稳定 **0.030 s**，在节点里却从 0.2 s 一路涨到
**33 s**（实测 `CB mpc 27.264`、`mpc 33.698`），机器人因此一直执行过期指令、冲过终点。

原因不是 IPOPT：分段计时显示 `solve 0.02 s`、`constr 0.01–4.8 s`——变慢的是**回调的墙钟
时间**，即 Python 回调在多线程执行器里与大量订阅回调（`/curr_state` 50 Hz、`/global_path`
10 Hz×190 个路点）竞争 GIL/执行器锁而被反复抢占。

修复：规划器改用 **`SingleThreadedExecutor`**。这更接近原实现——`rospy.Timer` 的回调本身
就是串行的，而 rclpy 的多线程执行器是移植时"顺手"引入的。改回单线程后同一个回调稳定在
**0.032 s/周期（≈15–19 Hz）**。

### ⚠️ 2.6 `local_map` 的全局变量没有赋值 → 永远检测不到障碍物

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

### ⚠️ 2.7 `local_map` 话题命名空间错了 → KF 收不到障碍物

原节点用私有 `NodeHandle("~")`，因此发布的话题是 `/local_map_pub/for_obs_track`
（RViz 配置里也是 `/local_map_pub/gridmap`、`/local_map_pub/local_pcd`、
`/local_map_pub/ellipse_vis`）；而 `obs_kf` 订阅的是**绝对名**
`/local_map_pub/for_obs_track`。移植版发布到 `/for_obs_track`，名字对不上，
`obs_kf` 永远收不到数据 → 规划器 `self.ob` 永远为空 → **完全没有避障**。

修复：四个发布器改回 `/local_map_pub/...`。

### ⚠️ 2.8 CasADi / IPOPT 在 Ubuntu 26.04 上的坑

apt 的 `python3-casadi` (3.7.0) 是针对 numpy 1.x 编译的，与本机 numpy 2.3.5 不兼容：
连 `casadi.MX.sym('x')` 都会段错误。改用 PyPI 的 `casadi`（3.8.1，`abi3` wheel，
自带 IPOPT）：

```bash
pip3 install --user --break-system-packages casadi==3.8.1
```

### 2.9 有意保留的上游问题（为了保真，不做修改）

用户要求"只让原论文的实现能在本机跑起来，不改它本身的问题"，因此下面这些**上游自带**的
缺陷被原样保留：

* `obs_kf.cpp` 用固定 100 长度的数组按跟踪标签索引（`ObsKF _obs_kf[obs_kf_buffer_size]`）。
  `local_map` 的标签是单调递增的，长跑会越界——这是原实现的问题，未改动。
* `ellipse.hpp` 里 `resolution` 硬编码 0.05，而节点参数是 0.1（原仓库即如此）。
* `get_ellipse_array()` 的退化分支里 `Ellipse ellipse;` 的 `label/variance` 未初始化
  （唯一例外：移植版把这两个字段显式置 0，否则行为不可复现）。
* MPC 每个周期重新构建 CasADi 问题（原实现如此，实测 0.03 s/周期，不需要优化）。

### 2.10 可视化与"只能跑一次"的两个使用问题

这两条**不影响算法**，但直接影响能不能反复做实验：

* **RViz 看起来一团乱**：
  * `simulation.rviz` 里 `/global_path` 的 Durability 写成 `Transient Local`，而发布者是
    volatile → QoS 不兼容，**黄线全局路径永远不显示**（RViz 日志：
    `offering incompatible QoS ... DURABILITY_QOS_POLICY`）。已改成 `Volatile`。
  * 配置里原本**没有 `RobotModel`**，而 `jackal_tf.urdf` 只有 `<link>` 没有几何体 →
    RViz 里看不到机器人。现在 URDF 增加了与 `model.sdf` 一致的车体/四轮/雷达几何，
    并加上了 `RobotModel` 显示（`/robot_description`，transient local）。
  * `/obs_predict_vis_pub` 每帧发 **25 步 × 障碍物数** 个圆柱（4 个障碍就是 100 个），
    叠在屏幕上几乎不可辨。可视化改为每 5 步画一个截面（`obs_kf::visEllipse` 的
    `kVisStride`），`/obs_predict_pub` 的数据**未改动**。
  * 观感还包括：TF 显示默认关闭、点云尺寸调小、加了一个俯视视角。
* **只能复现一次**：`gz-sim-main` 经常在 `Ctrl+C` 后存活；点 Gazebo 窗口的 × 只会杀掉
  GUI 客户端，服务端继续持有 world，于是下一次启动报
  `Another world of the same name is running`。新增 `scripts/stop.sh` 做彻底清理，
  并用 `scripts/start.sh` 在启动前自动调用它，从而可以反复启动。

### 2.11 其它

* `mpc_scene/movetest.cpp`：从 `gazebo_ros` 的 `/gazebo/set_model_state` 服务改为
  gz-transport 请求 `/world/mpc_dcbf/set_pose`，100 Hz 遥操作语义不变（`<static>true</static>`
  的模型也能被 set_pose 移动）。
* `local_map` 的 `gridmap` 话题保留原实现的 **transient_local（latched）** QoS。
* `headless:=true` 原来并没有生效（`gz_sim.launch.py` 没有该参数），现在 headless 时
  用 `-s` 只启动服务端，否则启动 GUI。
* `scripts/import_repos.sh` 用纯 `git` 拉取 grid_map/filters（不依赖 `vcs`）。
* 规划器额外发布 `/local_planner/solve_time`（Float32），只用于记录求解耗时，不参与控制。

---

## 3. 行为保真核对（2026-11 复核）

复核方式：直接把 `~/catkin_ws/src/MPC-D-CBF` 的原文件与移植版逐项对照，再用实验量测
闭环行为。发现 **两个会改变机器人行为** 的移植偏差，均已修正；另有两个"环境相关、
未改上游逻辑"的差异，如实记录。

### 3.1 ⚠️ 差速底盘少了 `wheel_separation_multiplier: 1.5`（已修正）

原车控制链是 Gazebo Classic 的 `diff_drive_controller`，
`jackal_control/config/control.yaml` 里写着：

```yaml
wheel_separation_multiplier: 1.5   # 四轮滑移转向的侧滑补偿
```

几何轮距是 0.37559 m，乘 1.5 后控制器实际按 **0.563385 m** 换算左右轮速，用来补偿
四轮无法转向时的横向刮擦损失。移植时 `model.sdf` 直接写了 0.37559，于是：

| 指令 ω [rad/s] | 实测角速度（0.37559） | 实测角速度（0.563385） | 论文控制模型假设 |
|---|---|---|---|
| 0.5 | — | 0.525 | 0.5 |
| 1.0 | **0.615**（≈0.62×） | **1.003** | 1.0 |
| 2.0 | — | 1.794 | 2.0 |

也就是说，修正前机器人"只按指令的 2/3 转向"，而 MPC 的预测模型 `θ_{k+1}=θ_k+ω_kT`
假设 ω 被精确执行——闭环跟踪因此系统性变差。修正后工作区间内跟踪比 0.90–1.05。
量测脚本：`python3 scripts/record_rviz.py` 无关，方法是**纯转动实验**
（kill 掉 `controller`，直接发 `/cmd_vel` 只给 ω，用 gz 的
`/world/mpc_dcbf/dynamic_pose/info` 读真实偏航角，取稳态中位数）。

### 3.2 ⚠️ 机器人的位姿源换成了轮式里程计（已改回真值）

原车 `scene/src/getrobot_pose.cpp` 配合 Gazebo Classic 的 **`p3d` 插件**，拿到的是
世界真值位姿。移植版最初把 gz `DiffDrive` 的 `/model/jackal/odometry` 桥接过来当
`/base_pose_ground_truth`——那是**编码器积分**，四轮滑移转向每打滑一次就积累误差。
实测一次 60 s 实验结束：

* 轮式里程计：(8.90, 0.00)
* 仿真真值：(8.27, −1.13)  → **误差 1.3 m**

后果不是"差一点点"，而是规划器**以为自己已经到终点**（判据是距终点 < 1 m 时 v 下界
放到 0），于是发出 v=0 的规划、机器人半路停住。改法：

* `mpc_scene/src/odom_tf.cpp` 重写为用 gz-transport 订阅
  `/world/mpc_dcbf/dynamic_pose/info`，取 `jackal` 的位姿 → 广播 TF `world→base_link`
  → 同时发布 `/base_pose_ground_truth`（与 p3d 等价）；
* `bridge.yaml` 删掉 DiffDrive 里程计那条桥；
* `odom_tf` 加 `use_sim_time: True`（不然 TF 时间戳是墙钟，RViz 会报外推失败）。

修正后 `/curr_state`、`/base_pose_ground_truth`、gz 真值三者一致（8.900 / 8.900 / 8.899）。

### 3.3 激光雷达规格与原车不同（未改，已记录）

原车 `jackal_description/urdf/jackal.urdf.xacro` 用的是 velodyne HDL-32E：

```xml
<HDL-32E parent="base_link" name="velodyne" topic="/velodyne_points"
          hz="30" samples="440" gpu="false"/>
```

移植版是 gz `gpu_lidar`：**10 Hz、720 采样、0.3–70 m**，安装高度 0.5 m。
差异：方位分辨率更细（0.5° vs 0.82°），但刷新率只有 1/3；近端能看 0.3 m（原 0.7 m）。
`local_map` 本身以 11 Hz 消费最新一帧点云，所以实际影响主要体现在"障碍物运动更新延迟
≈0.1 s"上，未单独做变量实验。

### 3.4 障碍物横穿速度约为原车的 1/2（未改上游逻辑，已记录）

上游 `moving_cylinder.hpp` 的 `updateState()` 是**每个循环 tick 加一次增量**（不乘 dt）：

```cpp
model_state_.pose.position.x += twist_.linear.x;
model_state_.pose.position.y += twist_.linear.y;
```

所以"障碍物速度"= 增量 × 循环频率。原版每圈要做 8 次 `set_model_state` 服务调用，
Gazebo Classic 下约 12–14 ms/圈（≈70–80 Hz）；本机 gz-transport 的 `set_pose` 更慢，
实测 **40.7 Hz**，于是 `vx=-0.01/tick` 的 `cylinder_2` 实际速度是 **0.407 m/s**（原车约 0.7–0.8 m/s）。
障碍物慢一半 → 机器人从"绕过去"变成"贴着过"，这是最小间距 0.39–0.43 m 与论文 0.828 m
的主要差距来源。按要求没有改上游的逐 tick 逻辑，也没有为了指标去调障碍物速度。

### 3.5 修正后的三次重复

| 运行 | Min dist [m] | Cons time [s] | Speed var | 重规划 [Hz] | 求解均值 [ms] | 到点误差 [m] |
|---|---|---|---|---|---|---|
| #1 | 0.411 | 21.82 | 0.0213 | 18.4 | 20.1 | 0.10 |
| #2 | 0.387 | 22.78 | 0.1007 | 18.1 | 20.2 | 0.10 |
| #3 | 0.428 | 21.85 | 0.0196 | 18.4 | 19.6 | 0.10 |

修正前（同样的三次重复）：最小间距 0.529 / 0.072 / **−0.204**（两次接触），
其中两次跑到中途停住不再前进——正是 3.1 / 3.2 两个偏差造成的。
