# intpc_local_planner ROS2 Package

intpc_local_planner是一个基于ROS2 Nav2框架的高性能局部规划器插件，采用集成路径规划与控制策略，实现实时轨迹规划和障碍规避。

## 功能特点

- **集成规划与控制**：将路径生成与跟踪控制集成，减少模块间通信延迟
- **精确路径跟踪**：采用比例切向-法向控制策略实现高精度路径跟踪
- **智能障碍规避**：基于距离加权的避障算法，结合切向-法向向量控制
- **运动学约束**：考虑机器人物理限制，生成符合实际的控制命令
- **无缝Nav2集成**：作为标准Nav2控制器插件，易于集成到现有导航系统
- **可视化支持**：提供调试和演示用的实时可视化标记

## 算法原理

intpc_local_planner的核心算法由以下组件构成：

### 1. 参考轨迹生成

采用直线路径跟踪策略，通过固定前瞻距离（50cm）计算参考轨迹点，简化了路径生成过程，提高了算法实时性和稳定性。

### 2. 参考速度计算

基于机器人当前位置和路径信息，计算参考速度向量：

```
v_ref = v_d * τ + k * e * n
```

其中：
- `v_d`：期望速度
- `τ`：路径切向单位向量
- `k`：比例增益
- `e`：路径跟踪误差
- `n`：路径法向单位向量

### 3. 障碍规避策略

实现基于距离加权的避障算法：

1. 检测最近障碍物并计算避障方向向量
2. 根据距离障碍物的距离动态调整避障权重
3. 结合参考速度和避障速度生成最终控制命令

### 4. 控制命令转换

将笛卡尔空间控制速度转换为机器人线速度和角速度命令：
- 线速度：控制向量大小，受最大速度限制
- 角速度：基于期望航向角和当前航向角误差计算

## 代码框架

### 核心类结构

#### IntpcLocalPlanner类

核心规划算法实现类：

```cpp
class IntpcLocalPlanner {
public:
  // 构造函数和析构函数
  IntpcLocalPlanner(double max_vel, double acc_lim);
  ~IntpcLocalPlanner();
  
  // 路径参数初始化（目标点坐标）
  void initializePathParams(double goal_x, double goal_y);
  
  // 参考速度计算
  void reference(double k, double x, double y, double vd, double& e, Eigen::Vector2d& vf);
  
  // 前向运动学计算
  void forwardKinematics(double x, double y, double theta, const Eigen::Vector2d& u,
                         double dt, double l, double& x_new, double& y_new, double& theta_new);
  
  // 主要控制计算接口
  Eigen::Vector2d computeIntpcControl(double x, double y, double theta,
                                     const std::vector<double>& x_obstacles,
                                     const std::vector<double>& y_obstacles,
                                     const std::vector<double>& obstacle_radii,
                                     double k_gain, double max_speed);
  
private:
  // 机器人参数
  double wheel_base_;
  
  // 控制器参数
  double alpha_;
  double l_;
  double d_;
  double max_velocity_;
  double acceleration_limit_;
  
  // 路径参数
  double goal_x_;
  double goal_y_;
  double vd_;
};
```

#### IntpcLocalPlannerROS类

Nav2控制器插件实现类：

```cpp
class IntpcLocalPlannerROS : public nav2_core::Controller {
public:
  // 构造函数和析构函数
  IntpcLocalPlannerROS();
  ~IntpcLocalPlannerROS();
  
  // 框架生命周期管理
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
                 std::string name,
                 std::shared_ptr<tf2_ros::Buffer> tf,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;
  
  // 设置全局路径
  void setPlan(const nav_msgs::msg::Path & orig_global_plan) override;
  
  // 计算速度命令
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker * goal_checker);
  
protected:
  // 辅助方法
  nav_msgs::msg::Path transformGlobalPlan(const nav_msgs::msg::Path &global_plan,
                                         const std::string &global_frame,
                                         const geometry_msgs::msg::PoseStamped &robot_pose);
  
  // 获取本地目标点（固定50cm前瞻距离）
  geometry_msgs::msg::PoseStamped getLocalGoal(const nav_msgs::msg::Path &transformed_plan);
  
  // 速度限制
  void saturateVelocity(geometry_msgs::msg::Twist &twist);
  
  // 从代价图提取障碍物
  void extractObstaclesFromCostmap(
    const geometry_msgs::msg::PoseStamped &pose,
    std::vector<double> &x_obstacles,
    std::vector<double> &y_obstacles,
    std::vector<double> &obstacle_radii);
  
  // 成员变量
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  
  TFBufferPtr tf_;
  CostmapROSPtr costmap_ros_;
  
  nav_msgs::msg::Path global_plan_;
  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  
  // 运动参数
  double max_vel_x_;
  double max_vel_x_backwards_;
  double max_vel_theta_;
  double acc_lim_x_;
  double acc_lim_theta_;
  
  // 核心规划器
  std::unique_ptr<IntpcLocalPlanner> planner_;
  
  // 控制器参数
  double k_gain_;
  double obstacle_radius_;
  double robot_radius_;
  
  // 可视化
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};
```
```

### 文件结构

```
Intpc_local_planner/
├── CMakeLists.txt                 # CMake构建文件
├── include/
│   └── intpc_local_planner/       # 头文件目录
│       ├── intpc_local_planner.h  # 核心规划器头文件
│       └── intpc_local_planner_ros.h  # ROS接口头文件
├── intpc_local_planner_plugin.xml  # 插件描述文件
├── package.xml                    # 包信息文件
├── scripts/                       # 辅助测试脚本（仅用于独立测试，导航系统不依赖）
│   ├── cbf.py                     # 控制屏障函数实现
│   ├── detect.py                  # 障碍物检测
│   ├── fourier.py                 # 傅里叶路径生成（历史遗留，当前版本不使用）
│   ├── kinematics.py              # 运动学计算
│   ├── main.py                    # 主测试脚本
│   └── obstacle.py                # 障碍物表示
└── src/
    ├── intpc_local_planner.cpp    # 核心规划器实现
    └── intpc_local_planner_ros.cpp  # ROS接口实现
```

## 配置参数

| 参数类别 | 参数名 | 类型 | 说明 | 默认值 |
|---------|--------|------|------|--------|
| **运动限制** | `max_vel_x` | double | 最大前进线速度 (m/s) | 1.0 |
| **运动限制** | `max_vel_x_backwards` | double | 最大后退线速度 (m/s) | 0.5 |
| **运动限制** | `max_vel_theta` | double | 最大角速度 (rad/s) | 1.5 |
| **运动限制** | `acc_lim_x` | double | 线加速度限制 (m/s²) | 2.0 |
| **运动限制** | `acc_lim_theta` | double | 角加速度限制 (rad/s²) | 3.0 |
| **规划器参数** | `k_gain` | double | 路径跟踪比例增益 | 1.0 |
| **规划器参数** | `obstacle_radius` | double | 障碍物检测半径 (m) | 0.25 |
| **规划器参数** | `robot_radius` | double | 机器人碰撞半径 (m) | 0.2 |
| **控制参数** | `lookahead_distance` | double | 路径前瞻距离 (固定为0.5m) | - |

## 使用方法

### 1. 集成到Navigation2

将intpc_local_planner配置为Nav2控制器插件：

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    controller_plugins:
      - FollowPath
    FollowPath:
      plugin: intpc_local_planner/IntpcLocalPlannerROS
      max_vel_x: 0.5
      max_vel_x_backwards: 0.3
      max_vel_theta: 1.0
      acc_lim_x: 1.0
      acc_lim_theta: 1.0
      k_gain: 1.0
      obstacle_radius: 0.2
      robot_radius: 0.15
```

### 2. 启动导航系统

```bash
# 启动导航系统
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True map:=my_map.yaml

# 或使用项目提供的启动文件
ros2 launch rm_nav_bringup bringup_sim.launch.py planner_type:=intpc
```

## 依赖项

- **ROS2 Humble** 或更高版本
- **Navigation2** 导航框架
- **Eigen3** 矩阵运算库
- **tf2** 坐标变换库
- **nav2_costmap_2d** 障碍物检测

## 安装方法

### 1. 克隆仓库

```bash
git clone https://github.com/Xiancaijiang/Intpc_local_planner.git
```

### 2. 安装依赖

```bash
cd Intpc_local_planner
rosdep install --from-paths src --ignore-src -r -y
```

### 3. 构建包

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select intpc_local_planner
```

## 更新说明

### 版本1.1 (当前版本)

- **移除傅里叶路径依赖**：简化为直线路径跟踪，不再使用傅里叶级数生成路径
- **固定前瞻距离**：采用50cm固定前瞻距离，提高算法稳定性
- **优化接口设计**：简化`initializePathParams`和`reference`接口
- **移除冗余参数**：删除`path_shape_`和`lookahead_dist_`等不再使用的参数
- **提升实时性能**：优化控制算法，减少计算复杂度

### 版本1.0

- 初始版本发布
- 支持傅里叶路径生成
- 基本的路径跟踪和障碍规避功能

## 联系方式

如有问题或建议，请通过GitHub Issues提交。

## 许可证

本项目采用Apache 2.0许可证，详见LICENSE文件。