# Intpc_local_planner

## 一. 项目介绍

本项目是一个基于ROS2 Nav2框架的导航包，支持多种局部规划器的集成与切换。用户可以轻松添加新的局部规划器算法，并通过参数配置实现无缝切换。

## 二. 环境配置

当前开发环境为 Ubuntu22.04, ROS2 humble, Gazebo Classic 11.10.0

1. 克隆仓库

    ```sh
    git clone --recursive https://github.com/Xiancaijiang/Intpc_local_planner.git
    ```

2. 安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)

    ```sh
    sudo apt install cmake
    ```

    ```sh
    git clone https://github.com/Livox-SDK/Livox-SDK2.git
    cd ./Livox-SDK2/
    mkdir build
    cd build
    cmake .. && make -j
    sudo make install
    ```

3. 安装依赖

    ```sh
    cd Intpc_local_planner

    rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

4. 编译

    ```sh
    colcon build --symlink-install
    ```

## 三. 运行

### 3.1 可选参数

1. `world`:

    - 仿真模式
        - `RMUL` - [2024 Robomaster 3V3 场地](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22942&extra=page%3D1)
        - `RMUC` - [2024 Robomaster 7V7 场地](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22942&extra=page%3D1)

    - 真实环境
        - 自定，world 等价于 `.pcd(ICP使用的点云图)` 文件和 `.yaml(Nav使用的栅格地图)` 的名称

2. `mode`:
   - `mapping` - 边建图边导航
   - `nav` - 已知全局地图导航

3. `lio`:
   - `fastlio` - 使用 [Fast_LIO](https://github.com/LihanChen2004/FAST_LIO/tree/ROS2)，里程计约 10Hz
   - `pointlio` - 使用 [Point_LIO](https://github.com/LihanChen2004/Point-LIO/tree/RM2024_SMBU_auto_sentry)，可以输出100+Hz的Odometry，对导航更友好，但相对的，CPU占用会更高

4. `localization` (仅 `mode:=nav` 时本参数有效)
   - `slam_toolbox` - 使用 [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) localization 模式定位，动态场景中效果更好
   - `amcl` - 使用 [AMCL](https://navigation.ros.org/configuration/packages/configuring-amcl.html) 经典算法定位
   - `icp` - 使用 [icp_registration](https://github.com/baiyeweiguang/CSU-RM-Sentry/tree/main/src/rm_localization/icp_registration)，仅在第一次启动或者手动设置 /initialpose 时进行点云配准。获得初始位姿后只依赖 LIO 进行定位，没有回环检测，在长时间运行后可能会出现累积误差。

    Tips:
    1. 若使用 AMCL 算法定位时，启动后需要在 rviz2 中手动给定初始位姿。
    2. 若使用 slam_toolbox 定位，需要提供 .posegraph 地图，详见 [如何保存 .pgm 和 .posegraph 地图？](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/issues/I9427I)
    3. 若使用 ICP_Localization 定位，需要提供 .pcd 点云图

5. `lio_rviz`:
   - `True` - 可视化 FAST_LIO 或 Point_LIO 的点云图

6. `nav_rviz`:
   - `True` - 可视化 navigation2

7. `planner_type`:
   - `teb` - 使用 TEB (Timed Elastic Band) 局部规划器
   - `intpc` - 使用 IntPC (Integrated Planning and Control) 局部规划器

### 3.2 仿真模式示例

- 边建图边导航

    ```sh
    ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=mapping \
    lio:=fastlio \
    planner_type:=teb \
    lio_rviz:=False \
    nav_rviz:=True
    ```

- 已知全局地图导航

    ```sh
    ros2 launch rm_nav_bringup bringup_sim.launch.py \
    world:=RMUL \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    planner_type:=teb \
    lio_rviz:=False \
    nav_rviz:=True
    ```

### 3.3 真实模式示例

- 边建图边导航

    ```sh
    ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=YOUR_WORLD_NAME \
    mode:=mapping  \
    lio:=fastlio \
    planner_type:=teb \
    lio_rviz:=False \
    nav_rviz:=True
    ```

    Tips:

    1. 保存点云 pcd 文件：需先在 [fastlio_mid360.yaml](src/rm_nav_bringup/config/reality/fastlio_mid360_real.yaml) 中 将 `pcd_save_en` 改为 `true`，并设置 .pcd 文件的路径，运行时新开终端输入命令 `ros2 service call /map_save std_srvs/srv/Trigger`，即可保存点云文件。
    2. 保存地图：请参考 [如何保存 .pgm 和 .posegraph 地图？](https://gitee.com/SMBU-POLARBEAR/pb_rmsimulation/issues/I9427I)。地图名需要与 `YOUR_WORLD_NAME` 保持一致。

- 已知全局地图导航

    ```sh
    ros2 launch rm_nav_bringup bringup_real.launch.py \
    world:=YOUR_WORLD_NAME \
    mode:=nav \
    lio:=fastlio \
    localization:=slam_toolbox \
    planner_type:=teb \
    lio_rviz:=False \
    nav_rviz:=True
    ```

### 3.4 小工具 - 键盘控制

```sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 四. 如何加入新的 Local Planner

本导航包支持轻松集成新的局部规划器算法。以下是添加新局部规划器的详细步骤：

### 4.1 1. 准备规划器代码

将你的局部规划器代码添加到 `src/rm_navigation/` 目录下，确保它符合 Nav2 局部规划器的接口要求。

### 4.2 2. 创建配置文件

在 `src/rm_navigation/rm_navigation/config/` 目录下创建新的配置文件，命名为 `nav2_params_<planner_name>.yaml`。该文件应包含新规划器的所有配置参数。

示例配置文件结构：

```yaml
planner_server:
  ros__parameters:
    planner_plugin_types:
      - nav2_navfn_planner/NavfnPlanner
      - nav2_smac_planner/SmacPlannerLattice
    planner_plugins:
      - GridBased
      - SmacPlanner
    # GridBased 配置
    GridBased:
      plugin: nav2_navfn_planner/NavfnPlanner
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
    # SmacPlanner 配置
    SmacPlanner:
      plugin: nav2_smac_planner/SmacPlannerLattice
      tolerance: 0.5
      downsample_costmap: true
      downsampling_factor: 1

controller_server:
  ros__parameters:
    controller_plugins:
      - FollowPath
    # 新规划器的控制器配置
    FollowPath:
      plugin: <your_planner_package>/<your_planner_class>
      # 其他规划器特定参数
      param1: value1
      param2: value2
```

### 4.3 3. 修改 Launch 文件

需要修改 `src/rm_navigation/rm_navigation/launch/navigation_launch.py` 文件，以支持新的规划器：

1. 在文件开头的参数声明部分，为新规划器添加一个选项：

```python
declare_planner_type_cmd = DeclareLaunchArgument(
    'planner_type',
    default_value='teb',
    description='Choose local planner: teb, intpc or <your_planner_name>')
```

2. 在配置文件选择部分，添加新规划器的条件判断：

```python
if planner_type == 'teb':
    params_file = os.path.join(
        get_package_share_directory('rm_navigation'),
        'config',
        'nav2_params_teb.yaml')
elif planner_type == 'intpc':
    params_file = os.path.join(
        get_package_share_directory('rm_navigation'),
        'config',
        'nav2_params_intpc.yaml')
elif planner_type == '<your_planner_name>':
    params_file = os.path.join(
        get_package_share_directory('rm_navigation'),
        'config',
        'nav2_params_<your_planner_name>.yaml')
else:
    # 默认使用 teb 规划器
    params_file = os.path.join(
        get_package_share_directory('rm_navigation'),
        'config',
        'nav2_params_teb.yaml')
```

### 4.4 4. 更新 README

在 README.md 文件的可选参数部分，为 `planner_type` 添加新的选项说明：

```markdown
7. `planner_type`:
   - `teb` - 使用 TEB (Timed Elastic Band) 局部规划器
   - `intpc` - 使用 IntPC (Integrated Planning and Control) 局部规划器
   - `<your_planner_name>` - 使用 <your_planner_description> 局部规划器
```

### 4.5 5. 编译和测试

编译项目并使用新的规划器进行测试：

```sh
colcon build --symlink-install
ros2 launch rm_nav_bringup bringup_sim.launch.py \
world:=RMUL \
mode:=mapping \
lio:=fastlio \
planner_type:=<your_planner_name> \
lio_rviz:=False \
nav_rviz:=True
```

## 五. 实车适配关键参数

1. 雷达 ip

    本导航包已内置 [livox_ros_driver2](https://gitee.com/SMBU-POLARBEAR/livox_ros_driver2_humble)，可直接修改 [MID360_config.json](./src/rm_nav_bringup/config/reality/MID360_config.json) - `lidar_configs` - `ip`

2. 测量机器人底盘正中心到雷达的相对坐标

    x, y 距离比较重要，将影响云台旋转时解算到 base_link 的坐标准确性

    填入 [measurement_params_real.yaml](./src/rm_nav_bringup/config/reality/measurement_params_real.yaml)

    若雷达倾斜放置，无需在此处填入 rpy，而是将点云旋转角度填入 [MID360_config.json](./src/rm_nav_bringup/config/reality/MID360_config.json) - `extrinsic_parameter`

3. 测量雷达与地面的垂直距离

    此参数影响点云分割效果

    填入 [segmentation_real.yaml](./src/rm_nav_bringup/config/reality/segmentation_real.yaml) - `sensor_height`

4. nav2_params

    参数很多，比较重要的是 robot_radius 和 速度相关参数。详见 [nav2官方文档](https://docs.nav2.org/)

## 六. 项目结构

```
├── src/
│   ├── rm_driver/
│   ├── rm_localization/
│   ├── rm_nav_bringup/
│   ├── rm_navigation/
│   │   ├── Intpc_local_planner/
│   │   ├── costmap_converter/
│   │   ├── fake_vel_transform/
│   │   ├── rm_navigation/
│   │   │   ├── config/
│   │   │   │   ├── nav2_params_teb.yaml
│   │   │   │   └── nav2_params_intpc.yaml
│   │   │   └── launch/
│   │   │       └── navigation_launch.py
│   │   └── teb_local_planner/
│   ├── rm_perception/
│   └── rm_simulation/
├── README.md
└── README_old.md
```

## 七. 致谢
- Mid360 点云仿真：参考了 livox_laser_simulation、 livox_laser_simulation_RO2、 Issue15: CustomMsg。
- 原Navigation2算法框架基于 [中南大学 FYT 战队 RM 哨兵上位机算法](https://github.com/baiyeweiguang/CSU-RM-Sentry) 深圳北理莫斯科大学 北极熊战队 哨兵导航仿真/实车包(https://github.com/LihanChen2004/PB_RMSimulation.git)修改并适配
- 感谢所有为开源导航社区做出贡献的开发者们