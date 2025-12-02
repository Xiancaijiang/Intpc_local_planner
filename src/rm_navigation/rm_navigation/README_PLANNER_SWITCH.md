# TEB与Intpc规划器切换功能说明

本文档详细介绍如何在ROS 2 Navigation2系统中切换TEB和Intpc本地规划器。

## 功能概述

通过修改后的启动文件，您可以在启动时通过参数选择使用TEB规划器或Intpc规划器，实现完全隔离的运行环境。这种方式确保：

1. 规划器之间互不影响
2. 可根据需要选择不同的规划器进行测试和使用
3. 保留了原始系统的稳定性

## 实现原理

1. 在`navigation_launch.py`和`bringup_rm_navigation.py`中添加了`planner_type`启动参数
2. 创建了两个独立的配置文件：
   - `nav2_params_teb.yaml`：仅包含TEB规划器配置
   - `nav2_params_intpc.yaml`：仅包含Intpc规划器配置
3. 启动文件根据`planner_type`参数动态选择对应的配置文件

## 使用方法

### 基本使用

通过`planner_type`参数选择要使用的规划器：

```bash
# 使用TEB规划器（默认）
ros2 launch rm_nav_bringup bringup_rm_navigation.py planner_type:=teb

# 使用Intpc规划器
ros2 launch rm_nav_bringup bringup_rm_navigation.py planner_type:=intpc
```

### 完整启动命令示例

包含地图和其他参数的完整启动命令：

```bash
# 使用TEB规划器启动导航系统
ros2 launch rm_nav_bringup bringup_rm_navigation.py planner_type:=teb map:=/path/to/your/map.yaml use_sim_time:=false

# 使用Intpc规划器启动导航系统
ros2 launch rm_nav_bringup bringup_rm_navigation.py planner_type:=intpc map:=/path/to/your/map.yaml use_sim_time:=false
```

### 高级使用：指定自定义配置文件

如果需要使用自定义配置文件，仍然可以通过`params_file`参数指定：

```bash
# 使用自定义配置文件启动（将忽略planner_type参数）
ros2 launch rm_nav_bringup bringup_rm_navigation.py params_file:=/path/to/your/custom_params.yaml
```

## 配置文件说明

### TEB规划器配置

位于：`rm_navigation/params/nav2_params_teb.yaml`

- 仅包含`controller_plugins: ["FollowPath"]`
- `FollowPath`配置为`teb_local_planner::TebLocalPlannerROS`
- 包含完整的TEB规划器参数配置

### Intpc规划器配置

位于：`rm_navigation/params/nav2_params_intpc.yaml`

- 仅包含`controller_plugins: ["IntpcController"]`
- `IntpcController`配置为`intpc_local_planner::IntpcLocalPlannerROS`
- 包含完整的Intpc规划器参数配置

## 注意事项

1. **隔离运行**：两个规划器完全隔离运行，不会互相影响
2. **默认行为**：如果不指定`planner_type`参数，默认使用TEB规划器
3. **优先级**：如果同时指定`planner_type`和`params_file`参数，系统将优先使用`params_file`指定的配置文件
4. **参数修改**：如果需要调整规划器参数，请直接编辑对应的配置文件
5. **系统稳定性**：由于两个规划器完全隔离，Intpc系统不会影响原navigation2系统的正常运行

## 故障排除

1. **无法启动规划器**：检查配置文件中的插件名称是否正确，以及相关包是否已安装
2. **导航异常**：尝试调整对应规划器的参数，如`obstacle_inflation`、`lookahead_dist`等
3. **切换失败**：检查启动命令中的参数拼写是否正确，确保`planner_type`的值为`teb`或`intpc`

## 性能建议

1. TEB规划器适合复杂环境下的动态避障
2. Intpc规划器可根据实际测试情况调整参数以获得最佳性能
3. 在不同场景下可以通过切换规划器进行性能对比和优化