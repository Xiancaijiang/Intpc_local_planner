#!/bin/bash

echo "开始克隆Intpc_local_planner仓库..."
git clone https://github.com/Xiancaijiang/Intpc_local_planner.git
cd Intpc_local_planner

echo "初始化所有子模块..."
git submodule init

echo "更新所有子模块..."
git submodule update

echo "更新嵌套子模块..."
git submodule foreach --recursive git submodule init
git submodule foreach --recursive git submodule update

echo "检查子模块状态..."
git submodule status

echo "克隆完成！"