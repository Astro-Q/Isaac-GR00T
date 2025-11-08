# 仿真环境设置指南

本指南介绍如何设置Isaac Sim 5.0和Isaac Lab环境用于Unitree G1机器人仿真。

## 前置要求

1. **NVIDIA GPU**: 支持CUDA的NVIDIA GPU（推荐RTX 3090或更高）
2. **操作系统**: Ubuntu 20.04或22.04
3. **Isaac Sim 5.0**: 已安装Isaac Sim 5.0
4. **Isaac Lab**: 已安装Isaac Lab

## 安装步骤

### 1. 安装Isaac Sim 5.0

参考官方文档：https://docs.omniverse.nvidia.com/apps/isaacsim/latest/install_python.html

```bash
# 下载并安装Isaac Sim
# 注意：Isaac Sim需要单独安装，不在pip包中
```

### 2. 设置Isaac Sim Python环境

```bash
# 使用Isaac Sim自带的Python环境
cd /path/to/isaac-sim
./python.sh -m pip install omni-isaac-gym omni-isaac-lab
```

### 3. 配置环境变量

```bash
# 添加到 ~/.bashrc
export ISAAC_SIM_PATH=/path/to/isaac-sim
export PYTHONPATH=$ISAAC_SIM_PATH:$PYTHONPATH
```

### 4. 安装项目依赖

```bash
cd unitree_gr00t_integration
pip install -r requirements.txt
```

## 验证安装

运行测试脚本验证环境是否正确设置：

```bash
python -c "from omni.isaac.core import World; print('Isaac Sim installed successfully')"
```

## Unitree G1 URDF文件

需要准备Unitree G1的URDF文件。可以从以下来源获取：

1. **Unitree官方**: https://github.com/unitreerobotics
2. **自定义URDF**: 根据实际机器人配置创建

URDF文件应放置在：
```
unitree_gr00t_integration/assets/unitree_g1/urdf/unitree_g1.urdf
```

## 常见问题

### Q: 无法导入omni.isaac模块
A: 确保Isaac Sim的Python环境已正确设置，并且PYTHONPATH包含Isaac Sim路径。

### Q: GPU内存不足
A: 降低仿真分辨率或减少同时运行的实例数量。

### Q: URDF加载失败
A: 检查URDF文件路径是否正确，以及URDF文件格式是否有效。
