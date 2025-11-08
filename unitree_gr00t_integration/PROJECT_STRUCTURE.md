# Unitree G1 + GR00T-N1.5 Integration Project Structure

unitree_gr00t_integration/
├── README.md                          # 主README文件
├── setup.py                           # 安装脚本
├── requirements.txt                   # Python依赖
│
├── isaac_sim/                         # Isaac Sim仿真模块
│   ├── __init__.py
│   ├── configs/                       # 仿真配置文件
│   │   └── unitree_g1_config.yaml
│   ├── scenes/                        # 仿真场景
│   │   ├── __init__.py
│   │   └── pick_and_place_scene.py
│   └── tasks/                         # 任务定义
│       ├── __init__.py
│       └── pick_and_place_task.py
│
├── data_collection/                   # 数据采集模块
│   ├── __init__.py
│   ├── lerobot_collector.py          # LeRobot数据采集器
│   └── teleop/                        # 遥操作接口（待实现）
│       └── __init__.py
│
├── training/                          # 模型训练模块
│   ├── __init__.py
│   ├── configs/                       # 训练配置
│   │   └── unitree_g1_training_config.yaml
│   └── finetune.py                    # 微调脚本
│
├── deployment/                        # 部署模块
│   ├── __init__.py
│   ├── sim_deploy.py                 # 仿真部署
│   └── real_robot_deploy.py          # 真实机器人部署
│
├── examples/                          # 示例脚本
│   ├── collect_data.py               # 数据采集示例
│   ├── train_model.py                # 训练示例
│   └── deploy_policy.py              # 部署示例
│
├── docs/                              # 文档
│   ├── simulation_setup.md           # 仿真环境设置
│   ├── data_collection.md            # 数据采集指南
│   ├── training.md                   # 训练指南
│   └── deployment.md                 # 部署指南
│
└── assets/                            # 资源文件（需要用户添加）
    └── unitree_g1/
        └── urdf/
            └── unitree_g1.urdf       # Unitree G1 URDF文件

## 集成说明

本项目整合了以下三个仓库的功能：

1. **Isaac-GR00T** (https://github.com/Astro-Q/Isaac-GR00T)
   - GR00T-N1.5模型和训练代码
   - 数据加载和处理
   - 推理服务

2. **unitree_sim_isaaclab** (https://github.com/Astro-Q/unitree_sim_isaaclab)
   - Unitree G1的Isaac Sim仿真
   - 机器人URDF和场景配置

3. **unitree_IL_lerobot** (https://github.com/Astro-Q/unitree_IL_lerobot)
   - LeRobot格式数据采集
   - 遥操作接口

## 使用流程

1. **环境设置**: 安装Isaac Sim 5.0和依赖
2. **数据采集**: 在仿真中采集演示数据
3. **模型训练**: 使用GR00T-N1.5微调
4. **部署测试**: 在仿真中测试策略
5. **真实部署**: 部署到真实机器人
