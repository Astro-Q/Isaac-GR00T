# Unitree G1 + GR00T-N1.5 集成工程

## 项目概述

本项目整合了三个开源项目，为Unitree G1人形机器人提供完整的VLA（Vision-Language-Action）模型训练和部署流程：

1. **Isaac-GR00T** - NVIDIA GR00T-N1.5 VLA模型
2. **unitree_sim_isaaclab** - Unitree G1的Isaac Sim仿真
3. **unitree_IL_lerobot** - LeRobot格式数据采集

## 功能特性

✅ **Isaac Sim 5.0 & Isaac Lab仿真环境**  
✅ **Unitree G1双臂5指灵巧手仿真**  
✅ **抓取任务仿真场景**  
✅ **LeRobot格式数据采集**  
✅ **GR00T-N1.5模型微调**  
✅ **仿真和真实机器人部署**  

## 重要说明：依赖关系

⚠️ **本工程目前不能完全独立运行**，需要依赖父目录的 `gr00t` 模块。

### 目录结构要求

本工程必须位于 `Isaac-GR00T` 仓库的根目录下：

```
Isaac-GR00T/
├── gr00t/                      # GR00T核心模块（必需）
├── scripts/                    # 训练脚本（必需）
└── unitree_gr00t_integration/  # 本工程
    ├── README.md
    ├── requirements.txt
    └── ...
```

### 独立运行分析

详细分析请参考：[STANDALONE_ANALYSIS.md](STANDALONE_ANALYSIS.md)

## 快速开始

### 1. 环境设置

#### 方法1：使用自动安装脚本（推荐）

```bash
# 运行安装脚本（会自动安装所有依赖）
bash install_dependencies.sh
```

#### 方法2：手动安装

```bash
# 1. 安装 gr00t 包（必需）
cd ..  # 回到 Isaac-GR00T 根目录
pip install -e .

# 2. 安装本工程的Python依赖
cd unitree_gr00t_integration
pip install -r requirements.txt

# 3. 安装Isaac Sim环境（需要单独安装Isaac Sim 5.0）
# 参考: docs/simulation_setup.md
```

### 2. 验证安装

```bash
# 检查 gr00t 模块是否可以导入
python3 -c "import gr00t; print('✓ gr00t 模块安装成功')"
```

### 3. 完整工作流程

使用快速启动脚本运行完整流程：

```bash
# 完整流程：数据采集 → 训练 → 部署
python quick_start.py full \
    --output-dir ./collected_data \
    --checkpoint-dir ./checkpoints/unitree_g1 \
    --episodes 100 \
    --max-steps 15000
```

### 4. 分步执行

#### 步骤1: 数据采集

```bash
python examples/collect_data.py \
    --output-dir ./collected_data \
    --task pick_and_place \
    --episodes 100 \
    --episode-length 1000
```

#### 步骤2: 模型训练

```bash
python examples/train_model.py \
    --dataset-paths ./collected_data \
    --output-dir ./checkpoints/unitree_g1 \
    --num-gpus 1 \
    --batch-size 64 \
    --max-steps 15000
```

#### 步骤3: 策略部署

```bash
# 仿真部署
python examples/deploy_policy.py \
    --mode sim \
    --checkpoint-path ./checkpoints/unitree_g1/checkpoint-15000 \
    --task-description "Pick the apple and place it in the basket"
```

## 项目结构

```
unitree_gr00t_integration/
├── README.md                    # 本文件
├── quick_start.py               # 快速启动脚本
├── isaac_sim/                   # 仿真模块
├── data_collection/             # 数据采集
├── training/                    # 模型训练
├── deployment/                  # 部署模块
├── examples/                    # 示例脚本
└── docs/                        # 文档
```

详细结构请参考 [PROJECT_STRUCTURE.md](PROJECT_STRUCTURE.md)

## 文档

- [独立运行分析](STANDALONE_ANALYSIS.md) - 了解依赖关系和独立运行方案
- [仿真环境设置](docs/simulation_setup.md)
- [数据采集指南](docs/data_collection.md)
- [模型训练指南](docs/training.md)
- [部署指南](docs/deployment.md)

## 依赖项目

本项目整合了以下开源项目：

- [Isaac-GR00T](https://github.com/Astro-Q/Isaac-GR00T) - GR00T-N1.5模型
- [unitree_sim_isaaclab](https://github.com/Astro-Q/unitree_sim_isaaclab) - Unitree仿真
- [unitree_IL_lerobot](https://github.com/Astro-Q/unitree_IL_lerobot) - 数据采集

## 许可证

本项目整合了多个开源项目，请参考各项目的许可证：
- Isaac-GR00T: Apache 2.0
- unitree_sim_isaaclab: 请参考原项目
- unitree_IL_lerobot: 请参考原项目

## 贡献

欢迎提交Issue和Pull Request！

## 联系方式

如有问题，请提交Issue或联系项目维护者。
