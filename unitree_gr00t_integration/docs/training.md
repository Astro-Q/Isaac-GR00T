# 模型训练指南

本指南介绍如何使用采集的数据微调GR00T-N1.5模型。

## 快速开始

### 基本训练命令

```bash
python examples/train_model.py \
    --dataset-paths ./collected_data \
    --output-dir ./checkpoints/unitree_g1 \
    --num-gpus 1 \
    --batch-size 64 \
    --max-steps 15000
```

### 使用多个数据集

```bash
python examples/train_model.py \
    --dataset-paths ./collected_data/task1 ./collected_data/task2 \
    --output-dir ./checkpoints/unitree_g1_multi_task
```

## 训练配置

### 数据配置

项目已包含`unitree_g1`数据配置，支持：
- 双臂控制（left_arm, right_arm）
- 双手控制（left_hand, right_hand）
- 视频输入（ego_view）

### 模型配置

默认使用GR00T-N1.5-3B模型：
- **Backbone**: Eagle 2.5 VLM（冻结）
- **Action Head**: Flow Matching Diffusion Transformer
- **Embodiment Tag**: `new_embodiment`

### 训练超参数

推荐配置：

| 参数 | 值 | 说明 |
|------|-----|------|
| batch_size | 64 | 根据GPU内存调整 |
| learning_rate | 1e-4 | 初始学习率 |
| max_steps | 15000 | 训练步数 |
| warmup_steps | 500 | 预热步数 |
| denoising_steps | 4 | 推理时去噪步数 |

### 微调选项

可以控制哪些部分参与微调：

```bash
# 只微调action head（推荐用于小数据集）
python examples/train_model.py \
    --dataset-paths ./collected_data \
    --tune-visual false \
    --tune-llm false \
    --tune-projector true \
    --tune-diffusion-model true

# 全模型微调（需要更多数据和计算资源）
python examples/train_model.py \
    --dataset-paths ./collected_data \
    --tune-visual true \
    --tune-projector true \
    --tune-diffusion-model true
```

### LoRA微调（低内存选项）

对于GPU内存有限的情况，可以使用LoRA：

```bash
python examples/train_model.py \
    --dataset-paths ./collected_data \
    --use-lora \
    --lora-rank 64 \
    --lora-alpha 128
```

## 监控训练

### Wandb集成

训练过程会自动记录到Wandb（如果配置了）：

```bash
export WANDB_API_KEY=your_api_key
python examples/train_model.py \
    --dataset-paths ./collected_data \
    --wandb-project unitree-g1-gr00t
```

### 检查点

训练过程中会定期保存检查点：
- 每1000步保存一次
- 保存在`--output-dir`指定的目录
- 保留最近3个检查点

## 评估模型

训练完成后，评估模型性能：

```bash
python scripts/eval_policy.py \
    --plot \
    --embodiment-tag new_embodiment \
    --model-path ./checkpoints/unitree_g1/checkpoint-15000 \
    --data-config unitree_g1 \
    --dataset-path ./collected_data
```

## 常见问题

### Q: GPU内存不足
A: 
- 减小batch_size
- 使用`--no-tune-diffusion-model`标志
- 使用LoRA微调

### Q: 训练不收敛
A:
- 检查数据质量和数量
- 调整学习率
- 增加训练步数
- 检查数据预处理是否正确

### Q: 训练速度慢
A:
- 使用更多GPU（`--num-gpus 2`）
- 启用混合精度训练
- 使用更快的视频后端（torchcodec）

## 下一步

训练完成后，可以部署模型：
```bash
python examples/deploy_policy.py \
    --mode sim \
    --checkpoint-path ./checkpoints/unitree_g1/checkpoint-15000
```
