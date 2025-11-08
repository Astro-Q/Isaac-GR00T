# 部署指南

本指南介绍如何部署训练好的GR00T-N1.5策略到仿真或真实机器人。

## 部署模式

### 1. 仿真部署

在Isaac Sim中运行策略：

```bash
python examples/deploy_policy.py \
    --mode sim \
    --checkpoint-path ./checkpoints/unitree_g1/checkpoint-15000 \
    --task-description "Pick the apple and place it in the basket"
```

### 2. 真实机器人部署

#### 步骤1: 启动推理服务器

在一个终端启动推理服务器：

```bash
python scripts/inference_service.py \
    --server \
    --model-path ./checkpoints/unitree_g1/checkpoint-15000 \
    --embodiment-tag new_embodiment \
    --data-config unitree_g1 \
    --denoising-steps 4
```

#### 步骤2: 运行机器人客户端

在另一个终端运行部署脚本：

```bash
python examples/deploy_policy.py \
    --mode real \
    --checkpoint-path ./checkpoints/unitree_g1/checkpoint-15000 \
    --use-server \
    --server-host localhost \
    --server-port 5555 \
    --task-description "Pick the apple and place it in the basket"
```

## 部署架构

### 服务器-客户端模式（推荐）

```
┌─────────────────┐         ┌──────────────────┐         ┌──────────────┐
│  Robot Client   │────────▶│  Inference       │────────▶│  Unitree G1  │
│  (deployment)   │  obs    │  Server          │  action  │  Robot       │
└─────────────────┘         └──────────────────┘         └──────────────┘
```

优势：
- 模型运行在GPU服务器上
- 机器人客户端只需发送观测和接收动作
- 易于调试和监控

### 直接部署模式

模型直接加载到机器人控制计算机：

```bash
python examples/deploy_policy.py \
    --mode real \
    --checkpoint-path ./checkpoints/unitree_g1/checkpoint-15000 \
    --use-server false
```

## 机器人接口集成

### Unitree G1 SDK集成

需要集成Unitree G1的SDK来获取状态和控制机器人：

```python
# 示例：集成Unitree SDK
from unitree_sdk import UnitreeG1

class UnitreeG1Interface:
    def __init__(self, port="/dev/ttyUSB0"):
        self.robot = UnitreeG1(port=port)
        
    def get_joint_positions(self):
        return self.robot.get_joint_positions()
        
    def set_joint_positions(self, positions):
        self.robot.set_joint_positions(positions)
```

### 相机接口

集成机器人相机获取图像：

```python
import cv2

class CameraInterface:
    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        
    def get_image(self):
        ret, frame = self.cap.read()
        return frame if ret else None
```

## 安全注意事项

1. **限速**: 限制动作幅度，避免突然的大幅运动
2. **碰撞检测**: 实现碰撞检测和安全停止
3. **紧急停止**: 提供紧急停止按钮
4. **监控**: 实时监控机器人状态和策略输出

## 性能优化

### 推理速度

- 使用`--denoising-steps 4`（4步通常足够）
- 使用TensorRT加速（参考deployment_scripts/README.md）
- 优化图像预处理

### 控制频率

推荐控制频率：10-20 Hz

```python
control_frequency = 10.0  # Hz
dt = 1.0 / control_frequency
```

## 调试技巧

### 1. 可视化观测和动作

```python
import matplotlib.pyplot as plt

# 可视化图像
plt.imshow(obs['image'])
plt.show()

# 可视化动作
print(f"Action: {action[:10]}")  # 打印前10维
```

### 2. 记录部署数据

```python
# 保存部署时的观测和动作用于分析
deployment_data = {
    "observations": [],
    "actions": [],
    "rewards": [],
}
```

### 3. 使用仿真验证

先在仿真中验证策略，再部署到真实机器人。

## 常见问题

### Q: 动作不稳定
A: 
- 检查动作归一化是否正确
- 添加动作平滑（moving average）
- 检查观测是否正常

### Q: 机器人不执行动作
A:
- 检查机器人连接
- 验证动作空间是否匹配
- 检查安全限制是否过严

### Q: 推理延迟过高
A:
- 使用TensorRT加速
- 减少denoising steps
- 优化图像分辨率

## 下一步

部署成功后，可以：
1. 收集在线数据用于进一步微调
2. 评估策略性能
3. 扩展到更多任务
