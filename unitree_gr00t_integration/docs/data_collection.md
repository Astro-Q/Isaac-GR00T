# 数据采集指南

本指南介绍如何在Isaac Sim中采集Unitree G1机器人的演示数据。

## 数据格式

采集的数据遵循LeRobot兼容格式：

```
collected_data/
├── videos/              # 视频文件
│   ├── episode_00000.mp4
│   └── ...
├── data/                # 状态和动作数据（Parquet格式）
│   ├── episode_00000.parquet
│   └── ...
└── meta/                # 元数据
    ├── modality.json    # 数据模态配置
    ├── episode_00000.json
    └── ...
```

## 采集方式

### 1. 脚本化采集（用于快速生成数据）

使用预定义的策略（如随机策略）采集数据：

```bash
python examples/collect_data.py \
    --output-dir ./collected_data \
    --task pick_and_place \
    --episodes 100 \
    --episode-length 1000
```

### 2. 遥操作采集（推荐用于高质量演示）

使用键盘或手柄进行遥操作：

```bash
python examples/collect_data.py \
    --output-dir ./collected_data \
    --task pick_and_place \
    --episodes 50 \
    --use-teleop \
    --teleop-mode keyboard
```

### 3. 编程接口采集

使用Python API进行自定义采集：

```python
from data_collection.lerobot_collector import LeRobotCollector
from isaac_sim.scenes.pick_and_place_scene import PickAndPlaceScene

# 初始化采集器
collector = LeRobotCollector(output_dir="./collected_data")

# 初始化场景
scene = PickAndPlaceScene(world=world, robot_urdf_path="...")
scene.setup_scene()

# 采集一个episode
collector.start_episode(task_description="Pick the apple")
for step in range(1000):
    obs = scene.get_observation()
    action = get_action_from_policy_or_teleop(obs)
    collector.add_step(observation=obs, action=action)
    scene.apply_action(action)
    scene.world.step()
    
episode_idx = collector.end_episode()
```

## 数据模态配置

采集完成后，确保创建`modality.json`文件：

```json
{
    "state": {
        "left_arm": {"start": 0, "end": 7},
        "right_arm": {"start": 7, "end": 14},
        "left_hand": {"start": 14, "end": 21},
        "right_hand": {"start": 21, "end": 28}
    },
    "action": {
        "left_arm": {"start": 0, "end": 7},
        "right_arm": {"start": 7, "end": 14},
        "left_hand": {"start": 14, "end": 21},
        "right_hand": {"start": 21, "end": 28}
    },
    "video": {
        "ego_view": {
            "original_key": "observation.images.ego_view"
        }
    },
    "annotation": {
        "human.task_description": {
            "original_key": "task_description"
        }
    }
}
```

采集器会自动创建此文件，但可以手动修改以满足特定需求。

## 数据质量建议

1. **多样性**: 采集不同物体、不同位置、不同角度的演示
2. **一致性**: 保持任务描述清晰一致
3. **长度**: 每个episode建议500-2000步
4. **数量**: 至少50-100个episode用于微调

## 验证数据

使用GR00T的数据加载工具验证数据格式：

```bash
python scripts/load_dataset.py --dataset-path ./collected_data
```

## 下一步

采集完数据后，可以开始模型微调：
```bash
python examples/train_model.py --dataset-paths ./collected_data
```
