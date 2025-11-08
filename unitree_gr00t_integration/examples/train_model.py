#!/usr/bin/env python3
"""
Example script for training GR00T-N1.5 on Unitree G1 data.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import tyro
from dataclasses import dataclass
from typing import List

from training.finetune import FinetuneConfig, main as finetune_main


@dataclass
class TrainConfig:
    """Configuration for training."""
    
    dataset_paths: List[str]
    """Paths to dataset directories"""
    
    output_dir: str = "./checkpoints/unitree_g1"
    """Output directory for checkpoints"""
    
    num_gpus: int = 1
    """Number of GPUs"""
    
    batch_size: int = 64
    """Batch size"""
    
    max_steps: int = 15000
    """Maximum training steps"""
    
    learning_rate: float = 1e-4
    """Learning rate"""
    
    model_path: str = "nvidia/GR00T-N1.5-3B"
    """Pretrained model path"""
    
    data_config: str = "unitree_g1"
    """Data configuration name"""
    
    wandb_project: str = "unitree-g1-gr00t"
    """Wandb project name"""


def main(config: TrainConfig):
    """Main training function."""
    print("=" * 80)
    print("Unitree G1 + GR00T-N1.5 Training")
    print("=" * 80)
    
    # Create finetune config
    finetune_config = FinetuneConfig(
        dataset_paths=config.dataset_paths,
        output_dir=config.output_dir,
        num_gpus=config.num_gpus,
        batch_size=config.batch_size,
        max_steps=config.max_steps,
        learning_rate=config.learning_rate,
        model_path=config.model_path,
        data_config=config.data_config,
        wandb_project=config.wandb_project,
    )
    
    # Run training
    finetune_main(finetune_config)


if __name__ == "__main__":
    config = tyro.cli(TrainConfig)
    main(config)
