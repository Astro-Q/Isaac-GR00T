#!/usr/bin/env python3
"""
Fine-tuning script for GR00T-N1.5 on Unitree G1 data.
"""

import sys
import os
from pathlib import Path

# Add parent directory to path to import gr00t modules
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import tyro
from dataclasses import dataclass
from typing import List, Optional
import yaml

from gr00t.experiment.data_config import load_data_config, UnitreeG1DataConfig
from gr00t.data.embodiment_tags import EmbodimentTag
from scripts.gr00t_finetune import main as gr00t_finetune_main


@dataclass
class FinetuneConfig:
    """Configuration for fine-tuning."""
    
    # Data paths
    dataset_paths: List[str]
    """Paths to dataset directories (can be multiple)"""
    
    # Model
    model_path: str = "nvidia/GR00T-N1.5-3B"
    """Path to pretrained model"""
    
    embodiment_tag: str = "new_embodiment"
    """Embodiment tag for the model"""
    
    data_config: str = "unitree_g1"
    """Data configuration name"""
    
    # Training
    num_gpus: int = 1
    """Number of GPUs to use"""
    
    batch_size: int = 64
    """Batch size per GPU"""
    
    max_steps: int = 15000
    """Maximum training steps"""
    
    learning_rate: float = 1e-4
    """Learning rate"""
    
    # Output
    output_dir: str = "./checkpoints/unitree_g1"
    """Output directory for checkpoints"""
    
    # Fine-tuning options
    tune_visual: bool = True
    """Whether to tune visual encoder"""
    
    tune_llm: bool = False
    """Whether to tune LLM"""
    
    tune_projector: bool = True
    """Whether to tune projector"""
    
    tune_diffusion_model: bool = True
    """Whether to tune diffusion model"""
    
    # LoRA
    use_lora: bool = False
    """Whether to use LoRA"""
    
    lora_rank: int = 64
    """LoRA rank"""
    
    lora_alpha: int = 128
    """LoRA alpha"""
    
    # Other options
    video_backend: str = "decord"
    """Video backend (decord or torchcodec)"""
    
    no_tune_diffusion_model: bool = False
    """Flag to disable diffusion model tuning (for low memory)"""
    
    gradient_checkpointing: bool = True
    """Enable gradient checkpointing"""
    
    mixed_precision: str = "bf16"
    """Mixed precision training (bf16, fp16, or no)"""
    
    wandb_project: Optional[str] = "unitree-g1-gr00t"
    """Wandb project name"""
    
    log_interval: int = 100
    """Logging interval"""
    
    save_interval: int = 1000
    """Checkpoint saving interval"""
    
    eval_interval: int = 1000
    """Evaluation interval"""


def main(config: FinetuneConfig):
    """Main training function."""
    print("=" * 80)
    print("Unitree G1 + GR00T-N1.5 Fine-tuning")
    print("=" * 80)
    print(f"Dataset paths: {config.dataset_paths}")
    print(f"Model: {config.model_path}")
    print(f"Embodiment tag: {config.embodiment_tag}")
    print(f"Data config: {config.data_config}")
    print(f"Output dir: {config.output_dir}")
    print("=" * 80)
    
    # Prepare arguments for gr00t_finetune script
    import sys
    original_argv = sys.argv.copy()
    
    # Build command line arguments
    sys.argv = [
        "gr00t_finetune.py",
        "--dataset-path"] + config.dataset_paths + [
        "--num-gpus", str(config.num_gpus),
        "--batch-size", str(config.batch_size),
        "--max-steps", str(config.max_steps),
        "--output-dir", config.output_dir,
        "--data-config", config.data_config,
        "--embodiment-tag", config.embodiment_tag,
        "--model-path", config.model_path,
    ]
    
    # Add optional flags
    if config.no_tune_diffusion_model or not config.tune_diffusion_model:
        sys.argv.append("--no-tune-diffusion-model")
    if config.use_lora:
        sys.argv.extend(["--lora-rank", str(config.lora_rank)])
        sys.argv.extend(["--lora-alpha", str(config.lora_alpha)])
    if config.wandb_project:
        sys.argv.extend(["--wandb-project", config.wandb_project])
        
    try:
        # Import and run the finetune script
        from scripts.gr00t_finetune import main as finetune_main
        finetune_main()
    finally:
        sys.argv = original_argv


if __name__ == "__main__":
    config = tyro.cli(FinetuneConfig)
    main(config)
