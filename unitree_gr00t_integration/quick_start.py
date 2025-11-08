#!/usr/bin/env python3
"""
快速启动脚本 - 完整的Unitree G1 + GR00T-N1.5工作流程
"""

import sys
from pathlib import Path
import argparse


def print_banner():
    """打印欢迎横幅"""
    banner = """
    ╔══════════════════════════════════════════════════════════════╗
    ║                                                              ║
    ║     Unitree G1 + GR00T-N1.5 完整集成工程                    ║
    ║                                                              ║
    ║     仿真 → 数据采集 → 训练 → 部署                           ║
    ║                                                              ║
    ╚══════════════════════════════════════════════════════════════╝
    """
    print(banner)


def check_environment():
    """检查环境设置"""
    print("检查环境设置...")
    
    # 检查Isaac Sim
    try:
        import omni.isaac.core
        print("✓ Isaac Sim已安装")
    except ImportError:
        print("✗ Isaac Sim未安装，请参考 docs/simulation_setup.md")
        return False
        
    # 检查GR00T
    try:
        import gr00t
        print("✓ GR00T模块已安装")
    except ImportError:
        print("✗ GR00T模块未安装，请运行: pip install -e ../")
        return False
        
    # 检查其他依赖
    try:
        import numpy
        import torch
        import cv2
        print("✓ 基础依赖已安装")
    except ImportError:
        print("✗ 缺少基础依赖，请运行: pip install -r requirements.txt")
        return False
        
    return True


def collect_data(args):
    """数据采集"""
    print("\n" + "="*60)
    print("步骤1: 数据采集")
    print("="*60)
    
    cmd = [
        "python", "examples/collect_data.py",
        "--output-dir", args.output_dir,
        "--task", args.task,
        "--episodes", str(args.episodes),
        "--episode-length", str(args.episode_length),
    ]
    
    if args.use_teleop:
        cmd.extend(["--use-teleop", "--teleop-mode", args.teleop_mode])
        
    print(f"运行命令: {' '.join(cmd)}")
    import subprocess
    subprocess.run(cmd)


def train_model(args):
    """模型训练"""
    print("\n" + "="*60)
    print("步骤2: 模型训练")
    print("="*60)
    
    cmd = [
        "python", "examples/train_model.py",
        "--dataset-paths", args.dataset_path,
        "--output-dir", args.checkpoint_dir,
        "--num-gpus", str(args.num_gpus),
        "--batch-size", str(args.batch_size),
        "--max-steps", str(args.max_steps),
    ]
    
    print(f"运行命令: {' '.join(cmd)}")
    import subprocess
    subprocess.run(cmd)


def deploy_policy(args):
    """策略部署"""
    print("\n" + "="*60)
    print("步骤3: 策略部署")
    print("="*60)
    
    cmd = [
        "python", "examples/deploy_policy.py",
        "--mode", args.deploy_mode,
        "--checkpoint-path", args.checkpoint_path,
        "--task-description", args.task_description,
    ]
    
    print(f"运行命令: {' '.join(cmd)}")
    import subprocess
    subprocess.run(cmd)


def main():
    parser = argparse.ArgumentParser(
        description="Unitree G1 + GR00T-N1.5 完整工作流程"
    )
    
    subparsers = parser.add_subparsers(dest="command", help="命令")
    
    # 数据采集
    collect_parser = subparsers.add_parser("collect", help="采集数据")
    collect_parser.add_argument("--output-dir", default="./collected_data")
    collect_parser.add_argument("--task", default="pick_and_place")
    collect_parser.add_argument("--episodes", type=int, default=100)
    collect_parser.add_argument("--episode-length", type=int, default=1000)
    collect_parser.add_argument("--use-teleop", action="store_true")
    collect_parser.add_argument("--teleop-mode", default="keyboard")
    
    # 模型训练
    train_parser = subparsers.add_parser("train", help="训练模型")
    train_parser.add_argument("--dataset-path", required=True)
    train_parser.add_argument("--checkpoint-dir", default="./checkpoints/unitree_g1")
    train_parser.add_argument("--num-gpus", type=int, default=1)
    train_parser.add_argument("--batch-size", type=int, default=64)
    train_parser.add_argument("--max-steps", type=int, default=15000)
    
    # 策略部署
    deploy_parser = subparsers.add_parser("deploy", help="部署策略")
    deploy_parser.add_argument("--checkpoint-path", required=True)
    deploy_parser.add_argument("--deploy-mode", choices=["sim", "real"], default="sim")
    deploy_parser.add_argument("--task-description", default="Pick the apple and place it in the basket")
    
    # 完整流程
    full_parser = subparsers.add_parser("full", help="完整流程")
    full_parser.add_argument("--output-dir", default="./collected_data")
    full_parser.add_argument("--checkpoint-dir", default="./checkpoints/unitree_g1")
    full_parser.add_argument("--episodes", type=int, default=100)
    full_parser.add_argument("--max-steps", type=int, default=15000)
    
    args = parser.parse_args()
    
    print_banner()
    
    if not check_environment():
        print("\n请先完成环境设置！")
        return
        
    if args.command == "collect":
        collect_data(args)
    elif args.command == "train":
        train_model(args)
    elif args.command == "deploy":
        deploy_policy(args)
    elif args.command == "full":
        print("\n运行完整流程...")
        # 1. 采集数据
        collect_args = argparse.Namespace(
            output_dir=args.output_dir,
            task="pick_and_place",
            episodes=args.episodes,
            episode_length=1000,
            use_teleop=False,
            teleop_mode="keyboard",
        )
        collect_data(collect_args)
        
        # 2. 训练模型
        train_args = argparse.Namespace(
            dataset_path=args.output_dir,
            checkpoint_dir=args.checkpoint_dir,
            num_gpus=1,
            batch_size=64,
            max_steps=args.max_steps,
        )
        train_model(train_args)
        
        # 3. 部署策略
        import glob
        checkpoints = glob.glob(f"{args.checkpoint_dir}/checkpoint-*")
        if checkpoints:
            latest_checkpoint = max(checkpoints, key=lambda x: int(x.split("-")[-1]))
            deploy_args = argparse.Namespace(
                checkpoint_path=latest_checkpoint,
                deploy_mode="sim",
                task_description="Pick the apple and place it in the basket",
            )
            deploy_policy(deploy_args)
        else:
            print("未找到训练好的检查点！")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
