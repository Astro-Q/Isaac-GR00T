#!/usr/bin/env python3
"""
Example script for deploying GR00T-N1.5 policy on Unitree G1.
Supports both simulation and real robot deployment.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import tyro
from dataclasses import dataclass
from typing import Literal

from deployment.sim_deploy import SimDeployConfig, SimDeployment
from deployment.real_robot_deploy import RealRobotDeployConfig, RealRobotDeployment


@dataclass
class DeployConfig:
    """Configuration for policy deployment."""
    
    mode: Literal["sim", "real"] = "sim"
    """Deployment mode: simulation or real robot"""
    
    checkpoint_path: str
    """Path to trained model checkpoint"""
    
    # Task
    task_description: str = "Pick the apple and place it in the basket"
    """Language instruction"""
    
    # Model config
    embodiment_tag: str = "new_embodiment"
    """Embodiment tag"""
    
    data_config: str = "unitree_g1"
    """Data configuration"""
    
    # Simulation config (for sim mode)
    robot_urdf_path: str = "assets/unitree_g1/urdf/unitree_g1.urdf"
    """Robot URDF path (for sim)"""
    
    # Real robot config (for real mode)
    use_server: bool = True
    """Use inference server (for real robot)"""
    
    server_host: str = "localhost"
    """Server host"""
    
    server_port: int = 5555
    """Server port"""
    
    robot_port: str = None
    """Robot port (for real robot)"""


def main(config: DeployConfig):
    """Main deployment function."""
    print("=" * 80)
    print(f"Unitree G1 Policy Deployment ({config.mode.upper()} mode)")
    print("=" * 80)
    print(f"Checkpoint: {config.checkpoint_path}")
    print(f"Task: {config.task_description}")
    print("=" * 80)
    
    if config.mode == "sim":
        # Simulation deployment
        sim_config = SimDeployConfig(
            model_path=config.checkpoint_path,
            embodiment_tag=config.embodiment_tag,
            data_config=config.data_config,
            robot_urdf_path=config.robot_urdf_path,
            task_description=config.task_description,
        )
        deployment = SimDeployment(sim_config)
        deployment.run()
        
    elif config.mode == "real":
        # Real robot deployment
        real_config = RealRobotDeployConfig(
            model_path=config.checkpoint_path,
            embodiment_tag=config.embodiment_tag,
            data_config=config.data_config,
            use_server=config.use_server,
            server_host=config.server_host,
            server_port=config.server_port,
            robot_port=config.robot_port,
            task_description=config.task_description,
        )
        deployment = RealRobotDeployment(real_config)
        deployment.run()
        
    else:
        raise ValueError(f"Unknown mode: {config.mode}")


if __name__ == "__main__":
    config = tyro.cli(DeployConfig)
    main(config)
