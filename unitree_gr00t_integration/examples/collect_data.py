#!/usr/bin/env python3
"""
Example script for collecting demonstration data in Isaac Sim.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import numpy as np
import tyro
from dataclasses import dataclass
from typing import Optional

from data_collection.lerobot_collector import LeRobotCollector
from isaac_sim.scenes.pick_and_place_scene import PickAndPlaceScene
from omni.isaac.core import World


@dataclass
class CollectDataConfig:
    """Configuration for data collection."""
    
    output_dir: str = "./collected_data"
    """Output directory for collected data"""
    
    task: str = "pick_and_place"
    """Task name"""
    
    episodes: int = 100
    """Number of episodes to collect"""
    
    episode_length: int = 1000
    """Maximum length of each episode"""
    
    robot_urdf_path: str = "assets/unitree_g1/urdf/unitree_g1.urdf"
    """Path to robot URDF"""
    
    task_description: str = "Pick the apple and place it in the basket"
    """Task description for demonstrations"""
    
    # Control
    control_frequency: float = 20.0  # Hz
    """Control frequency"""
    
    # Teleoperation (if enabled)
    use_teleop: bool = False
    """Use teleoperation for data collection"""
    
    teleop_mode: str = "keyboard"  # or "joystick"
    """Teleoperation mode"""


def collect_episode(
    collector: LeRobotCollector,
    scene: PickAndPlaceScene,
    task_description: str,
    episode_length: int,
    control_frequency: float,
    use_teleop: bool = False,
):
    """Collect a single episode."""
    collector.start_episode(task_description=task_description)
    scene.reset()
    
    dt = 1.0 / control_frequency
    
    for step in range(episode_length):
        # Get observation
        obs = scene.get_observation()
        
        # Get action (from teleop or policy)
        if use_teleop:
            # TODO: Get action from teleoperation
            action = np.zeros(43)  # Placeholder
        else:
            # Use random or scripted policy for now
            action = np.random.uniform(-0.1, 0.1, 43)
            
        # Add step to collector
        collector.add_step(
            observation={"joint_positions": obs.get("joint_positions", np.zeros(43))},
            action=action,
        )
        
        # Apply action
        scene.apply_action(action)
        
        # Step simulation
        scene.world.step(render=True)
        
        # Check termination
        # TODO: Add proper termination conditions
        
    # End episode
    episode_idx = collector.end_episode()
    return episode_idx


def main(config: CollectDataConfig):
    """Main data collection function."""
    print("=" * 80)
    print("Unitree G1 Data Collection")
    print("=" * 80)
    print(f"Output directory: {config.output_dir}")
    print(f"Task: {config.task}")
    print(f"Episodes: {config.episodes}")
    print(f"Episode length: {config.episode_length}")
    print("=" * 80)
    
    # Initialize data collector
    collector = LeRobotCollector(
        output_dir=config.output_dir,
        episode_length=config.episode_length,
    )
    
    # Initialize simulation
    world = World(stage_units_in_meters=1.0)
    scene = PickAndPlaceScene(
        world=world,
        robot_urdf_path=config.robot_urdf_path,
    )
    scene.setup_scene()
    world.reset()
    
    # Collect episodes
    for episode in range(config.episodes):
        print(f"\nCollecting episode {episode + 1}/{config.episodes}...")
        
        episode_idx = collect_episode(
            collector=collector,
            scene=scene,
            task_description=config.task_description,
            episode_length=config.episode_length,
            control_frequency=config.control_frequency,
            use_teleop=config.use_teleop,
        )
        
        print(f"Episode {episode_idx} collected successfully")
        
    # Create modality config
    collector.create_modality_config()
    
    print("\n" + "=" * 80)
    print(f"Data collection complete! Data saved to {config.output_dir}")
    print("=" * 80)


if __name__ == "__main__":
    config = tyro.cli(CollectDataConfig)
    main(config)
