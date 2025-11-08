"""
Simulation deployment for Unitree G1 with GR00T-N1.5 policy.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import numpy as np
from typing import Dict, Any, Optional
import tyro
from dataclasses import dataclass

from gr00t.model.policy import Gr00tPolicy
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.experiment.data_config import load_data_config
from gr00t.eval.service import ExternalRobotInferenceClient

# Isaac Sim imports
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.types import ArticulationAction


@dataclass
class SimDeployConfig:
    """Configuration for simulation deployment."""
    
    # Model
    model_path: str
    """Path to trained model checkpoint"""
    
    embodiment_tag: str = "new_embodiment"
    """Embodiment tag"""
    
    data_config: str = "unitree_g1"
    """Data configuration name"""
    
    # Simulation
    robot_urdf_path: str = "assets/unitree_g1/urdf/unitree_g1.urdf"
    """Path to robot URDF"""
    
    # Policy
    denoising_steps: int = 4
    """Number of denoising steps"""
    
    action_horizon: int = 16
    """Action horizon"""
    
    # Task
    task_description: str = "Pick the apple and place it in the basket"
    """Language instruction for the task"""
    
    # Control
    control_frequency: float = 10.0  # Hz
    """Control frequency"""
    
    # Visualization
    headless: bool = False
    """Run in headless mode"""


class SimDeployment:
    """Simulation deployment manager."""
    
    def __init__(self, config: SimDeployConfig):
        self.config = config
        
        # Load policy
        print(f"Loading policy from {config.model_path}...")
        data_config = load_data_config(config.data_config)
        modality_config = data_config.modality_config()
        transforms = data_config.transform()
        
        self.policy = Gr00tPolicy(
            model_path=config.model_path,
            modality_config=modality_config,
            modality_transform=transforms,
            embodiment_tag=EmbodimentTag[config.embodiment_tag.upper()] if hasattr(EmbodimentTag, config.embodiment_tag.upper()) else EmbodimentTag.NEW_EMBODIMENT,
            device="cuda",
        )
        
        # Initialize simulation
        self.world = None
        self.robot = None
        
    def setup_simulation(self):
        """Set up Isaac Sim simulation."""
        print("Setting up simulation...")
        self.world = World(stage_units_in_meters=1.0)
        
        # Load robot
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/unitree_g1",
                name="unitree_g1",
                usd_path=self.config.robot_urdf_path,
                position=np.array([0.0, 0.0, 0.0]),
            )
        )
        
        # Reset world
        self.world.reset()
        
    def get_observation(self) -> Dict[str, Any]:
        """Get current observation from simulation."""
        obs = {}
        
        # Robot state
        if self.robot is not None:
            joint_positions = self.robot.get_joint_positions()
            obs["state"] = {
                "left_arm": joint_positions[0:7],
                "right_arm": joint_positions[7:14],
                "left_hand": joint_positions[14:21],
                "right_hand": joint_positions[21:28],
            }
            
        # Camera images (if cameras are set up)
        # TODO: Add camera capture
        
        # Language instruction
        obs["language"] = self.config.task_description
        
        return obs
        
    def run_episode(self, max_steps: int = 1000):
        """Run a single episode."""
        if self.world is None:
            self.setup_simulation()
            
        self.world.reset()
        
        for step in range(max_steps):
            # Get observation
            obs = self.get_observation()
            
            # Get action from policy
            action_output = self.policy.get_action(obs)
            action = action_output["action_pred"][0]  # Take first action from horizon
            
            # Apply action
            if self.robot is not None:
                articulation_action = ArticulationAction(joint_positions=action)
                self.robot.apply_action(articulation_action)
                
            # Step simulation
            self.world.step(render=True)
            
            # Check termination (simplified)
            # TODO: Add proper termination conditions
            
        print(f"Episode completed after {max_steps} steps")
        
    def run(self):
        """Run the deployment."""
        self.setup_simulation()
        self.run_episode()


def main():
    """Main entry point."""
    config = tyro.cli(SimDeployConfig)
    deployment = SimDeployment(config)
    deployment.run()


if __name__ == "__main__":
    main()
