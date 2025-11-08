"""
Real robot deployment for Unitree G1 with GR00T-N1.5 policy.
"""

import sys
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

import numpy as np
from typing import Dict, Any, Optional
import tyro
from dataclasses import dataclass
import time

from gr00t.model.policy import Gr00tPolicy
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.experiment.data_config import load_data_config
from gr00t.eval.service import ExternalRobotInferenceClient


@dataclass
class RealRobotDeployConfig:
    """Configuration for real robot deployment."""
    
    # Model
    model_path: str
    """Path to trained model checkpoint"""
    
    embodiment_tag: str = "new_embodiment"
    """Embodiment tag"""
    
    data_config: str = "unitree_g1"
    """Data configuration name"""
    
    # Policy server (if using server mode)
    use_server: bool = True
    """Use inference server mode"""
    
    server_host: str = "localhost"
    """Server host"""
    
    server_port: int = 5555
    """Server port"""
    
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
    
    # Robot interface
    robot_type: str = "unitree_g1"
    """Robot type"""
    
    robot_port: Optional[str] = None
    """Robot port (e.g., /dev/ttyUSB0)"""


class RealRobotDeployment:
    """Real robot deployment manager."""
    
    def __init__(self, config: RealRobotDeployConfig):
        self.config = config
        
        if config.use_server:
            # Use inference server
            print(f"Connecting to inference server at {config.server_host}:{config.server_port}...")
            self.policy_client = ExternalRobotInferenceClient(
                host=config.server_host,
                port=config.server_port,
            )
            self.policy = None
        else:
            # Load policy directly
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
            self.policy_client = None
            
        # Initialize robot interface
        self.robot_interface = self._init_robot_interface()
        
    def _init_robot_interface(self):
        """Initialize robot interface."""
        # TODO: Integrate with Unitree robot SDK
        # Example: from unitree_sdk import UnitreeG1
        # return UnitreeG1(port=self.config.robot_port)
        print("Warning: Robot interface not implemented. Please integrate with Unitree SDK.")
        return None
        
    def get_observation(self) -> Dict[str, Any]:
        """Get current observation from robot."""
        obs = {}
        
        # Get robot state
        if self.robot_interface is not None:
            # TODO: Get actual robot state
            joint_positions = self.robot_interface.get_joint_positions()
            obs["state"] = {
                "left_arm": joint_positions[0:7],
                "right_arm": joint_positions[7:14],
                "left_hand": joint_positions[14:21],
                "right_hand": joint_positions[21:28],
            }
        else:
            # Mock data for testing
            obs["state"] = {
                "left_arm": np.zeros(7),
                "right_arm": np.zeros(7),
                "left_hand": np.zeros(7),
                "right_hand": np.zeros(7),
            }
            
        # Get camera images
        # TODO: Capture images from robot cameras
        
        # Language instruction
        obs["language"] = self.config.task_description
        
        return obs
        
    def apply_action(self, action: np.ndarray):
        """Apply action to robot."""
        if self.robot_interface is not None:
            # TODO: Send action to robot
            self.robot_interface.set_joint_positions(action)
        else:
            print(f"Mock: Applying action {action[:5]}...")
            
    def get_action(self, obs: Dict[str, Any]) -> np.ndarray:
        """Get action from policy."""
        if self.policy_client is not None:
            # Use inference server
            action_output = self.policy_client.get_action(obs)
            return action_output["action_pred"][0]
        elif self.policy is not None:
            # Use local policy
            action_output = self.policy.get_action(obs)
            return action_output["action_pred"][0]
        else:
            raise RuntimeError("No policy available")
            
    def run_episode(self, max_steps: int = 1000):
        """Run a single episode."""
        print(f"Starting episode with task: {self.config.task_description}")
        
        dt = 1.0 / self.config.control_frequency
        
        for step in range(max_steps):
            start_time = time.time()
            
            # Get observation
            obs = self.get_observation()
            
            # Get action from policy
            action = self.get_action(obs)
            
            # Apply action
            self.apply_action(action)
            
            # Wait for control frequency
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
                
            if step % 100 == 0:
                print(f"Step {step}/{max_steps}")
                
        print("Episode completed")
        
    def run(self):
        """Run the deployment."""
        self.run_episode()


def main():
    """Main entry point."""
    config = tyro.cli(RealRobotDeployConfig)
    deployment = RealRobotDeployment(config)
    deployment.run()


if __name__ == "__main__":
    main()
