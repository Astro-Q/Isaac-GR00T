"""
Pick and Place task implementation for Unitree G1.
"""

import numpy as np
from typing import Dict, Any, Optional, Tuple
from omni.isaac.core.tasks import BaseTask
from omni.isaac.core.utils.types import ArticulationAction
from .pick_and_place_scene import PickAndPlaceScene


class PickAndPlaceTask(BaseTask):
    """Pick and Place task for Unitree G1 robot."""
    
    def __init__(
        self,
        name: str = "pick_and_place",
        robot_urdf_path: str = "assets/unitree_g1/urdf/unitree_g1.urdf",
        **kwargs
    ):
        super().__init__(name=name, **kwargs)
        self.robot_urdf_path = robot_urdf_path
        self.scene: Optional[PickAndPlaceScene] = None
        
    def set_up_scene(self, scene):
        """Set up the task scene."""
        self.scene = PickAndPlaceScene(
            world=self._world,
            robot_urdf_path=self.robot_urdf_path,
        )
        self.scene.setup_scene()
        super().set_up_scene(scene)
        
    def get_observations(self) -> Dict[str, Any]:
        """Get task observations."""
        if self.scene is None:
            return {}
        return self.scene.get_observation()
        
    def pre_physics_step(self, actions: np.ndarray) -> None:
        """Execute actions before physics step."""
        if self.scene is not None:
            self.scene.apply_action(actions)
            
    def reset(self) -> None:
        """Reset the task."""
        if self.scene is not None:
            self.scene.reset()
            
    def calculate_reward(self) -> float:
        """Calculate task reward."""
        # Simple reward: distance to object and target
        if self.scene is None:
            return 0.0
            
        obs = self.scene.get_observation()
        
        # Get end-effector positions (simplified)
        # In real implementation, compute from forward kinematics
        left_eef_pos = self._get_end_effector_position("left")
        right_eef_pos = self._get_end_effector_position("right")
        
        # Distance to object
        if "apple" in obs["object_positions"]:
            obj_pos = obs["object_positions"]["apple"]
            dist_to_obj = min(
                np.linalg.norm(left_eef_pos - obj_pos),
                np.linalg.norm(right_eef_pos - obj_pos)
            )
        else:
            dist_to_obj = 1.0
            
        # Reward: negative distance (closer is better)
        reward = -dist_to_obj
        
        return reward
        
    def _get_end_effector_position(self, arm: str) -> np.ndarray:
        """Get end-effector position (simplified)."""
        # TODO: Implement forward kinematics
        # For now, return approximate position
        return np.array([0.0, 0.0, 0.8])
