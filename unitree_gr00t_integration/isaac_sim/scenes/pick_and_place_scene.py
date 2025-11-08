"""
Pick and Place task scene for Unitree G1 in Isaac Sim.
"""

import numpy as np
from typing import Dict, Any, Optional
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid, VisualCuboid
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
import omni.isaac.core.utils.nucleus as nucleus_utils


class PickAndPlaceScene:
    """Pick and Place task scene configuration."""
    
    def __init__(
        self,
        world: World,
        robot_urdf_path: str,
        workspace_bounds: Optional[Dict[str, np.ndarray]] = None,
    ):
        """
        Initialize the pick and place scene.
        
        Args:
            world: Isaac Sim World instance
            robot_urdf_path: Path to Unitree G1 URDF file
            workspace_bounds: Workspace boundaries {"min": array, "max": array}
        """
        self.world = world
        self.robot_urdf_path = robot_urdf_path
        self.workspace_bounds = workspace_bounds or {
            "min": np.array([-0.5, -0.5, 0.0]),
            "max": np.array([0.5, 0.5, 1.0]),
        }
        
        self.robot: Optional[Robot] = None
        self.objects: Dict[str, Any] = {}
        self.target_location: Optional[np.ndarray] = None
        
    def setup_scene(self):
        """Set up the simulation scene."""
        # Load robot
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/unitree_g1",
                name="unitree_g1",
                usd_path=self.robot_urdf_path,
                position=np.array([0.0, 0.0, 0.0]),
            )
        )
        
        # Add table
        table = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/table",
                name="table",
                position=np.array([0.0, 0.0, 0.4]),
                size=np.array([1.0, 0.6, 0.05]),
                color=np.array([0.5, 0.3, 0.2]),
            )
        )
        
        # Add target basket
        basket = self.world.scene.add(
            VisualCuboid(
                prim_path="/World/basket",
                name="basket",
                position=np.array([0.3, 0.0, 0.45]),
                size=np.array([0.2, 0.2, 0.1]),
                color=np.array([0.2, 0.2, 0.8]),
            )
        )
        self.target_location = np.array([0.3, 0.0, 0.45])
        
        # Add objects to pick (can be randomized)
        self._add_pickable_objects()
        
    def _add_pickable_objects(self):
        """Add objects that can be picked."""
        # Example: Add an apple
        apple = self.world.scene.add(
            DynamicCuboid(
                prim_path="/World/apple",
                name="apple",
                position=np.array([0.0, 0.0, 0.5]),
                size=np.array([0.05, 0.05, 0.05]),
                color=np.array([1.0, 0.0, 0.0]),
            )
        )
        self.objects["apple"] = apple
        
    def reset(self, seed: Optional[int] = None):
        """Reset the scene to initial state."""
        if seed is not None:
            np.random.seed(seed)
            
        # Reset robot to home position
        if self.robot is not None:
            home_position = self._get_home_position()
            self.robot.set_joint_positions(home_position)
            
        # Randomize object positions
        for obj_name, obj in self.objects.items():
            random_pos = self._get_random_object_position()
            obj.set_world_pose(position=random_pos)
            
    def _get_home_position(self) -> np.ndarray:
        """Get home position for the robot."""
        # 43 DOF: left_arm(7) + right_arm(7) + left_hand(7) + right_hand(7) + 
        #        left_leg(6) + right_leg(6) + waist(3)
        return np.zeros(43)
        
    def _get_random_object_position(self) -> np.ndarray:
        """Get random position within workspace."""
        min_bounds = self.workspace_bounds["min"]
        max_bounds = self.workspace_bounds["max"]
        return np.random.uniform(min_bounds, max_bounds)
        
    def get_observation(self) -> Dict[str, Any]:
        """Get current observation."""
        obs = {}
        
        # Robot state
        if self.robot is not None:
            obs["joint_positions"] = self.robot.get_joint_positions()
            obs["joint_velocities"] = self.robot.get_joint_velocities()
            
        # Camera images (if cameras are set up)
        # obs["images"] = self._get_camera_images()
        
        # Object positions
        obs["object_positions"] = {
            name: obj.get_world_pose()[0] 
            for name, obj in self.objects.items()
        }
        
        return obs
        
    def apply_action(self, action: np.ndarray):
        """Apply action to the robot."""
        if self.robot is not None:
            # Convert action to ArticulationAction
            articulation_action = ArticulationAction(joint_positions=action)
            self.robot.apply_action(articulation_action)
