"""
LeRobot-compatible data collector for Unitree G1 robot.
Collects demonstrations in the format required for GR00T training.
"""

import os
import json
import numpy as np
from pathlib import Path
from typing import Dict, Any, Optional, List
import pandas as pd
from datetime import datetime
import cv2


class LeRobotCollector:
    """
    Collects robot demonstrations in LeRobot format.
    
    Data structure:
    - videos/: Video files (.mp4)
    - data/: Parquet files with state/action data
    - meta/: JSON/JSONL files with metadata and modality config
    """
    
    def __init__(
        self,
        output_dir: str,
        episode_length: int = 1000,
        video_fps: int = 20,
        video_resolution: tuple = (640, 480),
    ):
        """
        Initialize the data collector.
        
        Args:
            output_dir: Directory to save collected data
            episode_length: Maximum length of an episode
            video_fps: FPS for video recording
            video_resolution: (width, height) for video
        """
        self.output_dir = Path(output_dir)
        self.episode_length = episode_length
        self.video_fps = video_fps
        self.video_resolution = video_resolution
        
        # Create directory structure
        self.videos_dir = self.output_dir / "videos"
        self.data_dir = self.output_dir / "data"
        self.meta_dir = self.output_dir / "meta"
        
        for dir_path in [self.videos_dir, self.data_dir, self.meta_dir]:
            dir_path.mkdir(parents=True, exist_ok=True)
            
        # Episode tracking
        self.current_episode = 0
        self.episode_data: List[Dict[str, Any]] = []
        self.episode_frames: List[np.ndarray] = []
        
    def start_episode(self, task_description: str = ""):
        """Start a new episode."""
        self.episode_data = []
        self.episode_frames = []
        self.task_description = task_description
        
    def add_step(
        self,
        observation: Dict[str, Any],
        action: np.ndarray,
        image: Optional[np.ndarray] = None,
    ):
        """
        Add a step to the current episode.
        
        Args:
            observation: Dict with state information
            action: Action array (43 dim for Unitree G1)
            image: RGB image (optional, can be added separately)
        """
        step_data = {
            "observation": observation,
            "action": action.tolist() if isinstance(action, np.ndarray) else action,
            "timestamp": datetime.now().isoformat(),
        }
        self.episode_data.append(step_data)
        
        if image is not None:
            self.episode_frames.append(image)
            
    def add_image(self, image: np.ndarray):
        """Add an image frame to the current episode."""
        self.episode_frames.append(image.copy())
        
    def end_episode(self) -> int:
        """
        End the current episode and save data.
        
        Returns:
            Episode index
        """
        if len(self.episode_data) == 0:
            print("Warning: Empty episode, skipping save")
            return -1
            
        episode_idx = self.current_episode
        self.current_episode += 1
        
        # Save video
        video_path = self.videos_dir / f"episode_{episode_idx:05d}.mp4"
        if len(self.episode_frames) > 0:
            self._save_video(video_path, self.episode_frames)
            
        # Save data as parquet
        data_path = self.data_dir / f"episode_{episode_idx:05d}.parquet"
        self._save_episode_data(data_path, self.episode_data)
        
        # Save metadata
        meta_path = self.meta_dir / f"episode_{episode_idx:05d}.json"
        self._save_episode_metadata(meta_path, episode_idx, self.task_description)
        
        print(f"Saved episode {episode_idx} to {self.output_dir}")
        return episode_idx
        
    def _save_video(self, video_path: Path, frames: List[np.ndarray]):
        """Save frames as MP4 video."""
        if len(frames) == 0:
            return
            
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(
            str(video_path),
            fourcc,
            self.video_fps,
            self.video_resolution
        )
        
        for frame in frames:
            # Resize if needed
            if frame.shape[:2] != self.video_resolution[::-1]:
                frame = cv2.resize(frame, self.video_resolution)
            out.write(frame)
            
        out.release()
        
    def _save_episode_data(self, data_path: Path, episode_data: List[Dict[str, Any]]):
        """Save episode data as parquet file."""
        # Convert to DataFrame format compatible with LeRobot
        records = []
        for i, step in enumerate(episode_data):
            record = {
                "frame_index": i,
                "timestamp": step.get("timestamp", ""),
            }
            
            # Add state information
            obs = step.get("observation", {})
            if "joint_positions" in obs:
                joint_pos = obs["joint_positions"]
                # Split into components (matching modality.json structure)
                record["state.left_arm"] = joint_pos[0:7].tolist() if isinstance(joint_pos, np.ndarray) else joint_pos[0:7]
                record["state.right_arm"] = joint_pos[7:14].tolist() if isinstance(joint_pos, np.ndarray) else joint_pos[7:14]
                record["state.left_hand"] = joint_pos[14:21].tolist() if isinstance(joint_pos, np.ndarray) else joint_pos[14:21]
                record["state.right_hand"] = joint_pos[21:28].tolist() if isinstance(joint_pos, np.ndarray) else joint_pos[21:28]
                
            # Add action information
            action = step.get("action", [])
            if isinstance(action, list) and len(action) >= 28:
                record["action.left_arm"] = action[0:7]
                record["action.right_arm"] = action[7:14]
                record["action.left_hand"] = action[14:21]
                record["action.right_hand"] = action[21:28]
                
            records.append(record)
            
        df = pd.DataFrame(records)
        df.to_parquet(data_path, index=False)
        
    def _save_episode_metadata(self, meta_path: Path, episode_idx: int, task_description: str):
        """Save episode metadata."""
        metadata = {
            "episode_index": episode_idx,
            "task_description": task_description,
            "episode_length": len(self.episode_data),
            "video_path": f"videos/episode_{episode_idx:05d}.mp4",
            "data_path": f"data/episode_{episode_idx:05d}.parquet",
        }
        
        with open(meta_path, "w") as f:
            json.dump(metadata, f, indent=2)
            
    def create_modality_config(self, output_path: Optional[str] = None):
        """Create modality.json file for the dataset."""
        if output_path is None:
            output_path = self.meta_dir / "modality.json"
        else:
            output_path = Path(output_path)
            
        modality_config = {
            "state": {
                "left_arm": {"start": 0, "end": 7},
                "right_arm": {"start": 7, "end": 14},
                "left_hand": {"start": 14, "end": 21},
                "right_hand": {"start": 21, "end": 28},
            },
            "action": {
                "left_arm": {"start": 0, "end": 7},
                "right_arm": {"start": 7, "end": 14},
                "left_hand": {"start": 14, "end": 21},
                "right_hand": {"start": 21, "end": 28},
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
        
        with open(output_path, "w") as f:
            json.dump(modality_config, f, indent=2)
            
        print(f"Created modality config at {output_path}")
