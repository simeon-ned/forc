"""Robot simulator module for control experiments and visualization.

This module provides a simulator interface built on top of MuJoCo for robot control
experiments. It supports different actuator types, real-time visualization, and video
recording capabilities.

Classes:
    ActuatorMotor: Base class for robot actuators
    ActuatorPosition: Position-controlled actuator implementation
    ActuatorVelocity: Velocity-controlled actuator implementation 
    Simulator: Main simulation environment for robot control

Example:
    >>> sim = Simulator(xml_path="robot.xml")
    >>> sim.set_controller(my_controller)
    >>> sim.run(time_limit=10.0)
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
from typing import Callable, Optional, Dict, Union, List, Any
from pathlib import Path
import mediapy as media
import signal
import sys
import os
import pinocchio as pin
from ._model_builder import ActuatorMotor, ActuatorPosition, ActuatorVelocity

class Simulator:
    """MuJoCo-based robot simulator with visualization capabilities.
    
    This class provides a high-level interface for robot simulation with support for:
    - Different actuator types (position, velocity, torque)
    - Real-time visualization
    - Video recording
    - Task space control
    - State monitoring and data collection
    
    Attributes:
        model: MuJoCo model instance
        data: MuJoCo data instance
        dt (float): Simulation timestep
        enable_task_space (bool): Whether task space control is enabled
        show_viewer (bool): Whether to show real-time visualization
        record_video (bool): Whether to record simulation video
        video_path (Path): Path to save recorded video
        fps (int): Video frame rate
        width (int): Video frame width
        height (int): Video frame height
        frames (List[np.ndarray]): Collected video frames
        controller (Optional[Callable]): Control function
    """
    
    def __init__(self, 
                 model: mujoco.MjModel,
                 enable_task_space: bool = False,
                 show_viewer: bool = True,
                 record_video: bool = False,
                 video_path: str = "logs/videos/simulation.mp4",
                 fps: int = 30,
                 width: int = 1280,
                 height: int = 720) -> None:
        """Initialize simulator with visualization options.
        
        Args:
            model: MuJoCo model instance
            enable_task_space: Whether to enable task space control features
            show_viewer: Whether to show real-time visualization
            record_video: Whether to record simulation video
            video_path: Path to save recorded video
            fps: Video frame rate
            width: Video frame width
            height: Video frame height
        """
        # Store model and create data
        self.model = model
        self.data = mujoco.MjData(model)
        self.dt = model.opt.timestep
        
        # Task space control option
        self.enable_task_space = enable_task_space
        
        # Visualization options
        self.show_viewer = show_viewer
        self.record_video = record_video
        self.video_path = Path(video_path)
        self.fps = fps
        
        # Video dimensions
        self.width = width
        self.height = height
        
        # Setup renderer for video recording
        self.renderer = mujoco.Renderer(self.model, width=self.width, height=self.height)
        
        # Video recording
        self.frames: List[np.ndarray] = []
        self._setup_video_recording()
        
        # Controller related attributes
        self.controller: Optional[Callable] = None
        
        # Cache frequently used IDs
        self._init_robot_properties()
        
        # Hide task space elements if disabled
        if not enable_task_space:
            self._disable_task_space()
        
        # Handle graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _setup_video_recording(self) -> None:
        """Setup video recording directory if enabled."""
        if self.record_video:
            self.video_path.parent.mkdir(parents=True, exist_ok=True)
            
    def _signal_handler(self, signum: int, frame: Any) -> None:
        """Handle Ctrl+C gracefully by saving video before exit.
        
        Args:
            signum: Signal number
            frame: Current stack frame
        """
        print("\nCaught Ctrl+C, saving video if recording...")
        self._save_video()
        sys.exit(0)
        
    def _save_video(self) -> None:
        """Save recorded video frames to file if any were captured."""
        if self.record_video and self.frames:
            print(f"Saving video to {self.video_path}...")
            media.write_video(str(self.video_path), self.frames, fps=self.fps)
            self.frames = []

    def _init_robot_properties(self) -> None:
        """Initialize robot-specific properties and cache frequently used IDs."""
        # Joint properties for UR5e
        self.joint_names = [
            "shoulder_pan",
            "shoulder_lift",
            "elbow",
            "wrist_1",
            "wrist_2",
            "wrist_3",
        ]
        self.dof_ids = np.array([self.model.joint(name).id for name in self.joint_names])
        self.actuator_ids = np.array([self.model.actuator(name).id for name in self.joint_names])
        
        # Task space properties
        self.site_name = "attachment_site"
        self.site_id = self.model.site(self.site_name).id
        self.mocap_name = "target"
        self.mocap_id = self.model.body(self.mocap_name).mocapid[0]
        
        # Get home position from keyframe
        self.key_name = "home"
        self.key_id = self.model.key(self.key_name).id
        self.q0 = self.model.key(self.key_name).qpos

    def _disable_task_space(self) -> None:
        """Disable task space elements by making them invisible and inactive."""
        # Hide target mocap body
        target_geom_id = self.model.body(self.mocap_name).geomadr[0]
        self.model.geom_rgba[target_geom_id] = [0, 0, 0, 0]  # Fully transparent
        
        # Hide all sites
        for i in range(self.model.nsite):
            self.model.site_rgba[i] = [0, 0, 0, 0]  # Fully transparent
        
        # Only disable mocap body interactions
        mocap_body_id = self.model.body(self.mocap_name).id
        self.model.body_contype[mocap_body_id] = 0
        self.model.body_conaffinity[mocap_body_id] = 0

    def set_controller(self, controller: Callable) -> None:
        """Set the controller function to be used in simulation.
        
        Args:
            controller: Function that computes control commands
        """
        self.controller = controller
        
    def reset(self) -> None:
        """Reset the simulation to initial state using home keyframe."""
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.key_id)

    def get_state(self) -> Dict[str, np.ndarray]:
        """Get current robot state.
        
        Returns:
            Dictionary containing:
                q: Joint positions
                dq: Joint velocities
                ee_pos: End-effector position (if task space enabled)
                ee_rot: End-effector rotation (if task space enabled)
                desired: Target pose (if task space enabled)
        """
        state = {
            'q': self.data.qpos[self.dof_ids].copy(),
            'dq': self.data.qvel[self.dof_ids].copy(),
        }
        
        if self.enable_task_space:
            state.update({
                'ee_pos': self.data.site(self.site_id).xpos.copy(),
                'ee_rot': self.data.site(self.site_id).xmat.copy(),
                'desired': {
                    'pos': self.data.mocap_pos[self.mocap_id].copy(),
                    'quat': self.data.mocap_quat[self.mocap_id].copy()
                }
            })
            
        return state

    def step(self, tau: np.ndarray) -> None:
        """Execute one simulation step with given control input.
        
        Args:
            tau: Joint torque commands
        """
        # Apply control
        np.clip(tau, *self.model.actuator_ctrlrange.T, out=tau)
        self.data.ctrl[self.actuator_ids] = tau
        
        # Step simulation
        mujoco.mj_step(self.model, self.data)

    def _capture_frame(self) -> np.ndarray:
        """Capture a frame using the renderer.
        
        Returns:
            RGB image array of current scene
        """
        self.renderer.update_scene(self.data)
        pixels = self.renderer.render()
        return pixels.copy()

    def run(self, time_limit: Optional[float] = None) -> None:
        """Run simulation with visualization and recording.
        
        Args:
            time_limit: Maximum simulation time in seconds
            
        Raises:
            AssertionError: If controller is not set
        """
        assert self.controller is not None, "Controller not set!"
        
        viewer = None
        if self.show_viewer:
            viewer = mujoco.viewer.launch_passive(
                model=self.model,
                data=self.data,
                show_left_ui=False,
                show_right_ui=False
            )
            self.reset()
            mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
            
            # Only show site frame if task space is enabled
            viewer.opt.frame = mujoco.mjtFrame.mjFRAME_SITE if self.enable_task_space else mujoco.mjtFrame.mjFRAME_NONE
        
        try:
            t = 0
            start_time = time.perf_counter()
            
            while (not viewer or viewer.is_running()):
                step_start = time.perf_counter()
                
                # Get state and compute control
                state = self.get_state()
                
                # Call controller with or without desired state
                if self.enable_task_space:
                    tau = self.controller(
                        q=state['q'],
                        dq=state['dq'],
                        desired=state['desired'],
                        t=t
                    )
                else:
                    tau = self.controller(
                        q=state['q'],
                        dq=state['dq'],
                        t=t
                    )
                
                # Step simulation
                self.step(tau)
                
                # Update visualization
                if viewer:
                    viewer.sync()
                
                # Record video if enabled
                if self.record_video:
                    if len(self.frames) < self.fps * t:
                        self.frames.append(self._capture_frame())
                
                # Time keeping
                t += self.dt
                if time_limit and t >= time_limit:
                    break
                
                # Real-time synchronization
                real_time = time.perf_counter() - start_time
                if t > real_time:
                    time.sleep(t - real_time)
                elif real_time - t > self.dt:
                    print(f"Warning: Simulation running slower than real-time by {real_time - t:.3f}s")
                    
        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        finally:
            if viewer:
                viewer.close()
            self._save_video()