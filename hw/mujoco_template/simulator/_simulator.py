import mujoco
import mujoco.viewer
import numpy as np
import time
from typing import Callable, Optional, Dict, Union
from pathlib import Path
import mediapy as media
import signal
import sys

class ActuatorMotor:
    def __init__(self, torque_range = [-100,100]) -> None:
        self.range = torque_range
        self.dyn = np.array([1, 0, 0])
        self.gain = np.array([1, 0, 0])
        self.bias = np.array([0, 0, 0])

    def __repr__(self) -> str:
        return f"ActuatorMotor(dyn={self.dyn}, gain={self.gain}, bias={self.bias})"

class ActuatorPosition(ActuatorMotor):
    def __init__(self, kp=1, kd=0, position_range = [-100,100]) -> None:
        super().__init__()
        self.range = position_range
        self.kp = kp
        self.kd = kd
        self.gain[0] = self.kp
        self.bias[1] = -self.kp
        self.bias[2] = -self.kd

class ActuatorVelocity(ActuatorMotor):
    def __init__(self, kv=1,  velocity_range = [-100,100]) -> None:
        super().__init__()
        self.range = velocity_range
        self.kv = kv
        self.gain[0] = self.kv
        self.bias[2] = -self.kv

class Simulator:
    def __init__(self, 
                 xml_path: str = "universal_robots_ur5e/scene.xml", 
                 dt: float = 0.002,
                 enable_task_space: bool = False,
                 show_viewer: bool = True,
                 record_video: bool = False,
                 video_path: str = "logs/videos/simulation.mp4",
                 fps: int = 30,
                 width: int = 1280,
                 height: int = 720):
        """Initialize simulator with visualization options."""
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = dt
        self.dt = dt
        
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
        self.frames = []
        self._setup_video_recording()
        
        # Controller related attributes
        self.controller: Optional[Callable] = None
        
        # Cache frequently used IDs
        self._init_robot_properties()
        
        # Hide task space elements if disabled
        if not enable_task_space:
            self._disable_task_space()
        
        # Default actuator configuration
        self._init_default_actuators()
        
        # Handle graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
    
    def _setup_video_recording(self):
        """Setup video recording if enabled."""
        if self.record_video:
            self.video_path.parent.mkdir(parents=True, exist_ok=True)
            
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully."""
        print("\nCaught Ctrl+C, saving video if recording...")
        self._save_video()
        sys.exit(0)
        
    def _save_video(self):
        """Save recorded video if any frames were captured."""
        if self.record_video and self.frames:
            print(f"Saving video to {self.video_path}...")
            media.write_video(str(self.video_path), self.frames, fps=self.fps)
            self.frames = []

    def _init_robot_properties(self):
        """Initialize robot-specific properties."""
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
        
        # Task space properties (always initialize for reference)
        self.site_name = "attachment_site"
        self.site_id = self.model.site(self.site_name).id
        self.mocap_name = "target"
        self.mocap_id = self.model.body(self.mocap_name).mocapid[0]
        
        # Get home position from keyframe
        self.key_name = "home"
        self.key_id = self.model.key(self.key_name).id
        self.q0 = self.model.key(self.key_name).qpos

    def _disable_task_space(self):
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

    def _init_default_actuators(self):
        """Initialize default actuator configuration (torque control)."""
        # Default torque ranges for UR5e joints (in Nm)
        default_ranges = {
            'shoulder_pan': [-150, 150],
            'shoulder_lift': [-150, 150],
            'elbow': [-150, 150],
            'wrist_1': [-28, 28],
            'wrist_2': [-28, 28],
            'wrist_3': [-28, 28]
        }
        
        self.actuator_configs = {}
        for name in self.joint_names:
            self.actuator_configs[name] = ActuatorMotor(torque_range=default_ranges[name])
        self._update_actuators()

    def _update_actuators(self):
        """Update all actuators in the model based on current configuration."""
        for name, actuator in self.actuator_configs.items():
            self.update_actuator(name, actuator)

    def update_actuator(self, actuator_id: Union[str, int], actuator: ActuatorMotor):
        """
        Update specific actuator in the model.
        
        Args:
            actuator_id: Actuator name or ID
            actuator: Actuator configuration object
        """
        model_actuator = self.model.actuator(actuator_id)
        model_actuator.dynprm = np.zeros(len(model_actuator.dynprm))
        model_actuator.gainprm = np.zeros(len(model_actuator.gainprm))
        model_actuator.biasprm = np.zeros(len(model_actuator.biasprm))
        model_actuator.ctrlrange = actuator.range 
        model_actuator.dynprm[:3] = actuator.dyn
        model_actuator.gainprm[:3] = actuator.gain
        model_actuator.biasprm[:3] = actuator.bias

    def configure_actuators(self, config: Dict[str, ActuatorMotor]):
        """
        Configure multiple actuators at once.
        
        Args:
            config: Dictionary mapping actuator names to their configurations
        """
        for name, actuator in config.items():
            if name in self.actuator_configs:
                self.actuator_configs[name] = actuator
            else:
                raise ValueError(f"Unknown actuator name: {name}")
        self._update_actuators()

    def enable_gravity_compensation(self):
        """Enable gravity compensation for the robot links."""
        pass  # Removed functionality
        
    def disable_gravity_compensation(self):
        """Disable gravity compensation for the robot links."""
        pass  # Removed functionality

    def set_controller(self, controller: Callable):
        """Set the controller function to be used in simulation."""
        self.controller = controller
        
    def reset(self):
        """Reset the simulation to initial state."""
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.key_id)

    def get_state(self):
        """Get current robot state."""
        state = {
            'q': self.data.qpos[self.dof_ids].copy(),
            'dq': self.data.qvel[self.dof_ids].copy(),
        }
        
        # Only include task space state if enabled
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

    def step(self, tau: np.ndarray):
        """Execute one simulation step with given control input."""
        # Apply control
        np.clip(tau, *self.model.actuator_ctrlrange.T, out=tau)
        self.data.ctrl[self.actuator_ids] = tau
        
        # Step simulation
        mujoco.mj_step(self.model, self.data)

    def _capture_frame(self):
        """Capture a frame using the renderer."""
        self.renderer.update_scene(self.data)
        pixels = self.renderer.render()
        return pixels.copy()  # Make sure we get a copy of the frame

    def run(self, time_limit: float = None):
        """Run simulation with visualization and recording."""
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