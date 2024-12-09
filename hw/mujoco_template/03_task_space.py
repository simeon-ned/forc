"""Task space (operational space) control example.

This example demonstrates how to implement task space control for a robot arm,
allowing it to track desired end-effector positions and orientations. The example
uses a simple PD control law but can be extended to more sophisticated controllers.

Key Concepts Demonstrated:
    - Task space control implementation
    - End-effector pose tracking
    - Real-time target visualization
    - Coordinate frame transformations

Example:
    To run this example:
    
    $ python 03_task_space.py

Notes:
    - The target pose can be modified interactively using the MuJoCo viewer
    - The controller gains may need tuning for different trajectories
    - The example uses a simplified task space controller for demonstration
"""

import numpy as np
from simulator import Simulator
from pathlib import Path
from typing import Dict
import os 
import pinocchio as pin

def task_space_controller(q: np.ndarray, dq: np.ndarray, t: float, desired: Dict) -> np.ndarray:
    """Example task space controller."""
    
    
    kp = np.array([1000, 1000, 1000, 10, 10, 0.1])
    kd = np.array([200, 200, 200, 2, 2, 0.01])
    q0 = np.array([0.0, -1.3, 1., 0, 0, 0])
    tau = kp * (q0 - q) - kd * dq
    
    # Convert desired pose to SE3
    desired_position = desired['pos']
    desired_quaternion = desired['quat'] # [w, x, y, z] in MuJoCo format
    desired_quaternion_pin = np.array([*desired_quaternion[1:], desired_quaternion[0]]) # Convert to [x,y,z,w] for Pinocchio
    # Convert to pose and SE3
    desired_pose = np.concatenate([desired_position, desired_quaternion_pin])
    desired_se3 = pin.XYZQUATToSE3(desired_pose)
    print(desired_se3)
    # Get end-effector frame
    ee_frame_id = model.getFrameId("end_effector")
    
    return tau

def main():
    # Create logging directories
    Path("logs/videos").mkdir(parents=True, exist_ok=True)
    
    print("\nRunning task space controller...")
    sim = Simulator(
        xml_path="robots/universal_robots_ur5e/scene.xml",
        enable_task_space=True,
        show_viewer=True,
        record_video=True,
        video_path="logs/videos/03_task_space.mp4",
        fps=30,
        width=1920,
        height=1080
    )
    sim.set_controller(task_space_controller)
    sim.run(time_limit=10.0)

if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    xml_path = os.path.join(current_dir, "robots/universal_robots_ur5e/ur5e.xml")
    model = pin.buildModelFromMJCF(xml_path)
    data = model.createData()
    main() 