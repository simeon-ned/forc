"""Joint space control example for robot manipulation.

This example demonstrates basic joint space control of a UR5e robot arm using
PD (Proportional-Derivative) control. The robot moves from its initial configuration
to a target joint configuration while minimizing position and velocity errors.

Key Concepts Demonstrated:
    - Joint space PD control implementation
    - Real-time simulation visualization
    - Video recording of simulation
    - Basic robot state access and control

Example:
    To run this example:
    
    $ python 01_joint_space.py

Notes:
    - The controller gains (kp, kd) are tuned for the UR5e robot
    - The target configuration (q0) is set to a predefined pose
    - The simulation runs for 10 seconds with real-time visualization
"""

import numpy as np
from simulator import Simulator
from pathlib import Path

def joint_controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    """Joint space PD controller.
    
    Args:
        q: Current joint positions [rad]
        dq: Current joint velocities [rad/s]
        t: Current simulation time [s]
        
    Returns:
        tau: Joint torques command [Nm]
    """
    # Control gains tuned for UR5e
    kp = np.array([1000, 1000, 1000, 10, 10, 0.1])
    kd = np.array([200, 200, 200, 2, 2, 0.01])
    
    # Target joint configuration
    q0 = np.array([-1.4, -1.3, 1., 0, 0, 0])
    
    # PD control law
    tau = kp * (q0 - q) - kd * dq
    return tau

def main():
    # Create logging directories
    Path("logs/videos").mkdir(parents=True, exist_ok=True)
    
    print("\nRunning real-time joint space control...")
    sim = Simulator(
        xml_path="robots/universal_robots_ur5e/scene.xml",
        record_video=True,
        video_path="logs/videos/01_joint_space.mp4",
        width=1920,
        height=1080
    )
    sim.set_controller(joint_controller)
    sim.run(time_limit=10.0)

if __name__ == "__main__":
    main() 