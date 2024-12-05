import numpy as np
from simulator import Simulator
from pathlib import Path
from typing import Dict

def task_space_controller(q: np.ndarray, dq: np.ndarray, t: float, desired: Dict) -> np.ndarray:
    """Example task space controller."""
    
    
    kp = np.array([1000, 1000, 1000, 10, 10, 0.1])
    kd = np.array([200, 200, 200, 2, 2, 0.01])
    q0 = np.array([-1.4, -1.3, 1., 0, 0, 0])
    tau = kp * (q0 - q) - kd * dq
    
    # The joint space controller is above, but you can use the task space controller instead
    # The trajectory can be hardcoded here or generated based on the desired task
    desired_position = desired['pos'] # [x_des, y_des, z_des]
    desired_quaternion = desired['quat'] # [w_des, x_des, y_des, z_des]
    print(desired_position, desired_quaternion)
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
    sim.run(time_limit=10000.0)

if __name__ == "__main__":
    main() 