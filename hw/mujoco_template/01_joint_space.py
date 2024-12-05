import numpy as np
from simulator import Simulator
from pathlib import Path

def joint_controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    """Joint space controller with PD control."""
    kp = np.array([1000, 1000, 1000, 10, 10, 0.1])
    kd = np.array([200, 200, 200, 2, 2, 0.01])
    q0 = np.array([-1.4, -1.3, 1., 0, 0, 0])
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