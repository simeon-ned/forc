"""Step-by-step simulation example with data logging and plotting.

This example shows how to run a robot simulation in headless mode (without visualization),
collect data at each timestep, and generate plots of the results. This is useful for
analyzing controller performance and robot behavior.

Key Concepts Demonstrated:
    - Headless simulation execution
    - Data collection during simulation
    - Result visualization using matplotlib
    - Video recording without real-time display

Example:
    To run this example:
    
    $ python 02_step_by_step.py

The script will generate:
    - Joint position plots in logs/plots/02_positions.png
    - Joint velocity plots in logs/plots/02_velocities.png
    - Simulation video in logs/videos/02_step_by_step.mp4
"""

import numpy as np
from simulator import Simulator
import matplotlib.pyplot as plt
from pathlib import Path

def joint_controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    """Joint space controller with PD control."""
    kp = np.array([1000, 1000, 1000, 10, 10, 0.1])
    kd = np.array([200, 200, 200, 2, 2, 0.01])
    q0 = np.array([-1.4, -1.3, 1., 0, 0, 0])
    tau = kp * (q0 - q) - kd * dq
    return tau

def plot_results(times: np.ndarray, positions: np.ndarray, velocities: np.ndarray):
    """Plot and save simulation results."""
    # Joint positions plot
    plt.figure(figsize=(10, 6))
    for i in range(positions.shape[1]):
        plt.plot(times, positions[:, i], label=f'Joint {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Positions [rad]')
    plt.title('Joint Positions over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig('logs/plots/02_positions.png')
    plt.close()
    
    # Joint velocities plot
    plt.figure(figsize=(10, 6))
    for i in range(velocities.shape[1]):
        plt.plot(times, velocities[:, i], label=f'Joint {i+1}')
    plt.xlabel('Time [s]')
    plt.ylabel('Joint Velocities [rad/s]')
    plt.title('Joint Velocities over Time')
    plt.legend()
    plt.grid(True)
    plt.savefig('logs/plots/02_velocities.png')
    plt.close()

def main():
    # Create logging directories
    Path("logs/videos").mkdir(parents=True, exist_ok=True)
    Path("logs/plots").mkdir(parents=True, exist_ok=True)
    
    print("\nRunning step-by-step simulation...")
    sim = Simulator(
        xml_path="robots/universal_robots_ur5e/scene.xml",
        dt=0.001,
        enable_task_space=False,
        show_viewer=False,
        record_video=True,
        video_path="logs/videos/02_step_by_step.mp4",
        width=1920,
        height=1080
    )
    
    sim.set_controller(joint_controller)
    sim.reset()
    
    # Simulation parameters
    t = 0
    dt = sim.dt
    time_limit = 5.0
    
    # Data collection
    times = []
    positions = []
    velocities = []
    
    while t < time_limit:
        state = sim.get_state()
        times.append(t)
        positions.append(state['q'])
        velocities.append(state['dq'])
        
        tau = joint_controller(q=state['q'], dq=state['dq'], t=t)
        sim.step(tau)
        
        if sim.record_video and len(sim.frames) < sim.fps * t:
            sim.frames.append(sim._capture_frame())
        t += dt
    
    # Process and save results
    times = np.array(times)
    positions = np.array(positions)
    velocities = np.array(velocities)
    
    print(f"Simulation completed: {len(times)} steps")
    print(f"Final joint positions: {positions[-1]}")
    
    sim._save_video()
    plot_results(times, positions, velocities)

if __name__ == "__main__":
    main() 