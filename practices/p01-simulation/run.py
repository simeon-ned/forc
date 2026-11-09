"""Load a Menagerie robot and explore it in the interactive MuJoCo viewer."""
import time

import mujoco
import mujoco.viewer
import mujoco_menagerie as mm
import numpy as np

def main():
    # Choose ONE robot: leave one line uncommented and comment out the others.
    # model = mm.load("unitree_g1")          # Floating-base humanoid.
    model = mm.load("universal_robots_ur5e")         # Floating-base quadruped.

    model.opt.timestep = 1.0 / 1000  # 500 Hz physics: 2 ms per step.
    render_every = 20              # Publish a new viewer frame at 50 Hz.

    # Menagerie ships position servos. Match the p02 torque motor:
    # p = a u + b0 + b1 q + b2 v, with a = 1 and b = 0, so ctrl is torque in N m.
    for actuator_id in range(model.nu):
        actuator = model.actuator(actuator_id)
        if model.actuator_forcelimited[actuator_id]:
            torque_range = np.array(model.actuator_forcerange[actuator_id])
        else:
            torque_range = np.array([-100.0, 100.0])
        actuator.dynprm[:] = 0
        actuator.gainprm[:] = 0
        actuator.biasprm[:] = 0
        actuator.ctrlrange[:] = torque_range
        actuator.dynprm[:3] = (1.0, 0.0, 0.0)
        actuator.gainprm[:3] = (1.0, 0.0, 0.0)
        actuator.biasprm[:3] = (0.0, 0.0, 0.0)
        model.actuator_dyntype[actuator_id] = mujoco.mjtDyn.mjDYN_NONE
        model.actuator_gaintype[actuator_id] = mujoco.mjtGain.mjGAIN_FIXED
        model.actuator_biastype[actuator_id] = mujoco.mjtBias.mjBIAS_AFFINE
        model.actuator_ctrllimited[actuator_id] = True

    data = mujoco.MjData(model)
    if model.nkey:
        mujoco.mj_resetDataKeyframe(model, data, 0)  # Supplied initial pose.
    data.ctrl[:] = 0  # Keyframes may contain position targets, not torques.
    mujoco.mj_forward(model, data)

    print(f"MuJoCo {mujoco.__version__}")
    print(f"nq={model.nq} positions, nv={model.nv} velocities, nu={model.nu} actuators")
    print(f"Bodies: {model.nbody}, joints: {model.njnt}, geoms: {model.ngeom}")
    print("Physics: 500 Hz (2 ms); viewer updates: 50 Hz (every 10 steps)")
    print("Torque control: data.ctrl is in N m; initial commands are zero.")
    print("Double-click a robot body to select it; Ctrl + right-drag applies force.")
    print("Ctrl + left-drag applies torque. Press Ctrl+C in this terminal to stop.", flush=True)

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            with viewer.lock():
                viewer.cam.lookat[:] = data.subtree_com[0]
                viewer.cam.distance = 1.8 * model.stat.extent
            viewer.sync()  # Show the initial pose and read GUI input.
            step = 0
            next_step = time.perf_counter()
            while viewer.is_running():
                with viewer.lock():
                    # Later: compute joint torques and assign data.ctrl[:] here.
                    # 
                    # encoder = 14 bit
                    encoder_resolution = 2**14
                    encoder_range = 2*np.pi
                    encoder_to_radians = encoder_range / encoder_resolution
                    y = data.qpos  + np.random.uniform(-1, 0.01, model.nq) * encoder_to_radians
                    dy = data.qvel

                    kp = 100
                    kd = 20
                    y_des = [2.0, -1.5, 1.5, 0.0, 0.0, 0.0]
                    data.ctrl[:] = -kp*(y - y_des) - kd*dy
                    y = data.qpos 
                    dy = data.qvel
                    print('Position:\n', y)
                    mujoco.mj_step(model, data)
                step += 1
                if step % render_every == 0:
                    # Exchange state and mouse forces only every 10 physics steps.
                    # Mouse forces persist between these 50 Hz updates.
                    viewer.sync()
                next_step += model.opt.timestep
                remaining = next_step - time.perf_counter()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    # If overloaded, run slower than real time without a catch-up burst.
                    next_step = time.perf_counter()
    except KeyboardInterrupt:
        print("\nSimulation stopped.")


if __name__ == "__main__":
    main()
