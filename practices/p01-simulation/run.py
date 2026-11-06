"""Load a Menagerie robot and explore it in the interactive MuJoCo viewer."""
import time

import mujoco
import mujoco.viewer
import mujoco_menagerie as mm


def main():
    # Choose ONE robot: leave one line uncommented and comment out the others.
    model = mm.load("unitree_g1")          # Floating-base humanoid.
    # model = mm.load("unitree_go2")         # Floating-base quadruped.

    model.opt.timestep = 1.0 / 500  # 500 Hz physics: 2 ms per step.
    render_every = 10              # Publish a new viewer frame at 50 Hz.

    # These robots use unit-gear hinge actuators: ctrl is joint torque in N m.
    # Replace position-servo gains/biases with direct-drive torque motors.
    for actuator_id in range(model.nu):
        joint_id = model.actuator_trnid[actuator_id, 0]
        if model.jnt_actfrclimited[joint_id]:
            # G1 stores effort limits on joints; its old ctrlrange is in radians.
            model.actuator_ctrlrange[actuator_id] = model.jnt_actfrcrange[joint_id]
            model.actuator_ctrllimited[actuator_id] = True
        # Go2 already has motor torque limits in ctrlrange; retain them.
    model.actuator_dyntype[:] = mujoco.mjtDyn.mjDYN_NONE
    model.actuator_gaintype[:] = mujoco.mjtGain.mjGAIN_FIXED
    model.actuator_biastype[:] = mujoco.mjtBias.mjBIAS_NONE
    model.actuator_gainprm[:] = 0
    model.actuator_gainprm[:, 0] = 1
    model.actuator_biasprm[:] = 0

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
