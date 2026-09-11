"""Small original MJCF models with explicit actuator and frame conventions."""
import mujoco

def arm(dt=0.005):
    return mujoco.MjModel.from_xml_string(f"""
    <mujoco><compiler angle="radian"/><option timestep="{dt}" gravity="0 0 -9.81" integrator="Euler"/>
    <default><joint type="hinge" axis="0 1 0" damping="0.05"/>
    <geom type="capsule" size="0.04" contype="0" conaffinity="0"/></default>
    <worldbody><body name="link1" pos="0 0 2.2"><joint name="j1"/>
    <geom fromto="0 0 0 0 0 -1" mass="1"/>
    <body name="link2" pos="0 0 -1"><joint name="j2"/>
    <geom fromto="0 0 0 0 0 -1" mass="1"/><site name="tip" pos="0 0 -1" size="0.03"/>
    </body></body></worldbody>
    <actuator><motor joint="j1" gear="1" ctrllimited="true" ctrlrange="-100 100"/>
    <motor joint="j2" gear="1" ctrllimited="true" ctrlrange="-100 100"/></actuator></mujoco>""")

def pendulum(dt=0.005):
    return mujoco.MjModel.from_xml_string(f"""
    <mujoco><option timestep="{dt}" gravity="0 0 -9.81"/>
    <worldbody><body name="pendulum"><joint name="hinge" type="hinge" axis="0 1 0"/>
    <inertial pos="0 0 -0.5" mass="1" diaginertia="0.01 0.01 0.01"/>
    <geom type="capsule" fromto="0 0 0 0 0 -1" size="0.03" contype="0" conaffinity="0"/>
    </body></worldbody><actuator><motor joint="hinge" gear="1"/></actuator></mujoco>""")

def cartpole(dt=0.01):
    return mujoco.MjModel.from_xml_string(f"""
    <mujoco><option timestep="{dt}" gravity="0 0 -9.81" integrator="Euler"/>
    <worldbody><body name="cart"><joint name="slider" type="slide" axis="1 0 0"/>
    <geom type="box" size="0.15 0.1 0.08" mass="1" contype="0" conaffinity="0"/>
    <body name="pole"><joint name="hinge" type="hinge" axis="0 1 0"/>
    <geom type="capsule" fromto="0 0 0 0 0 0.8" size="0.025" mass="0.2" contype="0" conaffinity="0"/>
    </body></body></worldbody><actuator>
    <motor joint="slider" gear="1" ctrllimited="true" ctrlrange="-20 20"/>
    </actuator></mujoco>""")

def quadrotor(dt=0.01):
    return mujoco.MjModel.from_xml_string(f"""
    <mujoco><option timestep="{dt}" gravity="0 0 -9.81" integrator="Euler"/>
    <worldbody><body name="quad" pos="0 0 1"><freejoint name="root"/>
    <inertial pos="0 0 0" mass="1" diaginertia="0.02 0.02 0.04"/>
    <geom type="box" size="0.12 0.12 0.025" contype="0" conaffinity="0"/>
    <site name="r1" pos="0.18 0 0"/><site name="r2" pos="0 0.18 0"/>
    <site name="r3" pos="-0.18 0 0"/><site name="r4" pos="0 -0.18 0"/>
    </body></worldbody><actuator>
    <motor site="r1" gear="0 0 1 0 0 0.02" ctrllimited="true" ctrlrange="0 10"/>
    <motor site="r2" gear="0 0 1 0 0 -0.02" ctrllimited="true" ctrlrange="0 10"/>
    <motor site="r3" gear="0 0 1 0 0 0.02" ctrllimited="true" ctrlrange="0 10"/>
    <motor site="r4" gear="0 0 1 0 0 -0.02" ctrllimited="true" ctrlrange="0 10"/>
    </actuator></mujoco>""")

def sliding_block(dt=0.002, friction=0.2, solref=0.02):
    # Translation-only model deliberately excludes rotation and rolling.
    return mujoco.MjModel.from_xml_string(f"""
    <mujoco><option timestep="{dt}" gravity="0 0 -9.81"/>
    <default><geom friction="{friction} 0.005 0.0001" solref="{solref} 1" solimp="0.9 0.95 0.001"/></default>
    <worldbody><geom type="plane" size="3 3 0.1"/>
    <body pos="0 0 0.3"><joint name="x" type="slide" axis="1 0 0"/>
    <joint name="z" type="slide" axis="0 0 1"/>
    <geom name="block" type="box" size="0.1 0.1 0.1" mass="1"/>
    </body></worldbody></mujoco>""")
