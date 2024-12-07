"""Introduction to Pinocchio for robot dynamics computation.

This example demonstrates how to use Pinocchio to compute various robot dynamics
quantities, organized into three main sections:
    1. Dynamics computations (mass matrix, gravity, Coriolis effects)
    2. Kinematics computations (forward kinematics, velocities, accelerations)
    3. Frame transformations and spatial algebra operations

Key Concepts Demonstrated:
    - Loading robot models in Pinocchio
    - Computing dynamic quantities
    - Accessing robot state information
    - Working with spatial algebra

Example:
    To run this example:
    
    $ python 04_pinocchio_intro.py

Notes:
    - The example uses random configurations for demonstration
    - All computations are intended for the UR5e robot model
    - Results are printed to console for inspection
    - This script requires proper installation of Pinocchio and its dependencies
"""

import numpy as np
import pinocchio as pin
import os

# Set random seed for reproducibility
np.random.seed(42)

# Load the robot model from scene XML
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir, "robots/universal_robots_ur5e/ur5e.xml")
model = pin.buildModelFromMJCF(xml_path)
data = model.createData()

# Print basic model information
print("\nRobot Model Info:")
print(f"Number of DOF: {model.nq}")

# Generate random configuration
q = np.random.uniform(-np.pi, np.pi, model.nq)  # random joint positions
dq = np.random.uniform(-1, 1, model.nv)  # random joint velocities
ddq = np.random.uniform(-1, 1, model.nv)  # random joint accelerations

print("\nRandom Configuration:")
print(f"Joint positions: {q}")
print(f"Joint velocities: {dq}")
print(f"Joint accelerations: {ddq}")

# =====================
# Dynamics Computations
# =====================
# Compute all dynamics quantities at once
pin.computeAllTerms(model, data, q, dq)

# Mass matrix
M = data.M
print("\nMass Matrix:")
print(M)

# Gravity terms
g = data.g
print("\nGravity Forces:")
print(g)

# Nonlinear effects (Coriolis + gravity)
nle = data.nle
print("\nNon-Linear Effects (Coriolis + Gravity):")
print(nle)

# Compute joint torque regressor
regressor = pin.computeJointTorqueRegressor(model, data, q, dq, ddq)
print("\nJoint Torque Regressor:")
print(f"Shape: {regressor.shape}")  # Should be (nv x 10*nv)
# print(regressor)

# =======================
# Kinematics Computations
# =======================
# Compute forward kinematics
pin.forwardKinematics(model, data, q, dq, ddq)

# Get end-effector frame ID
ee_frame_id = model.getFrameId("wrist_3_link")
print(f"\nEnd-effector frame ID: {ee_frame_id}")

frame = pin.LOCAL
# Calculate kinematics of frames 
pin.updateFramePlacement(model, data, ee_frame_id)

# Get velocities and accelerations
twist = pin.getFrameVelocity(model, data, ee_frame_id, frame)
dtwist = pin.getFrameAcceleration(model, data, ee_frame_id, frame)
J = pin.getFrameJacobian(model, data, ee_frame_id, frame)

# ===========================
# Frame and Spatial Operations
# ===========================
# Get the frame pose
ee_pose = data.oMf[ee_frame_id]
ee_position = ee_pose.translation
ee_rotation = ee_pose.rotation

# Transform desired velocity to end-effector frame
desired_velocity = np.array([0.1, 0.3, 0.1, 0.4, -0.2, 0.1])
desired_twist = ee_pose.actInv(pin.Motion(desired_velocity))

# Working with rotations and logarithms
pos_quat = pin.SE3ToXYZQUAT(ee_pose)
log_R = pin.log3(ee_rotation)

# Print results
print("\nEnd-Effector State:")
print(f"Position: {ee_position}")
print(f"Rotation Matrix:\n{ee_rotation}")
print(f"Twist: {twist}")
print(f"Acceleration: {dtwist}")
print(f"\nGeometric Jacobian (Local):\n{J}")
print(f"\nDesired Twist: {desired_twist}")
print(f"\nQuaternion: {pos_quat[3:]}")
print(f"\nLogarithm of Rotation Matrix:\n{log_R}")
