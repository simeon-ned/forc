"""Introduction to Pinocchio for robot dynamics computation.

This example demonstrates how to use Pinocchio to compute various robot dynamics
quantities including:
    - Mass matrix
    - Gravity compensation terms
    - Coriolis and centrifugal effects
    - Forward kinematics
    - Geometric Jacobian
    - Dynamic parameters regressor

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
    - All computations are performed for the UR5e robot model
    - Results are printed to console for inspection
"""

import numpy as np
import pinocchio as pin
import os

if __name__ == "__main__":
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
    
    print("\nRandom Configuration:")
    print(f"Joint positions: {q}")
    print(f"Joint velocities: {dq}")
    
    # Compute all dynamics quantities at once
    pin.computeAllTerms(model, data, q, dq)
    
    # Mass matrix is now in data.M
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
    
    # Get end-effector frame ID
    ee_frame_id = model.getFrameId("attachment_site")
    print(f"\nEnd-effector frame ID: {ee_frame_id}")
    
    # Forward kinematics already computed by computeAllTerms
    # Get end-effector pose
    ee_pose = data.oMf[ee_frame_id]
    ee_position = ee_pose.translation
    ee_rotation = ee_pose.rotation
    
    print("\nEnd-Effector State:")
    print(f"Position: {ee_position}")
    print(f"Rotation Matrix:\n{ee_rotation}")
    
    # Jacobian already computed by computeAllTerms
    J = pin.getFrameJacobian(model, data, ee_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    print("\nGeometric Jacobian (Local World-Aligned):")
    print(J)
    
    # Compute joint torque regressor
    # This gives us Y(q,dq,ddq) where tau = Y(q,dq,ddq) * phi
    # phi contains the dynamic parameters (masses, inertias, etc.)
    ddq = np.random.uniform(-1, 1, model.nv)  # random joint accelerations
    regressor = pin.computeJointTorqueRegressor(model, data, q, dq, ddq)
    
    print("\nJoint Torque Regressor:")
    print(f"Shape: {regressor.shape}")  # Should be (nv x 10*nv)
    print(regressor)
    