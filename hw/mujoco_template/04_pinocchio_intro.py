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
    print(f"Number of joints: {model.njoints}")
    print(f"Number of DOF: {model.nq}")
    print(f"Joint names: {[name for name in model.names[1:] if 'joint' in name]}")
    print(f"Frame names: {[name for name in model.frames]}")
    
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