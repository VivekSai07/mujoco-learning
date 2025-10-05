import mujoco
import numpy as np
import time
import mujoco.viewer

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

# --- IK Setup ---

# The name of the body for the end-effector
end_effector_name = "hand"  # You found this name from your inspection!
end_effector_id = model.body(end_effector_name).id

# Target position for the end-effector
target_pos = np.array([0.308, 0.001, 0.595])

# IK Parameters
damping = 0.1      # Damping factor for the DLS method
tolerance = 1e-4   # Tolerance for convergence
max_iterations = 100 # Maximum number of iterations

print(f"--- Running Manual Inverse Kinematics ---")
print(f"Targeting Position for '{end_effector_name}': {target_pos}")

# --- IK Solver Loop ---
# This loop runs OFF-LINE, before the main simulation, to find the solution.

# Start from the default home position
mujoco.mj_resetDataKeyframe(model, data, 0)
qpos_initial = data.qpos.copy() # Store the initial pose

for i in range(max_iterations):
    # 1. Run forward kinematics to get current end-effector position
    mujoco.mj_forward(model, data)
    current_pos = data.body(end_effector_id).xpos
    
    # 2. Calculate the position error (our 'delta_x')
    error = target_pos - current_pos
    
    # 3. Check for convergence
    if np.linalg.norm(error) < tolerance:
        print(f"IK Converged in {i+1} iterations.")
        break
        
    # 4. Calculate the Jacobian
    #    We need two Jacobians: one for translation (jacp) and one for rotation (jacr)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, jacr, end_effector_id)
    # For this problem, we only care about position, so we only use jacp
    # We also only care about the arm joints, so we slice it
    J = jacp[:, :7]
    
    # 5. Solve for the change in joint angles (delta_q) using DLS
    #    Formula: delta_q = J^T * (J * J^T + lambda^2 * I)^-1 * error
    J_T = J.T
    lambda_sq = (damping**2) * np.eye(J.shape[0])
    inv_term = np.linalg.inv(J @ J_T + lambda_sq)
    delta_q = J_T @ inv_term @ error
    
    # 6. Update the joint positions
    data.qpos[:7] += delta_q
    
else: # This 'else' belongs to the 'for' loop, it runs if the loop finishes without 'break'
    print("IK failed to converge within the maximum iterations.")

# Store the final solution
ik_solution_qpos = data.qpos.copy()


# --- Now, use the solution in a simulation with your PD controller ---

# Reset data to the initial position to watch it move
data.qpos[:] = qpos_initial
data.qvel[:] = 0

# PD Controller Gains
kp = 200.0
kd = 28.3

with mujoco.viewer.launch_passive(model, data) as viewer:
  start_time = time.time()
  while viewer.is_running() and time.time() - start_time < 10:
    # --- PD Controller Logic ---
    # The target is now the static IK solution we found
    position_error = ik_solution_qpos[:7] - data.qpos[:7]
    velocity_error = np.zeros(7) - data.qvel[:7]
    torque = (kp * position_error) + (kd * velocity_error)
    data.ctrl[:7] = torque

    mujoco.mj_step(model, data)
    viewer.sync()

print("\nSimulation with IK solution finished.")