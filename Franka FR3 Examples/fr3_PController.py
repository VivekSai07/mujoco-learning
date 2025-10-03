import mujoco
import mujoco.viewer
from robot_descriptions import fr3_mj_description
import numpy as np
import time

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

# --- Controller Parameters for Set-Point Control ---

# Define a target joint configuration (in radians) for the 7 arm joints.
# This is a common "ready" or "home" position.
target_qpos = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])

# Proportional gain (Kp). This determines how aggressively the robot
# tries to correct the position error. A higher value means faster response.
# Note: Very high values can lead to instability and oscillations!
kp = 20.0

# --- Simulation Loop ---

print("Running Example 1: Individual Joint Control (Set-Point)")
print(f"Target position: {np.round(target_qpos, 2)}")

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
  # Run the simulation for 10 seconds
  start_time = time.time()
  while viewer.is_running() and time.time() - start_time < 10:
    step_start = time.time()

    # --- P Controller Logic ---
    # 1. Calculate the position error for the 7 arm joints.
    #    data.qpos[:7] gives the current positions of the first 7 joints.
    position_error = target_qpos - data.qpos[:7]

    # 2. Calculate the control signal (torque).
    #    It's simply the error multiplied by the proportional gain.
    torque = kp * position_error

    # 3. Apply the control signal to the first 7 actuators.
    #    data.ctrl[:7] targets the actuators for the arm. We leave the gripper alone.
    data.ctrl[:7] = torque

    # Step the simulation
    mujoco.mj_step(model, data)

    # Render the scene and sync to maintain real-time visualization
    viewer.sync()

    # Optional: Print the error to see it decrease
    if (time.time() - start_time) % 1 < 0.02: # Print roughly every second
        current_error = np.linalg.norm(position_error)
        print(f"Time: {data.time:.2f}s, Error: {current_error:.4f}")

    # Maintain simulation speed
    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

print("Simulation finished.")
