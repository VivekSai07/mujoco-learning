import mujoco
import mujoco.viewer
import numpy as np
import time

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

# --- Controller Parameters for PD Control ---

# Target joint configuration remains the same
target_qpos = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
# We want the robot to be stationary at the target, so target velocity is zero.
target_qvel = np.zeros(7) 

# Proportional gain (Kp) - for position error
kp = 800.0

# Derivative gain (Kd) - for velocity error (damping)
# This value helps to reduce oscillations. Start with a value and tune it.
kd = 113.1

# --- Simulation Loop ---

print("Running Example 2: PD Joint Control")

with mujoco.viewer.launch_passive(model, data) as viewer:
  start_time = time.time()
  while viewer.is_running() and time.time() - start_time < 15:
    step_start = time.time()

    # --- PD Controller Logic ---
    # 1. Calculate position error
    position_error = target_qpos - data.qpos[:7]

    # 2. Calculate velocity error
    velocity_error = target_qvel - data.qvel[:7]

    # 3. Calculate the control signal (torque).
    # It's the sum of the P and D terms.
    torque = (kp * position_error) + (kd * velocity_error)

    # 4. Apply the control signal to the arm actuators
    data.ctrl[:7] = torque

    mujoco.mj_step(model, data)
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