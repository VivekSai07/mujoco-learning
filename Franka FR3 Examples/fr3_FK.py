import mujoco
import mujoco.viewer
import numpy as np
import time

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

print(model.body('hand').id)
# --- Find the ID of the end-effector body ---
# The name of the body can be found by inspecting the XML file.
# For the Franka Panda from mujoco_menagerie, it's typically 'panda_hand'.
try:
    end_effector_id = model.body('hand').id
except KeyError:
    print("Error: Could not find body 'panda_hand'. Please check the XML file for the correct name.")
    exit()


# PD Controller Gains from your experiments (Medium setting)
kp = 100.0
kd = 20.0

# Target joint configuration
target_qpos = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
target_qvel = np.zeros(7)

print("--- Running Forward Kinematics Example ---")

with mujoco.viewer.launch_passive(model, data) as viewer:
  start_time = time.time()
  
  # Run for 5 seconds to let the robot settle
  while viewer.is_running() and time.time() - start_time < 5:
    step_start = time.time()

    # --- PD Controller (to move the robot to the target joints) ---
    position_error = target_qpos - data.qpos[:7]
    velocity_error = target_qvel - data.qvel[:7]
    torque = (kp * position_error) + (kd * velocity_error)
    data.ctrl[:7] = torque

    mujoco.mj_step(model, data)
    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

  # --- After settling, read and print the FK results ---
  
  # Get the position of the end-effector
  hand_position = data.xpos[end_effector_id]
  
  # Get the orientation of the end-effector as a 3x3 rotation matrix
  hand_orientation_matrix = data.xmat[end_effector_id].reshape(3, 3)

  print("\nRobot has settled at the target joint configuration.")
  print(f"Target Joints (qpos): {np.round(target_qpos, 3)}")
  print("\n--- Forward Kinematics Result ---")
  print(f"Hand Position (x, y, z): {np.round(hand_position, 3)}")
  print(f"Hand Orientation (Rotation Matrix):\n{np.round(hand_orientation_matrix, 3)}")
  
  # Keep the viewer open for a bit longer to observe
  time.sleep(5)

print("\nForward Kinematics example finished.")