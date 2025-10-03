import mujoco
import mujoco.viewer
import numpy as np 
import time

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

# --- Best Practice: Check your model ---
print('Actuator names:', [model.actuator(i).name for i in range(model.nu)])
print('Number of actuators:', model.nu)

# The number of controllable actuators is 8
num_actuators = model.nu 

# Target for the 7 arm joints
arm_target_pos = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# Target for the gripper (e.g., 0.0 for closed, or a positive value for open)
gripper_target = np.array([1.0])

# Combine them into a single target array of the correct size (8,)
target_pos = np.concatenate([arm_target_pos, gripper_target])


# Set the control signal for all actuators
data.ctrl[:num_actuators] = target_pos

with mujoco.viewer.launch_passive(model, data) as viewer:
  start = time.time()
  while viewer.is_running() and time.time() - start < 10:
    step_start = time.time()

    # Step the simulation
    mujoco.mj_step(model, data)

    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)