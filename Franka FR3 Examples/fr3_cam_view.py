import mujoco
import mujoco.viewer
import numpy as np
import time

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

# --- Controller Parameters ---
target_qpos = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
kp = 100.0
kd = 20.0

print("Running Simulation with Custom Camera View")

# Launch the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:

  # ==================== NEW: CAMERA SETUP ====================
  # Once the viewer is launched, we can access its camera object `viewer.cam`
  
  # 1. Set the distance of the camera from the lookat point.
  #    A smaller value is more zoomed in.
  viewer.cam.distance = 2.5

  # 2. Set the azimuth (horizontal rotation) of the camera in degrees.
  #    0 is from the back, 90 is from the right, 180 is from the front.
  viewer.cam.azimuth = 135

  # 3. Set the elevation (vertical angle) of the camera in degrees.
  #    -90 is looking from directly above, 0 is from the side.
  viewer.cam.elevation = -30

  # 4. Set the 3D point the camera is looking at.
  #    Let's make it look at the base of the robot (body 'link1').
  #    We get the position of 'link1' to center our view.
  lookat_position = data.body('link1').xpos
  viewer.cam.lookat = lookat_position
  # ================= END OF CAMERA SETUP ====================


  # --- Simulation Loop (remains the same) ---
  start_time = time.time()
  while viewer.is_running() and time.time() - start_time < 10:
    step_start = time.time()

    position_error = target_qpos - data.qpos[:7]
    velocity_error = np.zeros(7) - data.qvel[:7]
    torque = (kp * position_error) + (kd * velocity_error)
    data.ctrl[:7] = torque

    mujoco.mj_step(model, data)
    viewer.sync()

    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

print("Simulation finished.")