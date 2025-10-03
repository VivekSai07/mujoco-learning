import mujoco
import mujoco.viewer
from robot_descriptions import fr3_mj_description
import numpy as np

# # Load the Franka model from the menagerie
# model = mujoco.MjModel.from_xml_path(fr3_mj_description.MJCF_PATH)
# data = mujoco.MjData(model)

# # Launch the MuJoCo viewer
# with mujoco.viewer.launch_passive(model, data) as viewer:
#   # Keep the viewer open
#   while viewer.is_running():
#     viewer.sync()

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

try:
    # Load the model from the local XML file
    model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
    data = mujoco.MjData(model)

    # Launch the MuJoCo viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
      # Keep the viewer open
      while viewer.is_running():
        t = data.time
        data.ctrl[:] =  np.cos(2 * np.pi * t)
        mujoco.mj_step(model, data)
        print(f"No.of coords: {model.nq}, DOF: {model.nv}, No.of actuators: {model.nu}, No.of bodies: {model.nbody}")
        # print(f"Time: {data.time:.2f} s, Ctrl: {data.ctrl}, data.qpos: {data.qpos}, data.qvel: {data.qvel}")
        viewer.sync()

except FileNotFoundError:
    print(f"Error: Could not find the XML file at '{FRANKA_SCENE_XML}'")
    print("Please make sure you have cloned the mujoco_menagerie repository and updated the MENAGERIE_PATH variable.")