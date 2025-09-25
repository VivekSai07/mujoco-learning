import mujoco
import numpy as np 
import mujoco.viewer

model_path = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie/boston_dynamics_spot/scene.xml"

model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Open the interactive viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Keep running until you close the window
    while viewer.is_running():
        mujoco.mj_step(model, data)   # Step the simulation
        viewer.sync()