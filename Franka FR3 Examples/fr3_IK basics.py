import mujoco
import numpy as np
import mediapy as media

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, width=640, height=480) # Specify size for clarity

# --- 1. Setup a nice camera view ---
camera = mujoco.MjvCamera()
mujoco.mjv_defaultFreeCamera(model, camera)
camera.distance = 2.0
camera.azimuth = 135
camera.elevation = -30
camera.lookat = [0.3, 0, 0.5] # Look at the middle of the workspace

# --- 2. Define the Target Joint Configuration ---
target_qpos = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])

# --- 3. Render the INITIAL Position ---
# The model loads in its default "home" position from the XML.
# Let's render this first.
print("Rendering initial 'home' position...")
mujoco.mj_forward(model, data)
initial_hand_pos = data.body('hand').xpos.copy()
renderer.update_scene(data, camera)
initial_plot = renderer.render()

# --- 4. Render the TARGET Position ---
# Now, we manually set the joint angles to our target configuration.
print("Rendering desired 'target' position...")
# ** THE FIX IS HERE: Set qpos to target_qpos **
# We use [:7] because target_qpos has 7 elements for the arm.
data.qpos[:7] = target_qpos

# We MUST call mj_forward() again to update the physics state (body positions, etc.)
# after changing the joint angles.
mujoco.mj_forward(model, data) 
target_hand_pos = data.body('hand').xpos.copy()
renderer.update_scene(data, camera)
target_plot = renderer.render()

# --- 5. Clean up the renderer ---
# ** FIX for the TypeError **
renderer.close()

# --- 6. Display the results ---
print("\n--- Results ---")
print(f"Initial Hand Position => {np.round(initial_hand_pos, 3)}")
print(f"Target Hand Position  => {np.round(target_hand_pos, 3)}\n")

images = {
    'Initial Position': initial_plot,
    'Target Position': target_plot,
}

media.show_images(images)

print("Script finished.")