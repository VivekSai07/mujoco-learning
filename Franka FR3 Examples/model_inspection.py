import mujoco
import mujoco.viewer
import numpy as np
import time

MENAGERIE_PATH = r"C:/Users/Vivek Sai/Downloads/Mujoco/mujoco_menagerie"
FRANKA_SCENE_XML = f"{MENAGERIE_PATH}/franka_emika_panda/scene.xml"

model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

print("--- Model Inspection ---")

# --- Bodies (Links) ---
print(f"\nNumber of Bodies: {model.nbody}")
print("Body Names:")
for i in range(model.nbody):
    print(f"  ID: {i}, Name: {model.body(i).name}")

# --- Joints ---
print(f"\nNumber of Joints: {model.njnt}")
print("Joint Details:")
for i in range(model.njnt):
    joint_name = model.joint(i).name
    
    # Get the joint's address in the qpos array
    qpos_address = model.jnt_qposadr[i]
    
    # Get the joint limits from model.jnt_range
    # This is an (njnt x 2) array: [lower_limit, upper_limit]
    joint_limits = model.jnt_range[i]
    
    # Check if the joint has limits defined (range[0] > range[1] means no limits)
    limited = model.jnt_limited[i]
    
    print(f"  ID: {i}, Name: {joint_name} (qpos_adr: {qpos_address})")
    if limited:
        # Convert radians to degrees for easier interpretation
        lower_limit_deg = np.rad2deg(joint_limits[0])
        upper_limit_deg = np.rad2deg(joint_limits[1])
        print(f"    Limits: [{joint_limits[0]:.4f}, {joint_limits[1]:.4f}] rad  ->  [{lower_limit_deg:.2f}, {upper_limit_deg:.2f}] deg")
    else:
        print("    Limits: UNLIMITED")


# --- Actuators (Useful for Control) ---
print(f"\nNumber of Actuators: {model.nu}")
print("Actuator Names:")
for i in range(model.nu):
    print(f"  ID: {i}, Name: {model.actuator(i).name}")