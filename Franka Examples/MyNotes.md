# Control the robot
**Example**: move arm joints to some values
```python
model = mujoco.MjModel.from_xml_path(FRANKA_SCENE_XML)
data = mujoco.MjData(model)

# Launch the MuJoCo viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Keep the viewer open
    while viewer.is_running():
    t = data.time
    data.ctrl[:] =  np.cos(2 * np.pi * t)
    mujoco.mj_step(model, data)
    # print(f"No.of coords: {model.nq}, DOF: {model.nv}, No.of actuators: {model.nu}, No.of bodies: {model.nbody}")
    # print(f"Time: {data.time:.2f} s, Ctrl: {data.ctrl}, data.qpos: {data.qpos}, data.qvel: {data.qvel}")
    viewer.sync()
```

---

##### **Print Statement - 1: The Static Model (`model`)**: No.of coords: 9, DOF: 9, No.of actuators: 8, No.of bodies: 12
1. `No.of coords: 9` **Why 9?**: Franka robot has 7 joints for the arm + 2 joints for the gripper fingers
2.  `DOF: 9` **Why 9?**: For robots with simple revolute (hinge) and prismatic (sliding) joints, `nv` is always equal to `nq`. Each joint can move independently, so each position coordinate has a corresponding velocity.
3. `No.of actuators: 8` **Why 8?**: Although there are 9 joints, there are only 8 actuators. This is because the two gripper fingers are controlled by a **single actuator**.
    *   **7 actuators** control the 7 arm joints (one motor per joint).
    *   **1 actuator** controls the 2 gripper finger joints simultaneously. This is a very common design for robotic grippers, where the fingers move together in a coupled or "mimicked" motion.
4. `No.of bodies: 12` **Why 12?**: This includes the static "world" body, the robot's base link, the 7 arm links, the "hand" link, and the two finger links. The exact count depends on how the XML file is structured.


##### **Print Statement - 2: The Dynamic State (data)**
1. `Time: 2.44 s`: The simulation has been running for 2.44 seconds.
2. `Ctrl: [-0.92507721 ... -0.92507721]`: This is an array with **8** elements. You are setting these values directly in your code with `data.ctrl[:] = np.cos(2 * np.pi * t)`. At time `t=2.44`, the value of `cos(2 * pi * 2.44)` is approximately -0.925. Since you are applying this to all 8 actuators, every element in the array has this same value.
3. `data.qpos: [ 4.12... -3.01...]`: This is an array with **9** elements. For the Franka Panda robot from `mujoco_menagerie`, these 9 positions correspond to:
    *   **Elements 0-6:** The angles (in radians) of the 7 joints of the robot's arm.
    *   **Elements 7-8:** The positions of the two gripper fingers (the "panda_finger_joint1" and "panda_finger_joint2").
4. `data.qvel: [ 1.80... -2.89...]`:  This is an array with **9** elements. Each element is the current velocity of the corresponding joint in `qpos`. The values are non-zero and quite large because your cosine control signal is causing the joints to oscillate back and forth rapidly.


### Summary
| Property | From `model` | Size | From `data` | Explanation for Franka Panda |
| :--- | :--- | :--- | :--- | :--- |
| **Position** | `model.nq` | 9 | `data.qpos` | 7 arm joint angles + 2 gripper finger positions |
| **Velocity** | `model.nv` | 9 | `data.qvel` | 7 arm joint velocities + 2 gripper finger velocities |
| **Control** | `model.nu` | 8 | `data.ctrl` | 7 arm motors + **1 gripper motor** (controlling both fingers) |
