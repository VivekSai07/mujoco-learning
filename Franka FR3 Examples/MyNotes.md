# Proportional (P)-Controller
1. `position_error = target_qpos - data.qpos[:7]`: This is the heart of feedback control. At every single step of the simulation, you calculate the difference between where you want the joints to be (`target_qpos`) and where they currently are (`data.qpos[:7]`).
2. `torque = kp * position_error`: This is the "Proportional" part of the P-controller. You are calculating a corrective torque that is proportional to the error.
    - If the error is large (the joint is far from its target), a large torque is applied to move it quickly.
    - If the error is small (the joint is close to its target), a small torque is applied for fine-tuning.
3. `data.ctrl[:7] = torque`: You apply these calculated torques to the arm's actuators. Notice you correctly sliced [:7] to control only the arm joints and leave the gripper actuator alone.

## The Role of kp (The Proportional Gain)
The kp value is a "tuning knob" for your controller.
    - **Low `kp`**: The robot will be "gentle" and move slowly towards its target. It might be too slow to overcome gravity or friction effectively.
    - **High `kp`**: The robot will be "aggressive" and respond very quickly. However, if `kp` is too high, the robot will overshoot its target, then over-correct in the other direction, leading to oscillations or even violent instability.

> Finding a good **`kp`** value is a fundamental task in robotics and control engineering.

---

# Improving the Controller (Introducing the "D")
P-controller is great, but the robot arm likely oscillates a bit around the target position before settling. To fix this, we can add a Derivative term, creating a PD Controller.
- The P term acts on the position error (like a spring).
- The D term acts on the velocity error (like a damper or shock absorber). It resists motion, which helps to reduce and eliminate the oscillations.