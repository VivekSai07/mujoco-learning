# Model Inspection
- Inspecting the model is critical, as it clarifies the structure of the robot perfectly and knowing the exact names for all the bodies and joints would be usefull.
- knowing the operational range of each joint is essential for:
    1. **Safety**: Preventing the robot from trying to move into configurations that would damage it.
    2. **Inverse Kinematics**: Constraining the IK solver to find solutions that are physically acheivable.
    3. **Realistic Planning**: Ensuring any trajectories generated are valid.

```bash
Number of Joints: 9
Joint Details:
  ID: 0, Name: joint1 (qpos_adr: 0)
    Limits: [-2.8973, 2.8973] rad  ->  [-166.00, 166.00] deg
  ID: 1, Name: joint2 (qpos_adr: 1)
    Limits: [-1.7628, 1.7628] rad  ->  [-101.00, 101.00] deg
  ID: 2, Name: joint3 (qpos_adr: 2)
    Limits: [-2.8973, 2.8973] rad  ->  [-166.00, 166.00] deg
  ID: 3, Name: joint4 (qpos_adr: 3)
    Limits: [-3.0718, -0.0698] rad  ->  [-176.00, -4.00] deg
  ID: 4, Name: joint5 (qpos_adr: 4)
    Limits: [-2.8973, 2.8973] rad  ->  [-166.00, 166.00] deg
  ID: 5, Name: joint6 (qpos_adr: 5)
    Limits: [-0.0175, 3.7525] rad  ->  [-1.00, 215.00] deg
  ID: 6, Name: joint7 (qpos_adr: 6)
    Limits: [-2.8973, 2.8973] rad  ->  [-166.00, 166.00] deg
  ID: 7, Name: finger_joint1 (qpos_adr: 7)
    Limits: [0.0000, 0.0400] rad  ->  [0.00, 2.29] deg
  ID: 8, Name: finger_joint2 (qpos_adr: 8)
    Limits: [0.0000, 0.0400] rad  ->  [0.00, 2.29] deg
```

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

# PD-Controller - Improving the Controller (Introducing the "D")
P-controller is great, but the robot arm likely oscillates a bit around the target position before settling. To fix this, we can add a Derivative term, creating a PD Controller.
- The P term acts on the position error (like a spring).
- The D term acts on the velocity error (like a damper or shock absorber). It resists motion, which helps to reduce and eliminate the oscillations.

- $K_p$ (proportional): stronger Kp → faster correction, smaller steady-state error, but more oscillation and higher torques.
- $K_d$ (derivative): provides damping. Increase Kd to reduce oscillation/overshoot.
- Baseline (rule-of-thumb) for critical-ish damping (assuming unit effective inertia):
```math
K_d \approx 2\sqrt{K_p \times I_{eff}}
```
> - If you don't know $I_{eff}$, start with $K_d \approx 2\sqrt{K_p}$. Use this as a baseline and scale up/down.

- Behavior regimes:
    - $K_d << 2\sqrt{K_p}$ → underdamped (fast but oscillatory).
    - $K_d ≈ 2\sqrt{K_p}$  → critically/near-critically damped (good compromise).
    - $K_d >> 2\sqrt{K_p}$ → overdamped (slower, little/no oscillation).

| Group      |  Kp | Kd low ($0.5\times$) | Kd baseline ($\approx 2\sqrt{K_p}$) | Kd high ($2 \times$ baseline) |
| ---------- | --: | ------------: | ------------------: | --------------------: |
| Very soft  |  20 |           4.5 |                 9.0 |                  18.0 |
| Soft       |  50 |           7.1 |                14.1 |                  28.2 |
| Medium     | 100 |          10.0 |                20.0 |                  40.0 |
| Firm       | 200 |          14.1 |                28.3 |                  56.6 |
| Stiff      | 400 |          20.0 |                40.0 |                  80.0 |
| Very stiff | 800 |          28.3 |                56.6 |                 113.1 |
