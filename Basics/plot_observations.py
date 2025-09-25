import gymnasium as gym
import matplotlib.pyplot as plt

env = gym.make("Ant-v5")
obs, info = env.reset()

positions = []
velocities = []

for step in range(300):
    action = env.action_space.sample()  # Random action
    obs, reward, terminated, truncated, info = env.step(action)

    qpos = obs[0:27]  # x, y, z positions
    qvel = obs[27:27+84]  # x, y, z velocities

    positions.append(qpos)
    velocities.append(qvel) 
    
    if terminated or truncated:
        obs, info = env.reset()

env. close()

plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.plot(positions)
plt.title("Ant root position (x) over time")
plt.xlabel("Step")
plt.ylabel("Position")

plt.subplot(1, 2, 2)
plt.plot(velocities)
plt.title("Ant root velocity (x) over time")
plt.xlabel("Step")
plt.ylabel("Velocity")

plt.tight_layout()
plt.show()