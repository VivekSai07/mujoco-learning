import gymnasium as gym

# Create an Ant robot environment (requires mujoco installed)
env = gym.make("Ant-v5", render_mode="human")

obs, info = env.reset()

for _ in range(500):  # Run 500 steps
    action = env.action_space.sample()  # Random action
    obs, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        obs, info = env.reset()

env.close()
