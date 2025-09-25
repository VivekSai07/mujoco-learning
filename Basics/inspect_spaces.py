import gymnasium as gym

env = gym.make("Ant-v5")

print("Observation space:", env.observation_space)
print("Action space:", env.action_space)

obs, info = env.reset()
print("\nSample observation:", obs[:10])
print("Sample action:", env.action_space.sample())

env.close()