import gym
import numpy as np
import f110_gym
import stable_baselines3
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.env_util import make_vec_env

# Wrapper to convert the F1TENTH gym dictionary observation into a flat numpy array
# Stable-Baselines3 works best with flat arrays (e.g., just the LiDAR scans and speed)
class F1TenthSB3Wrapper(gym.Wrapper):
    def __init__(self, env):
        super().__init__(env)
        # Assuming we downsample the 1080 LiDAR rays to 108 for faster training
        self.num_lidar_rays = 108
        
        # Observation space: LiDAR rays + speed + steering angle
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, 
            shape=(self.num_lidar_rays + 2,), dtype=np.float32
        )
        
        # Action space: [Steering, Speed]
        self.action_space = gym.spaces.Box(
            low=np.array([-0.4189, -5.0]), # Min steering rads, min speed m/s
            high=np.array([0.4189, 5.0]),  # Max steering rads, max speed m/s
            dtype=np.float32
        )

    def step(self, action):
        # Clip actions to prevent simulator physics from exploding
        steer = np.clip(action[0], -0.4189, 0.4189)
        speed = np.clip(action[1], -5.0, 5.0)
        
        # The F1TENTH gym expects a 2D numpy array for actions: [[steer, speed]]
        action_array = np.array([[steer, speed]])
        
        obs, step_reward, done, info = self.env.step(action_array)
        
        # Calculate a custom reward: encourage speed, penalize crashing
        reward = speed * 0.1  # Reward for going fast
        if done and info.get('collision'):
            reward -= 100.0  # Massive penalty for hitting walls
            
        flat_obs = self._flatten_obs(obs)
        return flat_obs, reward, done, info

    def reset(self, **kwargs):
        # f110_gym requires a starting pose array
        poses = np.array([[0.0, 0.0, 0.0]])
        kwargs['poses'] = poses
        obs = self.env.reset(**kwargs)
        # F1Tenth gym sometimes returns (obs, info) or just obs depending on version
        if isinstance(obs, tuple):
            obs = obs[0]
        return self._flatten_obs(obs)
        
    def _flatten_obs(self, obs):
        # Downsample LiDAR by taking every 10th ray
        lidar = obs['scans'][0][::10]
        linear_vel = obs['linear_vels_x'][0]
        ang_vel = obs['ang_vels_z'][0]
        
        # Combine into a single 1D array
        flat = np.concatenate([lidar, [linear_vel, ang_vel]])
        
        # Prevent numerical explosion (nan/inf) from breaking Stable Baselines
        flat = np.nan_to_num(flat, posinf=100.0, neginf=-100.0)
        return np.clip(flat, -100.0, 100.0).astype(np.float32)

def make_env():
    # Load a default track map (make sure to download a map like 'levine' or 'vegas')
    env = gym.make('f110_gym:f110-v0', map='d:/projects/Project_LAKSA/scratch/f1tenth_gym/gym/f110_gym/envs/maps/vegas', map_ext='.png', num_agents=1, disable_env_checker=True)
    env = F1TenthSB3Wrapper(env)
    return env

if __name__ == "__main__":
    print("Initializing F1TENTH Gym Environment...")
    
    # Create a vectorized environment (can use SubprocVecEnv for multi-core training)
    vec_env = DummyVecEnv([make_env])
    
    # Setup evaluation callback to save the best model automatically
    eval_callback = EvalCallback(vec_env, best_model_save_path='./logs/',
                                 log_path='./logs/', eval_freq=10000,
                                 deterministic=True, render=False)

    print("Creating PPO Model (MLP Architecture)...")
    # Using a 2-layer MLP (256, 256) which is proven to work well for 1D LiDAR data
    model = PPO("MlpPolicy", vec_env, verbose=1, 
                learning_rate=3e-4, 
                n_steps=2048, 
                batch_size=64, 
                n_epochs=10, 
                gamma=0.99, 
                tensorboard_log="./f1tenth_ppo_tensorboard/")

    print("Starting Training (Target: 1,000,000 steps)")
    try:
        # 1 million steps takes about 1-2 hours on a decent CPU
        model.learn(total_timesteps=1_000_000, callback=eval_callback)
    except KeyboardInterrupt:
        print("Training interrupted manually.")

    # Save the final model
    print("Saving final model to f1tenth_ppo_final.zip")
    model.save("f1tenth_ppo_final")
    
    print("Training complete! You can now deploy 'f1tenth_ppo_final.zip' to the Jetson.")
