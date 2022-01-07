import argparse

import pump_env
import os
import time
from stable_baselines3 import PPO, A2C
from sb3_contrib import QRDQN
from stable_baselines3.common.cmd_util import make_vec_env

parser = argparse.ArgumentParser(description='Train high level')
parser.add_argument('--num_train_its', type=int,
                    help='Number of training iterations', default=1000000)
parser.add_argument('--model', type=str, help='DQN or PPO', default='DQN')
parser.add_argument('--filename', type=str, help='Path to world',
                    default='../levels/world.textpb')
parser.add_argument('--save_dir', type=str, help='Director to save state',
                    default='/tmp/pump')
parser.add_argument('--train', type=int, help='Whether to train or not',
                    default=True)
parser.add_argument('--n_steps', type=int, help='Number of steps in final sim',
                    default=0)

args = parser.parse_args()

if args.model == 'DQN':
    env = pump_env.PumpEnv(args.filename)
    single_env = env
    env = make_vec_env(lambda: env, n_envs=1)
    model = QRDQN('MlpPolicy', env, verbose=2,
                  learning_rate = 1e-4,
                  # buffer_size=100000,
                  exploration_initial_eps=1,
                  exploration_fraction=0.9,
                  exploration_final_eps=0.025)  # 0.5, 0.05
else:
    env = pump_env.PumpEnv(filename)
    env = make_vec_env(lambda: env, n_envs=1)
    model = PPO('MlpPolicy', env, verbose=1)
    
if args.train:
    model.learn(args.num_train_its)
    os.makedirs(args.save_dir, exist_ok=True)
    model.save(args.save_dir + "/10k")
    if args.model == 'DQN':
        model.save_replay_buffer(args.save_dir + "/replay_buffer")
else:
    if args.model == 'DQN':
        model = QRDQN.load(args.save_dir + "/10k")
        model.load_replay_buffer(args.save_dir + "/replay_buffer")
    else:
        model.load(args.save_dir + "/10k")

import window
obs = env.reset()
single_env.render_all = True
n_steps = args.n_steps
single_env.callback = lambda: window.draw(single_env.world,1000) and  window.process_events()

for step in range(n_steps):
  action, _ = model.predict(obs, deterministic=False)
  print("Step {}".format(step + 1))
  print("Action: ", action)
  print(obs)
  if False:
      action = [1]
  if True:
      # Somewhat optimal agent.
      if obs[0][1] > obs[0][0]:
          action = [0]
      else:
          action = [2]

  obs, reward, done, info = env.step(action)
  if done:
      obs = env.reset()

  if not window.running: break


