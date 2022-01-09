# The basic environment for training an agent that learns how to pump.
import world
import json
import numpy as np
import gym
import math
import time
from Box2D import *
from gym import spaces

_GRAVITY = 9.81

class PumpEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self, level_name):
    n_actions = 3
    self.action_space = spaces.Discrete(n_actions)
    self.use_intersections = False
    num_obs = 3 if self.use_intersections else 2
    self.observation_space = spaces.Box(low=-1, high=1, shape=(num_obs,), dtype=np.float32)
    self.elapsed_time = 0
    self.level_name = level_name
    self.last_action = 1
    self.last_pos_x = -10000
    self.ground_intersection = None

    self.reset()
    self.callback = None
    super(PumpEnv, self).__init__()

  def render(self, mode='console', delay=0):
    if mode != 'human':
      raise NotImplementedError()

  def step(self, action):
    done = False
    reward = 0

    body = self.world.bodies["Body"]
    bike = self.world.bodies["Bike"]

    joint = self.world.joints["RearMotor"]
    joint.enableLimit = True

    # Make Box2D simulate the physics of our world for one step
    # Keep the same action for a longer period of time.
    TIME_STEP = 1.0 / 60.0
    NUM_STEPS = 6
    for i in range(0, NUM_STEPS):
        if action == 0:
            front_hub = self.world.bodies["FrontHub"]
            # We can only pump down if the pump "joint" isn't completely depressed
            # Also, added later, can only pump down if upright, and if front wheel
            # is close to ground (doesn't do anything really--except for cause the
            # simulation to take longer).
            if ((joint.translation > joint.limits[0]) and
                (body.position.y > bike.position.y) and
                (not self.use_intersections or
                 (self.intersect_ray(front_hub.position) < 0.6))):
                body.ApplyForceToCenter(Box2D.b2Vec2(0, -1), True)
        elif action == 2:
            if (joint.translation >= joint.limits[1] - 0.01):
                body.ApplyForceToCenter(Box2D.b2Vec2(0, body.mass * _GRAVITY), True)
            else:
                body.ApplyForceToCenter(Box2D.b2Vec2(0, 2 * body.mass * _GRAVITY), True)
        
        self.world.world.Step(TIME_STEP, 10, 10)
        self.elapsed_time += TIME_STEP
        if self.callback:
          self.callback()
        if (body.position.x < self.last_pos_x):
          done = True
          break
        else:
          self.last_pos_x = body.position.x

    velocity = self.world.bodies["Bike"].linearVelocity.x

    # Give some small reward for increase in velocity.
    reward = 0.01*(velocity - self.last_velocity)
    # Penalize for choosing action 1 (since it doesn't do anything)
    if action == 1:
      reward -= 0.005
    # And give a small penalty for changing actions.
    reward -= 0.001 * math.pow(action - self.last_action, 2)

    # At the end of the game, give reward proportional to the length of the game
    if done:
        reward += self.elapsed_time / 100.0

    self.last_action = action
    self.last_velocity = velocity

    return self._get_state(), reward, done, {}

  def reset(self):
    self.world = world.World()
    self.world.load(json.loads(open(self.level_name, 'r').read()))
    self.elapsed_time = 0
    self.last_action = 1
    self.last_pos_x = -10000
    self.last_velocity = 0

    # Simulate the 1st second without allowing the agent to take any action
    for i in range(0, 60):
      self.world.world.Step(1.0/60.0, 10, 10)

    # Build up a data structure that allows for fast intersection calculations
    if self.use_intersections:
      if not self.ground_intersection:
        self.ground_intersection = world.FastIntersection(self.world.bodies["Plane"])
      self.ground_intersection.set_body(self.world.bodies["Plane"])

    return self._get_state()

  def intersect_ray(self, pos):
    return self.ground_intersection.intersect_ray(pos)

  def _get_state(self):
    joint = self.world.joints["RearMotor"]

    ground = self.world.bodies["Plane"]
    rear_hub = self.world.bodies["RearHub"]
    front_hub = self.world.bodies["FrontHub"]
    radius = 0.5
    distances = []
    if self.use_intersections:
      distances = [
        self.intersect_ray(front_hub.position),
      ]
    # Some alternative distance signals considered (training didn't work well with these)
      #        self.intersect_ray(rear_hub.position),
      #        self.intersect_ray(front_hub.position + b2Vec2(radius, 0)),
      #        self.intersect_ray(front_hub.position + b2Vec2(2 * radius, 0)) 
    obs = [
        # joint.translation,
        # self.last_action,
        self.world.bodies["FrontHub"].position.y - self.world.bodies["Bike"].position.y,
        self.world.bodies["RearHub"].position.y - self.world.bodies["Bike"].position.y,
        # self.world.bodies["Bike"].linearVelocity.x,
    ] + distances
    
    return obs

