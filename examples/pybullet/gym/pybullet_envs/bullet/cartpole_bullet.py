"""
Classic cart-pole system implemented by Rich Sutton et al.
Copied from https://webdocs.cs.ualberta.ca/~sutton/book/code/pole.c
"""
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import logging
import math
import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import time
import subprocess
import pybullet as p
import pybullet_data
from pkg_resources import parse_version

logger = logging.getLogger(__name__)


class CartPoleBulletEnv(gym.Env):
  metadata = {'render.modes': ['human', 'rgb_array'], 'video.frames_per_second': 50}

  def __init__(self, zeta, renders=False):
    # start the bullet physics server
    self._renders = renders
    self._zeta = zeta
    if (renders):
      p.connect(p.GUI)
    else:
      p.connect(p.DIRECT)

    self.force_mag = 10

    self.seed()
    #    self.reset()
    self.viewer = None
    self._configure()

  def _configure(self, display=None):
    self.display = display

  def seed(self, seed=None):
    self.np_random, seed = seeding.np_random(seed)
    return [seed]

  def step(self, action):
    force = self.force_mag if action[0] == 1 else -self.force_mag

    p.setJointMotorControl2(self.cartpole, 0, p.TORQUE_CONTROL, force=force)
    p.stepSimulation()

    self.state = p.getJointState(self.cartpole, 1)[0:2] + p.getJointState(self.cartpole, 0)[0:2]
    theta, theta_dot, x, x_dot = self.state

    # done =  x < -self.x_threshold \
    #             or x > self.x_threshold \
    #             or theta < -self.theta_threshold_radians \
    #             or theta > self.theta_threshold_radians
    # done = bool(done)
    done = False
    reward = 1.0
    # print("state=",self.state)
    return np.array(self.state), reward, done, {}

  def reset(self, state=None):
    #    print("-----------reset simulation---------------")
    p.resetSimulation()
    self.cartpole = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "cartpole.urdf"),
                               [0, 0, 0])
    cart_mass = self._zeta[0]
    # pole_mass = self._zeta[1]
    p.changeDynamics(self.cartpole, -1, linearDamping=0, angularDamping=0)
    p.changeDynamics(self.cartpole, 0, linearDamping=0, angularDamping=0, mass=cart_mass)
    p.changeDynamics(self.cartpole, 1, linearDamping=0, angularDamping=0)
    self.timeStep = 0.02
    p.setJointMotorControl2(self.cartpole, 1, p.VELOCITY_CONTROL, force=0)
    p.setJointMotorControl2(self.cartpole, 0, p.VELOCITY_CONTROL, force=0)
    p.setGravity(0, 0, -9.8)
    p.setTimeStep(self.timeStep)
    p.setRealTimeSimulation(False)

    # randstate = self.np_random.uniform(low=-0.05, high=0.05, size=(4,))
    # p.resetJointState(self.cartpole, 1, randstate[0], randstate[1])
    # p.resetJointState(self.cartpole, 0, randstate[2], randstate[3])

    if state is not None:
      p.resetJointState(self.cartpole, 1, state[0], state[1])
      p.resetJointState(self.cartpole, 0, state[2], state[3])
    #print("randstate=",randstate)
    self.state = p.getJointState(self.cartpole, 1)[0:2] + p.getJointState(self.cartpole, 0)[0:2]
    #print("self.state=", self.state)
    return np.array(self.state)

  def render(self, mode='human', close=False):
    return
