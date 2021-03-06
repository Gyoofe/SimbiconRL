import argparse
import gym
import gym_foo3d
import torch.nn as nn
import torch
#from lib import import model, kfac
from PIL import Image

import numpy as np
import torch
import time

from lib import Model

ENV_ID = "foo-v0"
"""
HID_SIZE = 64
class ModelActor(nn.Module):
    def __init__(self, obs_size, act_size):
        super(ModelActor,self).__init__()

        self.mu = nn.Sequential(
                nn.Linear(obs_size, HID_SIZE),
                nn.Tanh(),
                nn.Linear(HID_SIZE, HID_SIZE),
                nn.Tanh(),
                nn.Linear(HID_SIZE, HID_SIZE),
                nn.Tanh(),

                nn.Linear(HID_SIZE, act_size),
                nn.Tanh(),
        )
        self.logstd = nn.Parameter(torch.zeros(act_size))

    def forward(self, x):
        return self.mu(x)
"""


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--env", required=True, help="enviroment to use")
    parser.add_argument("-m", "--model", required=True, help="Model file to load")
    parser.add_argument("-rd", "--rd", required=True, help="use Random Direction")
    parser.add_argument("-render", "--render", required=True, help="rendering")
    args = parser.parse_args()
    if args.env == "0":
        ENV_ID = "3d-v0"
    elif args.env == "1":
        ENV_ID = "3d-v1"
    elif args.env == "2":
        ENV_ID = "3d-v2"
    elif args.env == "3":
        ENV_ID = "3d-v3"
    elif args.env == "4":
        ENV_ID = "3d-v4"
    elif args.env == "5":
        ENV_ID = "3d-v5"
    elif args.env == "6":
        ENV_ID = "3d-v6"
    elif args.env == "7":
        ENV_ID = "3d-v7"
    else:
        print("use correct enviroment")
        quit()

    if args.rd == "F":
        cDirection = False
    else:
        cDirection = True

    if args.render == "F":
        render = False
    else:
        render = True

    env = gym.make(ENV_ID)
    print(args)
    env.init_dart()
    env.init_sim(cDirection,render)
    #net = Model.ModelActor(env.observation_space.shape[0], env.action_space.shape[0])
    #net.load_state_dict(torch.load(args.model))
    net = torch.load(args.model)
    net.eval() 

    obs = env.reset()
    env.set_linearActionRatio(1)
    if render:
        env.start_render()
        input()
    total_reward = 0.0
    total_steps = 0
    count = 2
    for _ in range(count):
        obs = env.reset()
        episodeReward = 0
        start_time = time.time()
        while True:
            obs_v = torch.cuda.FloatTensor(obs)
            #obs_v = torch.FloatTensor(obs)
            mu_v = net(obs_v)
            action = mu_v.squeeze(dim=0).data.cpu().numpy()
            action = np.clip(action, -1, 1)

            obs, reward, done, _ = env.step(action)
            total_reward += reward
            episodeReward += reward
            total_steps += env.frameskip
            #print("episodeReward: ",episodeReward)
            if done:
                break
        print("episodeReward: " ,episodeReward)
        print("time: %s" %(time.time() - start_time))
    print("In %d steps we got %.3f reward" % (total_steps/count, total_reward/count))

