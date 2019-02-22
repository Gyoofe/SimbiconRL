import copy
import glob
import os
import time
from collections import deque

import gym
import gym_foo
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from baseline_ppo.algo import ppo
from baseline_ppo.model import Policy
from baseline_ppo.storage import RolloutStorage
from baseline_ppo.utils import update_linear_schedule


num_steps = 4096
num_env_steps = 1e+8
num_processes = 1
num_updates = num_env_steps // num_steps // num_processes

torch.manual_seed(time.time())
torch.cuda.manual_seed_all(time.time())

ENV_ID = "foo-v0"
def main():
        torch.set_num_threads(1)
        device = torch.device("cuda:0") 

        envs = gym.make(ENV_ID)
        envs.init_dart()
        envs.init_sim()
        actor_critic = Policy(envs.observation_space.shape, envs.action_space)
        actor_critic.to(device)
        agent =ppo.PPO(actor_critic, 0.2, 10, 512, 0.5, 1e-3, 1e-3,None,5 )
        rollouts = RolloutStorage(num_steps, num_processes, envs.observation_space.shape,
                                envs.action_space,
                                actor_critic.recurrent_hidden_state_size)
        obs = envs.reset()
        rollouts.obs[0].copy_(torch.from_numpy(obs))
        rollouts.to(device)

        episode_rewards = deque(maxlen=10)
        
        envs.start_render()

        start = time.time()
        for j in range(int(num_updates)):
            print("1")
            update_linear_schedule(agent.optimizer, j, num_updates, 1e-3)
            print("2") 
            #agent.clip_param = 0.2 * (1 - j / float(num_updates))
            for step in range(num_steps):
                with torch.no_grad():
                    value, action, action_log_prob, recurrent_hidden_states = actor_critic.act(
                            rollouts.obs[step],
                            rollouts.recurrent_hidden_states[step],
                            rollouts.masks[step])

                #boser reward and next observation
                envAction = action.squeeze(dim=0).data.cpu().numpy()
                obs, reward, done, infos = envs.step(envAction)
                #input("steps!")
                #masks = torch.FloatTensor([[0.0] if done_ else [1.0]
                #                            for done_ in done])
                masks = torch.FloatTensor([[0.0] if done else [1.0]])


                rollouts.insert(torch.from_numpy(obs).to(device), recurrent_hidden_states, action, action_log_prob, value, torch.from_numpy(np.array(reward)).to(device), masks)
            print("3")
            with torch.no_grad():
                next_value = actor_critic.get_value(rollouts.obs[-1],
                                                    rollouts.recurrent_hidden_states[-1],
                                                    rollouts.masks[-1]).detach()
            rollouts.compute_returns(next_value, True, 0.99, 0.95)

            value_loss, action_loss, dist_entropy = agent.update(rollouts)

            rollouts.after_update()

            if (j % 100000 == 0 or j == num_updates -1):
                save_path = os.path.join("saves", "baseline_ppo-samples")
                try:
                    os.makedirs(save_path)
                except OSError:
                    pass

                save_model = actor_critic
                #use cuda
                save_model = copy.deepcopy(actor_critic).cpu()

                torch.save(save_model.state_dict(), "saves%d" % (j))

            total_num_steps = (j+1) * num_processes * num_steps


            if j % 10000 == 0 and len(episode_rewards) > 1:
                print("log interval")

            if j % 500000 and len(episode_rewards) > 1:
                print("eval interval")

            print("plot interval")
            

if __name__ == "__main__":
    main()
