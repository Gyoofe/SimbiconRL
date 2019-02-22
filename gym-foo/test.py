import gym
import gym_foo
import random
import numpy as np
import DQNc

env = gym.make("foo-v0")
#observation = env.reset()

num_episode = 10000
#env.start_render()

for i in range(num_episode):
    env.reset_model()
    done = 0
    state = env.get_state()
    for q in range(500):
        if done is 0:
            rTorque = DQNc.select_action(DQNc.torch.from_numpy(state).float())
            #print(rTorque)
            #input("rTorque")
            next_state,reward, done,y = env.step(rTorque,q)
            reward = DQNc.torch.tensor([reward*10],device=DQNc.device)
            #print(rTorque)
            #input("K")
            #action = rTorque.long()
            action = rTorque
            #print(action)
            #input("gg")
            DQNc.memory.push(DQNc.torch.from_numpy(state).double(),action.double(),DQNc.torch.from_numpy(next_state).double(), reward)

            state = next_state

            DQNc.optimize_model()
            
        else:
            break
        #env.render()
        #env.stop_render()
    #if done is 0:
        #input("timeOut")
    env.draw_plot(i, y)
    env.reset_model()

    if i % DQNc.TARGET_UPDATE == 0:
        DQNc.target_net.load_state_dict(DQNc.policy_net.state_dict)()

input("end")
