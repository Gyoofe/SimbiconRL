import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

from baseline_ppo.dist import DiagGaussian
from baseline_ppo.utils import init

class Flatten(nn.Module):
    def forward(self, x):
        return x.view(x.size(0), -1)


class Policy(nn.Module):
    def __init__(self, obs_shape, actions_space, base=None, base_kwargs=None):
        super(Policy, self).__init__()
        if base_kwargs is None:
            base_kwargs = {}
        if base is None:
            base = MLPBase

        self.base = base(obs_shape[0], **base_kwargs)

        ##actions_space is Box
        num_outputs = actions_space.shape[0]
        self.dist = DiagGaussian(self.base.output_size, num_outputs)

    @property
    def is_recurrent(self):
        return self.base.is_recurrent

    @property
    def recurrent_hidden_state_size(self):
        """size of rnn_hx."""
        return self.base.recurrent_hidden_state_size

    def forward(self, inputs, rnn_hxs, masks):
        raise NotImplementedError

    def act(self, inputs, rnn_hxs, masks, deterministic =False):
        value, actor_features, rnn_hxs = self.base(inputs, rnn_hxs, masks)
        dist = self.dist(actor_features)

        if deterministic:
            input("deterministic!")
            action = dist.mode()
        else:
            action = dist.sample()

        action_log_probs = dist.log_prob(action)
        dist_entropy = dist.entropy().mean()

        return value, action, action_log_probs, rnn_hxs

    def get_value(self, inputs, rnn_hxs, masks):
        value, _, _= self.base(inputs, rnn_hxs, masks)
        return value

    def evaluate_actions(self, inputs, rnn_hxs, masks, action):
        value, actor_features, rnn_hxs = self.base(inputs, rnn_hxs, masks)
        dist = self.dist(actor_features)

        action_log_probs = dist.log_prob(action)
        dist_entropy = dist.entropy().mean()

        return value, action_log_probs, dist_entropy, rnn_hxs


class NNBase(nn.Module):

    def __init__(self, recurrent, recurrent_input_size, hidden_size):
        super(NNBase, self).__init__()

        self._hidden_size = hidden_size
        self._recurrent = recurrent

        ##recurrent is False
    @property
    def output_size(self):
        return self._hidden_size

    #def _forward_gru

    @property
    def is_recurrent(self):
        return self._recurrent

    @property
    def recurrent_hidden_state_size(self):
        if self._recurrent:
            return self._hidden_size
        return 1

class MLPBase(NNBase):
    def __init__(self,num_inputs, recurrent = False, hidden_size = 64):
        super(MLPBase, self).__init__(recurrent, num_inputs, hidden_size)


        #@recurrent is False

        init_ = lambda m: init(m, nn.init.orthogonal_, lambda x: nn.init.constant_(x, 0), np.sqrt(2))

        self.actor = nn.Sequential(
                init_(nn.Linear(num_inputs, hidden_size)),
                nn.Tanh(),
                init_(nn.Linear(hidden_size, hidden_size)),
                nn.Tanh()
         )

        
        self.critic = nn.Sequential(
                init_(nn.Linear(num_inputs, hidden_size)),
                nn.Tanh(),
                init_(nn.Linear(hidden_size, hidden_size)),
                nn.Tanh()
        )

        self.critic_linear = init_(nn.Linear(hidden_size, 1))

        self.train()

    def forward(self, inputs, rnn_hxs = None, masks = None):
        x = inputs

        #recurrent is False

        hidden_critic = self.critic(x)
        hidden_actor = self.actor(x)

        return self.critic_linear(hidden_critic), hidden_actor, rnn_hxs
        
