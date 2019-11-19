from gym.envs.registration import register

register(
    id='3d-v6',
    entry_point='walk_environment.envs:FooEnv6',
)
