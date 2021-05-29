import gym
from stable_baselines3 import PPO

MAX_TEST_EPISODE_LEN = 18000  # 18k is the default for MineRLObtainDiamond.
TREECHOP_STEPS = 2000  # number of steps to run BC lumberjack for in evaluations.
TEST_MODEL_NAME = "./train/potato.zip"


# !!! Do not change this! This is part of the submission kit !!!
class EpisodeDone(Exception):
    pass


# !!! Do not change this! This is part of the submission kit !!!
class Episode(gym.Env):
    """A class for a single episode."""
    def __init__(self, env):
        self.env = env
        self.action_space = env.action_space
        self.observation_space = env.observation_space
        self._done = False

    def reset(self):
        if not self._done:
            return self.env.reset()

    def step(self, action):
        s, r, d, i = self.env.step(action)
        if d:
            self._done = True
            raise EpisodeDone()
        else:
            return s, r, d, i


class PovOnlyObservation(gym.ObservationWrapper):
    """
    Turns the observation space into POV only, ignoring the inventory. This is needed for stable_baselines3 RL agents,
    as they don't yet support dict observations. The support should be coming soon (as of April 2021).
    See following PR for details:
    https://github.com/DLR-RM/stable-baselines3/pull/243
    """
    def __init__(self, env):
        super().__init__(env)
        self.observation_space = self.env.observation_space['pov']

    def observation(self, observation):
        return observation['pov']


class ActionShaping(gym.ActionWrapper):
    """
    The default MineRL action space is the following dict:
    Dict(attack:Discrete(2),
         back:Discrete(2),
         camera:Box(low=-180.0, high=180.0, shape=(2,)),
         craft:Enum(crafting_table,none,planks,stick,torch),
         equip:Enum(air,iron_axe,iron_pickaxe,none,stone_axe,stone_pickaxe,wooden_axe,wooden_pickaxe),
         forward:Discrete(2),
         jump:Discrete(2),
         left:Discrete(2),
         nearbyCraft:Enum(furnace,iron_axe,iron_pickaxe,none,stone_axe,stone_pickaxe,wooden_axe,wooden_pickaxe),
         nearbySmelt:Enum(coal,iron_ingot,none),
         place:Enum(cobblestone,crafting_table,dirt,furnace,none,stone,torch),
         right:Discrete(2),
         sneak:Discrete(2),
         sprint:Discrete(2))
    It can be viewed as:
         - buttons, like attack, back, forward, sprint that are either pressed or not.
         - mouse, i.e. the continuous camera action in degrees. The two values are pitch (up/down), where up is
           negative, down is positive, and yaw (left/right), where left is negative, right is positive.
         - craft/equip/place actions for items specified above.
    So an example action could be sprint + forward + jump + attack + turn camera, all in one action.
    This wrapper makes the action space much smaller by selecting a few common actions and making the camera actions
    discrete. You can change these actions by changing self._actions below. That should just work with the RL agent,
    but would require some further tinkering below with the BC one.
    """
    def __init__(self, env, camera_angle=10, always_attack=False):
        super().__init__(env)

        self.camera_angle = camera_angle
        self.always_attack = always_attack
        self._actions = [
            [('attack', 1)],
            [('forward', 1)],
            # [('back', 1)],
            # [('left', 1)],
            # [('right', 1)],
            # [('jump', 1)],
            # [('forward', 1), ('attack', 1)],
            # [('craft', 'planks')],
            [('forward', 1), ('jump', 1)],
            [('camera', [-self.camera_angle, 0])],
            [('camera', [self.camera_angle, 0])],
            [('camera', [0, self.camera_angle])],
            [('camera', [0, -self.camera_angle])],
        ]

        self.actions = []
        for actions in self._actions:
            act = self.env.action_space.noop()
            for a, v in actions:
                act[a] = v
            if self.always_attack:
                act['attack'] = 1
            self.actions.append(act)

        self.action_space = gym.spaces.Discrete(len(self.actions))

    def action(self, action):
        return self.actions[action]


def str_to_act(env, actions):
    """
    Simplifies specifying actions for the scripted part of the agent.
    Some examples for a string with a single action:
        'craft:planks'
        'camera:[10,0]'
        'attack'
        'jump'
        ''
    There should be no spaces in single actions, as we use spaces to separate actions with multiple "buttons" pressed:
        'attack sprint forward'
        'forward camera:[0,10]'
    :param env: base MineRL environment.
    :param actions: string of actions.
    :return: dict action, compatible with the base MineRL environment.
    """
    act = env.action_space.noop()
    for action in actions.split():
        if ":" in action:
            k, v = action.split(':')
            if k == 'camera':
                act[k] = eval(v)
            else:
                act[k] = v
        else:
            act[action] = 1
    return act


def get_action_sequence():
    """
    Specify the action sequence for the scripted part of the agent.
    """
    # make planks, sticks, crafting table and wooden pickaxe:
    action_sequence = []
    action_sequence += [''] * 100
    action_sequence += ['craft:planks'] * 4
    action_sequence += ['craft:stick'] * 2
    action_sequence += ['craft:crafting_table']
    action_sequence += ['camera:[10,0]'] * 18
    action_sequence += ['attack'] * 20
    action_sequence += [''] * 10
    action_sequence += ['jump']
    action_sequence += [''] * 5
    action_sequence += ['place:crafting_table']
    action_sequence += [''] * 10

    # bug: looking straight down at a crafting table doesn't let you craft. So we look up a bit before crafting.
    action_sequence += ['camera:[-1,0]']
    action_sequence += ['nearbyCraft:wooden_pickaxe']
    action_sequence += ['camera:[1,0]']
    action_sequence += [''] * 10
    action_sequence += ['equip:wooden_pickaxe']
    action_sequence += [''] * 10

    # dig down:
    action_sequence += ['attack'] * 600
    action_sequence += [''] * 10

    return action_sequence


class MineRLAgent():
    """
    To compete in the competition, you are required to implement the two
    functions in this class:
        - load_agent: a function that loads e.g. network models
        - run_agent_on_episode: a function that plays one game of MineRL

    By default this agent behaves like a random agent: pick random action on
    each step.

    NOTE:
        This class enables the evaluator to run your agent in parallel in Threads,
        which means anything loaded in load_agent will be shared among parallel
        agents. Take care when tracking e.g. hidden state (this should go to run_agent_on_episode).
    """

    def load_agent(self):
        """
        This method is called at the beginning of the evaluation.
        You should load your model and do any preprocessing here.
        THIS METHOD IS ONLY CALLED ONCE AT THE BEGINNING OF THE EVALUATION.
        DO NOT LOAD YOUR MODEL ANYWHERE ELSE.
        """
        # Load up the PPO model.
        self.model = PPO.load(TEST_MODEL_NAME)

    def run_agent_on_episode(self, single_episode_env: Episode):
        """This method runs your agent on a SINGLE episode.

        You should just implement the standard environment interaction loop here:
            obs  = env.reset()
            while not done:
                env.step(self.agent.act(obs))
                ...

        NOTE:
            This method will be called in PARALLEL during evaluation.
            So, only store state in LOCAL variables.
            For example, if using an LSTM, don't store the hidden state in the class
            but as a local variable to the method.

        Args:
            env (gym.Env): The env your agent should interact with.
        """
        env = single_episode_env
        env = PovOnlyObservation(env)
        env = ActionShaping(env, always_attack=True)
        env1 = env.unwrapped

        self.model.set_env(env)

        obs = env.reset()
        done = False
        total_reward = 0
        steps = 0

        action_sequence = get_action_sequence()

        # RL part to get some logs:
        for i in range(TREECHOP_STEPS):
            action = self.model.predict(obs)
            obs, reward, done, _ = env.step(action[0])
            total_reward += reward
            steps += 1
            if done:
                break

        # scripted part to use the logs:
        if not done:
            for i, action in enumerate(action_sequence[:MAX_TEST_EPISODE_LEN - TREECHOP_STEPS]):
                obs, reward, done, _ = env1.step(str_to_act(env1, action))
                total_reward += reward
                steps += 1
                if done:
                    break
