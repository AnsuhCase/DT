from panda_gym.envs.core import Task
from panda_gym.utils import distance
import panda_gym
import numpy as np


class MyTask(Task):
    def __init__(self, sim):
        super().__init__(sim)
        # create an cube
        # self.sim.create_table(
        # length=1,
        # width=1,
        # height=1,
        # x_offset= 0.0)
        self.sim.create_box(
            body_name="object",
            half_extents=np.array([0.1,0.1,0.1]),
            mass=0,
            position=np.array([2,2,2]),
            specular_color=np.zeros(3),
            rgba_color=np.array([0.95, 0.95, 0.95, 1])
        )

    def reset(self):
        # randomly sample a goal position
        self.goal = np.random.uniform(-1, 10, 3)
        # reset the position of the object
        self.sim.set_base_pose("object", position=np.array([0.2,0.2,0.2]), orientation=np.array([2, 2,2,2]))

    def get_obs(self):
        # the observation is the position of the object
        observation =  self.sim.get_base_position("object")
        # print(observation)
        return observation

    def get_achieved_goal(self):
        # the achieved goal is the current position of the object
        achieved_goal = self.sim.get_base_position("object")
        # print(achieved_goal)
        return achieved_goal

    def is_success(self, achieved_goal, desired_goal, info={}):  # info is here for consistancy
        # compute the distance between the goal position and the current object position
        d = distance(achieved_goal, desired_goal)
        # return True if the distance is < 1.0, and False otherwise
        # print(d)
        return np.array(d < 1.0, dtype=np.bool8)

    def compute_reward(self, achieved_goal, desired_goal, info={}):  # info is here for consistancy
        # for this example, reward = 1.0 if the task is successfull, 0.0 otherwise
        val = self.is_success(achieved_goal, desired_goal, info).astype(np.float32)
        if val:
            print("HURRAH")
        return val


from panda_gym.envs.core import RobotTaskEnv
from panda_gym.pybullet import PyBullet


class MyRobotTaskEnv(RobotTaskEnv):
    """My robot-task environment."""

    def __init__(self, render=False):
        sim = PyBullet(render=render)
        robot = panda_gym.envs.robots.panda.Panda(sim)
        print(sim)
        task = MyTask(sim)
        super().__init__(robot,task)


env = MyRobotTaskEnv(render=True)

observation, info = env.reset()

images = []
for _ in range(1000):
    action = env.action_space.sample() # random action
    print(action)
    observation, reward, terminated, truncated, info = env.step(action)
    images.append(env.render('rgb_array')) # wait a bit to give a realistic temporal rendering

    if terminated or truncated:
        observation, info = env.reset()
        env.render() # wait a bit to give a realistic temporal rendering

# env.close()

from numpngw import write_apng

write_apng('anim.png', images, delay=100)  # real-time rendering = 40 ms between frames

print(images)