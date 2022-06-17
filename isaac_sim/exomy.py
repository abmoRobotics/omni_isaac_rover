#launch Isaac Sim before any other imports
#default first two lines in any standalone application
from omni.isaac.kit import SimulationApp

CONFIG = {
    "width": 1280,
    "height": 720,
    "window_width": 1920,
    "window_height": 1080,
    "headless": True,
    "renderer": "RayTracedLighting",
    "display_options": 3286,  # Set display options to show default grid
}

simulation_app = SimulationApp(launch_config=CONFIG) # we can also run as headless.
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
#from omni.isaac.core.articulations import ArticulationView
from exomy_controller import CoolController

# Default Livestream settings
simulation_app.set_setting("/app/window/drawMouse", True)
simulation_app.set_setting("/app/livestream/proto", "ws")
simulation_app.set_setting("/app/livestream/websocket/framerate_limit", 120)
simulation_app.set_setting("/ngx/enabled", False)
enable_extension("omni.kit.livestream.native")


from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from skrl.agents.torch.ppo import PPO, PPO_DEFAULT_CONFIG
import model
from gym.spaces import Box
import math
import gym
import numpy as np
import torch
def square(var):
    return var*var
def load_model(self, model_name, features=[256,160,128]):
    observation_space = self.observation_space
    action_space = self.action_space
    model = m.StochasticActorHeightmap(observation_space=observation_space, action_space=action_space, network_features=features, activation_function="relu")
    checkpoint = torch.load(model_name)
    # model.load_state_dict(checkpoint['state_dict'])
    model.eval()
    model.cuda()
    return model


world = World()
world.scene.add_default_ground_plane()
assets_root_path = "http://100.127.177.125:8080/omniverse://100.127.177.125"
exomy_asset_path = assets_root_path + "/Projects/usd/exomy/exomy_model/exomy_model3.usd"
add_reference_to_stage(usd_path=exomy_asset_path,prim_path="/World/Robot")
exomy_robot =  world.scene.add(
    Robot(
        prim_path="/World/Robot",
        name="Exomy",
        position=np.array([0, 0, 1.0]),
    ))
print("Num of degrees of freedom before first reset: " + str(exomy_robot.num_dof)) # prints None
# Resetting the world needs to be called before querying anything related to an articulation specifically.
# Its recommended to always do a reset after adding your assets, for physics handles to be propagated properly
world.reset()
_my_controller = CoolController()
exomy = world.scene.get_object("Exomy")
#exomy.apply_action(_my_controller.forward(command=[2,2]))

goal = np.array([3.0,0.0])
observation_space = Box(-math.inf,math.inf,(154,))
action_space = Box(-1.0,1.0,(2,))  
oldSteering = 0
oldVelocity = 0

cfg_ppo = PPO_DEFAULT_CONFIG.copy()
policy = {"policy": model.StochasticActorHeightmap(observation_space, action_space, network_features=[256,160,128], encoder_features=[60,20], activation_function="relu"),
                    "value": None}
policy["policy"].load("./450000_policy.pt")
agent = PPO(models=policy,
        memory=None, 
        cfg=cfg_ppo, 
        observation_space=observation_space, 
        action_space=action_space,
        device='cuda:0')

for i in range(111500):
    position, orientation = exomy_robot.get_world_pose()
    direction_vector = np.zeros((2,))
    direction_vector[0] = math.cos(orientation[2] - (math.pi/2)) # x value
    direction_vector[1] = math.sin(orientation[2] - (math.pi/2)) # y value
    goal_vec = goal - np.array([position[0], position[1]])

    heading_diff = math.atan2(goal_vec[0] * direction_vector[1] - goal_vec[1] * direction_vector[0], goal_vec[0] * direction_vector[0] + goal_vec[1] * direction_vector[1])
    print(orientation)
    target_dist = math.sqrt(square(goal - [position[0], position[1]]).sum(-1))
    if target_dist > 0.05:
        DepthInfo = torch.zeros((1,154))
        DepthInfo[0,0] = target_dist/4
        DepthInfo[0,1] = heading_diff/3
        #a[0,2] = msg.robot_rot[2]
        DepthInfo[0,2] = oldVelocity
        DepthInfo[0,3] = oldSteering       

        commands=agent.policy.act(DepthInfo,inference=True)
        velocity = torch.clip(commands[0][0][0], min = -1, max = 1)
        steering = torch.clip(commands[0][0][1], min = -1, max = 1)

        velocity = velocity.item()
        steering = steering.item()

        oldVelocity = velocity
        oldSteering = steering

        lin_vel = float(velocity) * 3
        ang_vel = float(steering) * 3
        vel_cmd, pos_cmd = _my_controller.forward(command=[lin_vel,ang_vel])
        exomy.apply_action(vel_cmd)
        exomy.apply_action(pos_cmd)
    position, orientation = exomy_robot.get_world_pose()
    # linear_velocity = exomy_robot.get_linear_velocity()
    # # will be shown on terminal
    #print("Exomy Position Is : " + str(position))
    # print("Cube's orientation is : " + str(orientation))
    # print("Cube's linear velocity is : " + str(linear_velocity))
    # we have control over stepping physics and rendering in this workflow
    # things run in sync
    world.step(render=True) # execute one physics step and one rendering step

simulation_app.close() # close Isaac Sim