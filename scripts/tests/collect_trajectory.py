from droid.controllers.oculus_controller import VRPolicy
from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import collect_trajectory

import os

# if os.path.exists("/home/lab/droid/traj_data/1.h5"):
#     # Delete the file
#     os.remove("/home/lab/droid/traj_data/1.h5")

# Make the robot env
env = RobotEnv(robot_type="panda", gripper_action_space="position")
# controller = VRPolicy(ip_address="10.51.244.11", right_controller=True, pos_action_gain=6, rot_action_gain=1)
controller = VRPolicy(ip_address=None, right_controller=True, pos_action_gain=6, rot_action_gain=1)

print("Ready")
i = 70
while i < 75:
    collect_trajectory(env, controller=controller, 
                       save_filepath=f"/home/lab/droid/traj_data/{i}.h5",
                       save_images=True, 
                    recording_folderpath="/home/lab/droid/traj_data"
                    )
    input('Press Enter to continue...')
    i += 1
# collect_trajectory(env, controller=controller)
