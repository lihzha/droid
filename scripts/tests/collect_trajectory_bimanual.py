from droid.controllers.oculus_controller import VRPolicy
from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import collect_trajectory

# Make the robot env
env_left = RobotEnv(robot_type="panda")
env_right = RobotEnv(robot_type="fr3")
controller = VRPolicy(ip_address="10.51.244.11", right_controller=True, pos_action_gain=6, rot_action_gain=1)
# controller = VRPolicy(ip_address=None, right_controller=False)

print("Ready")
# collect_trajectory(env, controller=controller, save_filepath="/home/lab/droid/traj_data/1.h5")
collect_trajectory(env, controller=controller)
