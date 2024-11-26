from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import replay_trajectory, visualize_trajectory

trajectory_folderpath = "/home/lab/droid/traj_data/"

# Make the robot env
# env = RobotEnv(robot_type="panda", gripper_action_space="position", action_space='cartesian_position')

# Replay Trajectory #
h5_filepath = trajectory_folderpath + "72.h5"
# replay_trajectory(env, filepath=h5_filepath)
visualize_trajectory(h5_filepath)