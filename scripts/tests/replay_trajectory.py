from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import replay_trajectory, visualize_trajectory

import argparse

def main(args):
    # Make the robot env
    # env = RobotEnv(robot_type="panda", gripper_action_space="position", action_space='cartesian_position')

    # Replay Trajectory #
    h5_filepath = args.folder + f"{args.num}.h5"
    # replay_trajectory(env, filepath=h5_filepath)
    visualize_trajectory(h5_filepath)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Replay a trajectory")
    parser.add_argument("-f", "--folder", type=str)
    parser.add_argument("-n", "--num", type=int, default=0)
    args = parser.parse_args()
    main(args)