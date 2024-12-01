"""
Collect human demos using Quest 2 controller

"""
import os
import argparse

from droid.controllers.oculus_controller import VRPolicy
from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import collect_trajectory


def main(args):
    # Make the robot env
    env = RobotEnv(robot_type="panda", gripper_action_space="position")
    # controller = VRPolicy(ip_address="10.51.244.11", right_controller=True, pos_action_gain=6, rot_action_gain=1)
    controller = VRPolicy(ip_address=None, right_controller=True, pos_action_gain=6, rot_action_gain=1)

    # Collect all
    cnt = args.start
    print(f"Collecting {args.total} trajectories starting at {cnt}...")
    while cnt < args.total:
        filepath = os.path.join(args.folder, f"{cnt}.h5")
        save_traj = collect_trajectory(
            env, 
            controller=controller,
            save_filepath=filepath,
            save_images=True, 
            recording_folderpath=args.folder,
        )
        if not save_traj:
            os.remove(filepath) # not removing MP4 files
            input(f"Trial {cnt} was not saved. Press Enter to retry...")
        else:
            cnt += 1
            if cnt < args.total:
                input(f'Press Enter to collect Trial {cnt}...')
                print("=====================================")
    print(f"Finished collecting {args.total} trajectories.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--total", type=int, default=50)
    parser.add_argument("-s", "--start", type=int, default=0)
    parser.add_argument("-f", "--folder", type=str, default="/home/lab/guided-data-collection/data/practice_nov30")
    args = parser.parse_args()
    main(args)
