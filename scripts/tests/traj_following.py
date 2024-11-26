from droid.robot_env import RobotEnv
import time
import numpy as np

#     os.remove("/home/lab/droid/traj_data/1.h5")
np.set_printoptions(precision=3, suppress=True)
# Make the robot env
env = RobotEnv(robot_type="panda", gripper_action_space="position", action_space='cartesian_position')

start_state = env.get_observation()['robot_state']['cartesian_position']

print("Stablizing at current state")

print("closing...")
for i in range(50):
    action = np.concatenate([start_state, [1.0]])
    time.sleep(0.1)
    env.step(action)

print("opening...")
for i in range(50):
    if i < 10 and i % 2 == 1:
        action = np.concatenate([start_state, [1.0]])
    action = np.concatenate([start_state, [0.0]])
    time.sleep(0.1)
    if i > 40:
        action = np.concatenate([start_state, [1.0]])

    env.step(action)



# time.sleep(2)

# print("Moving along x-axis")

# current_state = env.get_observation()['robot_state']['cartesian_position']
# # print(current_state[:3])
# action = np.concatenate([current_state, [0.0]])
# # action[0] += 0.02

# for i in range(25):
#     start_time = time.time()
#     action[1] += 0.01
#     action[0] += 0.01
#     time.sleep(0.09)
#     env.step(action)
#     current_state = np.array(env.get_observation()['robot_state']['cartesian_position'])
#     print(current_state[:3])
#     end_time = time.time()
#     # print("Step frequency: ", round(1/(end_time - start_time), 3))


