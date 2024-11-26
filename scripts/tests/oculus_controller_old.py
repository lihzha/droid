import time

import numpy as np
from oculus_reader.reader import OculusReader

from droid.misc.subprocess_utils import run_threaded_command
from droid.misc.transformations import add_angles, euler_to_quat, quat_diff, quat_to_euler, rmat_to_quat, quat_to_rmat


def vec_to_reorder_mat(vec):
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


def apply_transfer(mat: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    # xyz can be 3dim or 4dim (homogeneous) or can be a rotation matrix
    if len(xyz) == 3:
        xyz = np.append(xyz, 1)
    return np.matmul(mat, xyz)[:3]


class VRPolicy:
    def __init__(
        self,
        right_controller: bool = True,
        ip_address: str = None,
        max_lin_vel: float = 1,
        max_rot_vel: float = 1,
        max_gripper_vel: float = 1,
        spatial_coeff: float = 1,
        pos_action_gain: float = 5,
        rot_action_gain: float = 2,
        gripper_action_gain: float = 1,
        rmat_reorder_rot: list = [2, -1, -3, 4],  # default: [-2, -1, -3, 4]
        rmat_reorder_pos: list = [-1, -2, -3, 4],
    ):
        self.oculus_reader = OculusReader(ip_address=ip_address)
        self.vr_to_global_mat = np.eye(4)  # i.e. self.reference_pose
        self.max_lin_vel = max_lin_vel
        self.max_rot_vel = max_rot_vel
        self.max_gripper_vel = max_gripper_vel
        self.spatial_coeff = spatial_coeff
        self.pos_action_gain = pos_action_gain
        self.rot_action_gain = rot_action_gain
        self.gripper_action_gain = gripper_action_gain
        # self.T_global_quest = vec_to_reorder_mat(rmat_reorder_pos)
        self.T_global_quest = np.array([[-1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        # self.global_to_env_mat_pos = vec_to_reorder_mat(rmat_reorder_pos)
        self.global_to_env_mat = vec_to_reorder_mat(rmat_reorder_rot)
        self.controller_id = "r" if right_controller else "l"
        self.reset_orientation = True
        self.reset_state()

        if right_controller:
            self.T_robot_global = np.array(
                [
                    [np.sqrt(2) / 2, -np.sqrt(2) / 2, 0, 0],
                    [np.sqrt(2) / 2, np.sqrt(2) / 2, 0, 0],
                    [0.0, 0.0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )  # left robot
        else:
            self.T_robot_global = np.array(
                [
                    [-np.sqrt(2) / 2, -np.sqrt(2) / 2, 0, 0],
                    [np.sqrt(2) / 2, -np.sqrt(2) / 2, 0, 0],
                    [0.0, 0.0, 1, 0],
                    [0, 0, 0, 1],
                ]
            )  # right robot (FR3)

        self.T_tcp_ee = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.095], [0, 0, 0, 1]])

        # Start State Listening Thread #
        run_threaded_command(self._update_internal_state)

    def reset_state(self):
        self._state = {
            "poses": {},
            "buttons": {"A": False, "B": False, "X": False, "Y": False},
            "movement_enabled": False,
            "controller_on": True,
        }
        self.update_sensor = True
        self.reset_origin = True
        self.robot_origin = None
        self.vr_origin = None
        self.vr_state = None

    def _update_internal_state(self, num_wait_sec=5, hz=50):
        last_read_time = time.time()
        while True:
            # Regulate Read Frequency #
            time.sleep(1 / hz)

            # Read Controller
            time_since_read = time.time() - last_read_time
            poses, buttons = self.oculus_reader.get_transformations_and_buttons()
            self._state["controller_on"] = time_since_read < num_wait_sec
            if poses == {}:
                continue

            # Determine Control Pipeline #
            toggled = self._state["movement_enabled"] != buttons[self.controller_id.upper() + "G"]
            self.update_sensor = self.update_sensor or buttons[self.controller_id.upper() + "G"]
            self.reset_orientation = self.reset_orientation or buttons[self.controller_id.upper() + "J"]
            self.reset_origin = self.reset_origin or toggled

            # Save Info #
            self._state["poses"] = poses
            self._state["buttons"] = buttons
            self._state["movement_enabled"] = buttons[self.controller_id.upper() + "G"]
            self._state["controller_on"] = True
            last_read_time = time.time()

            # # Update Definition Of "Forward" #
            # stop_updating = self._state["buttons"][self.controller_id.upper() + "J"] or self._state["movement_enabled"]
            # if self.reset_orientation:
            #     rot_mat = np.asarray(self._state["poses"][self.controller_id]).copy()
            #     if stop_updating:
            #         self.reset_orientation = False
            #     # try to invert the rotation matrix, if not possible, then just use the identity matrix
            #     try:
            #         # rot_mat = np.linalg.inv(rot_mat)
            #         np.linalg.inv(rot_mat)
            #     except:
            #         print(f"exception for rot mat: {rot_mat}")
            #         rot_mat = np.eye(4)
            #         self.reset_orientation = True
            #     self.vr_to_global_mat = rot_mat

            # Update Definition Of "Forward" #
            stop_updating = self._state["buttons"][self.controller_id.upper() + "J"] or self._state["movement_enabled"]
            if self.reset_orientation:
                rot_mat = np.asarray(self._state["poses"][self.controller_id])
                if stop_updating:
                    self.reset_orientation = False
                # try to invert the rotation matrix, if not possible, then just use the identity matrix
                try:
                    rot_mat = np.linalg.inv(rot_mat)
                except:
                    print(f"exception for rot mat: {rot_mat}")
                    rot_mat = np.eye(4)
                    self.reset_orientation = True
                self.vr_to_global_mat = rot_mat

    # def _process_reading(self):
    #     rot_mat = np.asarray(self._state["poses"][self.controller_id]).copy()  # in headset frame
    #     # vr_pos = rot_mat[:3, 3].copy()
    #     # vr_quat = rmat_to_quat(rot_mat[:3, :3]).copy()
    #     # rot_mat = self.T_global_quest @ rot_mat @ np.linalg.inv(self.vr_to_global_mat)
    #     vr_pos = self.spatial_coeff * rot_mat[:3, 3]
    #     vr_quat = rmat_to_quat(rot_mat[:3, :3])
    #     # vr_pos = self.spatial_coeff * (self.global_to_env_mat_pos @ rot_mat)[:3, 3]
    #     # vr_quat = rmat_to_quat((self.T_global_quest @ self.vr_to_global_mat @ rot_mat)[:3, :3])
    #     vr_gripper = self._state["buttons"]["rightTrig" if self.controller_id == "r" else "leftTrig"][0]

    #     self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper, "rot_mat": rot_mat}

    def _process_reading(self):
        rot_mat = np.asarray(self._state["poses"][self.controller_id])
        rot_mat = np.linalg.inv(self.global_to_env_mat) @ self.vr_to_global_mat @ rot_mat
        vr_pos = self.spatial_coeff * rot_mat[:3, 3]
        vr_quat = rmat_to_quat(self.T_robot_global[:3, :3] @ rot_mat[:3, :3])
        vr_gripper = self._state["buttons"]["rightTrig" if self.controller_id == "r" else "leftTrig"][0]

        self.vr_state = {"pos": vr_pos, "quat": vr_quat, "gripper": vr_gripper}

    def _limit_velocity(self, lin_vel, rot_vel, gripper_vel):
        """Scales down the linear and angular magnitudes of the action"""
        lin_vel_norm = np.linalg.norm(lin_vel)
        rot_vel_norm = np.linalg.norm(rot_vel)
        gripper_vel_norm = np.linalg.norm(gripper_vel)
        if lin_vel_norm > self.max_lin_vel:
            lin_vel = lin_vel * self.max_lin_vel / lin_vel_norm
        if rot_vel_norm > self.max_rot_vel:
            rot_vel = rot_vel * self.max_rot_vel / rot_vel_norm
        if gripper_vel_norm > self.max_gripper_vel:
            gripper_vel = gripper_vel * self.max_gripper_vel / gripper_vel_norm
        return lin_vel, rot_vel, gripper_vel

    def _calculate_action(self, state_dict, include_info=False):
        # Read Sensor #
        if self.update_sensor:
            self._process_reading()
            self.update_sensor = False

        # Read Observation
        robot_pos = np.array(state_dict["cartesian_position"][:3])
        robot_euler = state_dict["cartesian_position"][3:]
        robot_quat = euler_to_quat(robot_euler)
        robot_gripper = state_dict["gripper_position"]

        # Reset Origin On Release #
        if self.reset_origin:
            self.robot_origin = {"pos": robot_pos, "quat": robot_quat}
            self.vr_origin = {
                "pos": self.vr_state["pos"],
                "quat": self.vr_state["quat"],
                # "rot_mat": self.vr_state["rot_mat"],
            }
            self.reset_origin = False

        # Calculate Positional Action #
        # Calculate Euler Action #
        robot_pos_offset = robot_pos - self.robot_origin["pos"]
        robot_quat_offset = quat_diff(robot_quat, self.robot_origin["quat"])

        # # Method 1
        # tgt_js = self.vr_state["rot_mat"]
        # current_js = self.vr_origin["rot_mat"]
        # tgt_tcp_global = self.T_global_quest @ tgt_js
        # current_tcp_global = self.T_global_quest @ current_js
        # tgt_ee_global = tgt_tcp_global @ self.T_tcp_ee
        # current_ee_global = current_tcp_global @ self.T_tcp_ee
        # tgt_ee_base = self.T_robot_global @ tgt_ee_global
        # current_ee_base = self.T_robot_global @ current_ee_global
        # target_pos_offset = tgt_ee_base[:3, 3] - current_ee_base[:3, 3]
        # pos_action = target_pos_offset - robot_pos_offset
        # current_quat = rmat_to_quat(current_ee_base[:3, :3])
        # target_quat = rmat_to_quat(tgt_ee_base[:3, :3])
        # if np.dot(target_quat, current_quat) < 0.0:
        #     current_quat = -current_quat
        # target_quat_offset = quat_diff(target_quat, current_quat)

        # Method 2
        # target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])

        target_pos_offset_quest = self.vr_state["pos"] - self.vr_origin["pos"]
        # target_pos_offset_global = apply_transfer(self.T_global_quest, target_pos_offset_quest)
        target_pos_offset_global = target_pos_offset_quest
        target_pos_offset_robot = apply_transfer(self.T_robot_global, target_pos_offset_global)
        pos_action = target_pos_offset_robot - robot_pos_offset

        # tgt_js = quat_to_rmat(self.vr_state["quat"])
        # current_js = quat_to_rmat(self.vr_origin["quat"])
        # tgt_tcp_global = self.T_global_quest[:3, :3] @ tgt_js  # in global frame
        # current_tcp_global = self.T_global_quest[:3, :3] @ current_js  # in global frame
        # tgt_ee_global = tgt_tcp_global @ self.T_tcp_ee[:3, :3]
        # current_ee_global = current_tcp_global @ self.T_tcp_ee[:3, :3]
        # tgt_ee_base = self.T_robot_global[:3, :3] @ tgt_ee_global
        # current_ee_base = self.T_robot_global[:3, :3] @ current_ee_global
        # current_quat = rmat_to_quat(current_ee_base)
        # target_quat = rmat_to_quat(tgt_ee_base)
        # if np.dot(target_quat, current_quat) < 0.0:
        #     current_quat = -current_quat
        # target_quat_offset = quat_diff(target_quat, current_quat)

        target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])

        quat_action = quat_diff(target_quat_offset, robot_quat_offset)
        euler_action = quat_to_euler(quat_action)

        # Calculate Gripper Action #
        gripper_action = (self.vr_state["gripper"] * 1.5) - robot_gripper

        # Calculate Desired Pose #
        target_pos = pos_action + robot_pos
        target_euler = add_angles(euler_action, robot_euler)
        target_cartesian = np.concatenate([target_pos, target_euler])
        target_gripper = self.vr_state["gripper"]

        # Scale Appropriately #
        pos_action *= self.pos_action_gain  # in base frame
        euler_action *= self.rot_action_gain  # in base frame

        gripper_action *= self.gripper_action_gain
        lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)

        # Prepare Return Values #
        info_dict = {"target_cartesian_position": target_cartesian, "target_gripper_position": target_gripper}
        action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
        action = action.clip(-1, 1)

        print(action[3:6])

        action[:3] = 0
        # action[3] = 0  # disable rotation around y axis
        action[4] = 0
        action[5] = 0

        # Return #
        if include_info:
            return action, info_dict
        else:
            return action

    # def _calculate_action(self, state_dict, include_info=False):
    #     # Read Sensor #
    #     if self.update_sensor:
    #         self._process_reading()
    #         self.update_sensor = False

    #     # Read Observation
    #     robot_pos = np.array(state_dict["cartesian_position"][:3])
    #     robot_euler = state_dict["cartesian_position"][3:]
    #     robot_quat = euler_to_quat(robot_euler)
    #     robot_gripper = state_dict["gripper_position"]

    #     # Reset Origin On Release #
    #     if self.reset_origin:
    #         self.robot_origin = {"pos": robot_pos, "quat": robot_quat}
    #         self.vr_origin = {"pos": self.vr_state["pos"], "quat": self.vr_state["quat"]}
    #         self.reset_origin = False

    #     # Calculate Positional Action #
    #     robot_pos_offset = robot_pos - self.robot_origin["pos"]
    #     target_pos_offset = self.vr_state["pos"] - self.vr_origin["pos"]
    #     pos_action = target_pos_offset - robot_pos_offset

    #     # Calculate Euler Action #
    #     robot_quat_offset = quat_diff(robot_quat, self.robot_origin["quat"])
    #     target_quat_offset = quat_diff(self.vr_state["quat"], self.vr_origin["quat"])
    #     quat_action = quat_diff(target_quat_offset, robot_quat_offset)
    #     euler_action = quat_to_euler(quat_action)

    #     # Calculate Gripper Action #
    #     gripper_action = (self.vr_state["gripper"] * 1.5) - robot_gripper

    #     # Calculate Desired Pose #
    #     target_pos = pos_action + robot_pos
    #     target_euler = add_angles(euler_action, robot_euler)
    #     target_cartesian = np.concatenate([target_pos, target_euler])
    #     target_gripper = self.vr_state["gripper"]

    #     # Scale Appropriately #
    #     pos_action *= self.pos_action_gain
    #     euler_action *= self.rot_action_gain
    #     gripper_action *= self.gripper_action_gain
    #     lin_vel, rot_vel, gripper_vel = self._limit_velocity(pos_action, euler_action, gripper_action)

    #     # Prepare Return Values #
    #     info_dict = {"target_cartesian_position": target_cartesian, "target_gripper_position": target_gripper}
    #     action = np.concatenate([lin_vel, rot_vel, [gripper_vel]])
    #     action = action.clip(-1, 1)

    #     # Return #
    #     if include_info:
    #         return action, info_dict
    #     else:
    #         return action

    def get_info(self):
        return {
            "success": self._state["buttons"]["A"] if self.controller_id == "r" else self._state["buttons"]["X"],
            "failure": self._state["buttons"]["B"] if self.controller_id == "r" else self._state["buttons"]["Y"],
            "movement_enabled": self._state["movement_enabled"],
            "controller_on": self._state["controller_on"],
        }

    def forward(self, obs_dict, include_info=False):
        if self._state["poses"] == {}:
            action = np.zeros(7)
            if include_info:
                return action, {}
            else:
                return action
        return self._calculate_action(obs_dict["robot_state"], include_info=include_info)
