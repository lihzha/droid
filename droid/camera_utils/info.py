from droid.misc.parameters import *

camera_type_dict = {
    hand_camera_id: 3,
    right_camera_id: 0,
    left_camera_id: 2,
}

camera_type_to_string_dict = {
    3: "hand_camera",
    0: "right_camera",
    2: "left_camera",
}

camera_name_dict = {
    hand_camera_id: "Hand Camera",
    right_camera_id: "Right WebCamera",
    left_camera_id: "Left WebCamera",
}


def get_camera_name(cam_id):
    if cam_id in camera_name_dict:
        return camera_name_dict[cam_id]
    return cam_id


def get_camera_type(cam_id):
    if cam_id not in camera_type_dict:
        return None
    type_int = camera_type_dict[cam_id]
    type_str = camera_type_to_string_dict[type_int]
    return type_str
