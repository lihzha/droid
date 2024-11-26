from copy import deepcopy

import cv2
import numpy as np

from droid.misc.parameters import hand_camera_id
from droid.misc.time import time_ms
import time
import pyrealsense2 as rs

import cv2

def get_available_camera_indices(max_cameras=10):
    available_cameras = []
    for camera_id in range(max_cameras):
        cap = cv2.VideoCapture(camera_id)
        if cap.isOpened():
            available_cameras.append(camera_id)
            cap.release()
    return available_cameras


def gather_web_cameras():
    all_web_cameras = []

    cameras = get_available_camera_indices(3)

    for cam in cameras:
        cam = Camera(cam, resolution=(96, 96))
        all_web_cameras.append(cam)

    return all_web_cameras

def gather_cameras():
    web_cameras = gather_web_cameras()
    hand_camera = RSCamera(camera_idx=hand_camera_id, resolution=(96, 96))
    return web_cameras + [hand_camera]


resize_func_map = {"cv2": cv2.resize, None: None}

# standard_params = dict(
#     depth_minimum_distance=0.1, camera_resolution=sl.RESOLUTION.HD720, depth_stabilization=False, camera_fps=60, camera_image_flip=sl.FLIP_MODE.OFF
# )

# advanced_params = dict(
#     depth_minimum_distance=0.1, camera_resolution=sl.RESOLUTION.HD2K, depth_stabilization=False, camera_fps=15, camera_image_flip=sl.FLIP_MODE.OFF
# )



class Camera:
    def __init__(self, camera_idx, resolution=(0, 0)):
        # Save Parameters
        self.serial_number = str(camera_idx)
        self.is_hand_camera = self.serial_number == hand_camera_id
        self.high_res_calibration = False
        self.current_mode = None
        self._extrinsics = {}
        self.latency = 0  # Can be estimated or calculated if required
        self.resizer_resolution = resolution
        self.skip_reading = False  # Example flag
        self.image = True  # Enable reading image
        self._video_writer = None
        # Open Camera 

    ### Intrinsics Method ###
    def get_intrinsics(self):
        """Return basic camera intrinsics, width, height, and fps."""
        return deepcopy(self._intrinsics)

    ### Basic Camera Utilities ###
    def _process_frame(self, frame):
        """Resize the frame if needed."""
        if self.resizer_resolution == (0, 0):  # If no resizing needed
            return frame
        # Resize the frame to the set resolution
        return cv2.resize(frame, self.resizer_resolution)

    ### Read Camera Frame ###
    def read_camera(self):
        """Read the camera frame, capture timestamps, and return data."""
        # Skip if Read Unnecessary
        if self.skip_reading:
            return {}, {}

        # Read Camera Frame
        timestamp_dict = {self.serial_number + "_read_start": time_ms()}
        ret, frame = self._cam.read()
        if self._video_writer is not None:
            self.add_frame(ret, frame)

        if not ret:  # Failed to grab frame
            print("Error: Could not read the frame.")
            return None

        timestamp_dict[self.serial_number + "_read_end"] = time_ms()

        # Benchmark Latency (Using time since webcam doesn't provide timestamps)
        received_time = timestamp_dict[self.serial_number + "_read_end"]
        timestamp_dict[self.serial_number + "_frame_received"] = received_time
        timestamp_dict[self.serial_number + "_estimated_capture"] = received_time - self.latency

        # Return Data
        data_dict = {}

        if self.image and frame is not None:
            # Process the frame if needed (resize)
            processed_frame = self._process_frame(frame)
            data_dict["image"] = {self.serial_number: processed_frame}

        return data_dict, timestamp_dict

    ### Release Camera ###
    def release(self):
        """Release the camera resource."""
        self._cam.release()

    def disable_camera(self):
        if self.current_mode == "disabled":
            return
        if hasattr(self, "_cam"):
            self._current_params = None
            self.release()
        self.current_mode = "disabled"
        print("Setting Disabled Mode")

    def is_running(self):
        return self.current_mode != "disabled"

    def enable_advanced_calibration(self):
        self.high_res_calibration = True

    def disable_advanced_calibration(self):
        self.high_res_calibration = False

    ### Recording Utilities ###
    def start_recording(self, filename):
        assert filename.endswith(".mp4") or filename.endswith(".avi"), "File must be .mp4 or .avi"
        
        # Choose codec based on file extension
        if filename.endswith(".mp4"):
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Appropriate codec for mp4
        else:
            fourcc = cv2.VideoWriter_fourcc(*"XVID")  # Codec for avi

        frame_width = int(self._cam.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(self._cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = 30.0  # Frame rate (you can adjust it if needed)

        # Initialize video writer
        self._video_writer = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))

        print("Starting Recording")

    def stop_recording(self):
        if hasattr(self, "_video_writer"):
            print("Stopping Recording")
            self._video_writer.release()
            self._video_writer = None

    def add_frame(self, ret, frame):
        """Capture a frame from the camera and add it to the video."""
        if self._video_writer is None:
            raise RuntimeError("Video recording has not started yet.")

        if ret:
            self._video_writer.write(frame)  # Add the captured frame to the video
        else:
            print("Failed to capture frame from camera.")

    def set_reading_parameters(
        self,
        image=True,
        resolution=(0, 0),
        resize_func=None,
    ):
        # Non-Permenant Values #
        self.traj_image = image
        self.traj_resolution = resolution

        # Permenant Values #
        self.resize_func = resize_func_map[resize_func]

    ### Camera Modes ###
    def set_calibration_mode(self):
        # Set Parameters #
        self.image = True  # Always capture images in calibration mode
        self.concatenate_images = False  # Web cameras don’t support stereo images
        self.skip_reading = False  # Always read frames

        # Use basic resolution and resizer if needed
        # self.resizer_resolution = (0, 0)

        # Set Mode (No need to configure camera like in ZED)
        if self.high_res_calibration:
            self._configure_camera((1920, 1080))  # Example high-res calibration
        else:
            self._configure_camera((640, 480))  # Standard calibration
        self.current_mode = "calibration"
        print("Setting Calibration Mode")

    def set_trajectory_mode(self):
        # Set Parameters #
        self.image = self.traj_image
        self.skip_reading = not self.image  # For webcam, we only care about the image

        if self.resize_func is None:
            # Use trajectory resolution if resize function is not provided
            # self.resizer_resolution = (0, 0)  # No resizing by default
            pass
        else:
            # Apply resizing if a resize function is provided
            self.resizer_resolution = self.traj_resolution

        # Set Mode
        self._configure_camera()
        self.current_mode = "trajectory"
        print("Setting Trajectory Mode")
        
    def _configure_camera(self, init_params=None):

        # Close Existing Camera
        self.disable_camera()

        # Initialize Camera using OpenCV
        self._cam = cv2.VideoCapture(int(self.serial_number))

        if not self._cam.isOpened():
            raise RuntimeError("Camera Failed To Open")

        # Set Width and Height
        self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Save Intrinsics with updated width and height
        self.latency = int(2.5 * (1e3 / self._cam.get(cv2.CAP_PROP_FPS)))  # Estimate latency

        # Get and Save Intrinsics (width, height, fps)
        self._intrinsics = {
            "width": int(self._cam.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(self._cam.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": int(self._cam.get(cv2.CAP_PROP_FPS)),
        }

        print(self._intrinsics)

        print("Opening Camera: ", self.serial_number)

        # i = 0
        # while i<100:
        #     # Read a frame from the webcam
        #     ret, frame = self._cam.read()
            
        #     # Check if the frame was captured successfully
        #     if not ret:
        #         print("Failed to capture image.")
        #         break

        #     # Display the frame
        #     cv2.imshow('Webcam Feed', frame)

        #     # Press 'q' to exit the loop
        #     if cv2.waitKey(1) & 0xFF == ord('q'):
        #         break
        #     i+=1

    # ### Calibration Utilities ###
    # def _process_intrinsics(self, params):
    #     intrinsics = {}
    #     intrinsics["cameraMatrix"] = np.array([[params.fx, 0, params.cx], [0, params.fy, params.cy], [0, 0, 1]])
    #     intrinsics["distCoeffs"] = np.array(list(params.disto))
    #     return intrinsics


class RSCamera(Camera):
    def __init__(self, camera_idx, resolution=(320, 240)):
        super().__init__(camera_idx=camera_idx, resolution=resolution)
        
    ### Read Camera Frame ###
    def read_camera(self):
        """Read the camera frame, capture timestamps, and return data."""
        # Skip if Read Unnecessary
        if self.skip_reading:
            return {}, {}

        # Read Camera Frame
        timestamp_dict = {self.serial_number + "_read_start": time_ms()}
        frame = self.pipeline.wait_for_frames()
        color_frame = frame.get_color_frame()
        if not color_frame:
            ret = False
        else:
            ret = True
            color_frame = np.asanyarray(color_frame.get_data())
        if self._video_writer is not None:
            self.add_frame(ret, color_frame)

        if not ret:  # Failed to grab frame
            print("Error: Could not read the frame.")
            return None

        timestamp_dict[self.serial_number + "_read_end"] = time_ms()

        # Benchmark Latency (Using time since webcam doesn't provide timestamps)
        received_time = timestamp_dict[self.serial_number + "_read_end"]
        timestamp_dict[self.serial_number + "_frame_received"] = received_time
        timestamp_dict[self.serial_number + "_estimated_capture"] = received_time - self.latency

        # Return Data
        data_dict = {}

        if self.image and color_frame is not None:
            # Process the frame if needed (resize)
            processed_frame = self._process_frame(color_frame)
            data_dict["image"] = {self.serial_number: processed_frame}

        return data_dict, timestamp_dict

    ### Release Camera ###
    def release(self):
        """Release the camera resource."""
        self.pipeline.stop()

    def disable_camera(self):
        if self.current_mode == "disabled":
            return
        if hasattr(self, "pipeline"):
            self._current_params = None
            self.release()
        self.current_mode = "disabled"
        print("Setting Disabled Mode")

    ### Recording Utilities ###
    def start_recording(self, filename):
        assert filename.endswith(".mp4") or filename.endswith(".avi"), "File must be .mp4 or .avi"
        
        # Choose codec based on file extension
        if filename.endswith(".mp4"):
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Appropriate codec for mp4
        else:
            fourcc = cv2.VideoWriter_fourcc(*"XVID")  # Codec for avi

        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        frame_width = color_frame.get_width()
        frame_height = color_frame.get_height()
        fps = 30.0  # Frame rate (you can adjust it if needed)

        # Initialize video writer
        self._video_writer = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))

        print("Starting Recording")   

    def _configure_camera(self, init_params=None):
        # Close Existing Camera
        self.disable_camera()

        # Initialize Camera using OpenCV
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        profile = self.pipeline.start(config)
        time.sleep(2)  # Allow the camera to stabilize

        # Check if the camera is opened (RealSense doesn't use .isOpened() like OpenCV)
        device = profile.get_device()
        if not device:
            raise RuntimeError("Camera Failed to Open")

        # Save latency (estimate based on RealSense FPS)
        # self.color_sensor = profile.get_device().first_color_sensor()
        # self.fps = self.color_sensor.get_option(rs.option.framerate)
        self.fps = 30
        self.latency = int(2.5 * (1e3 / self.fps))  # Estimate latency

        # Get and save intrinsics (width, height, fps)
        color_stream = profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        self._intrinsics = {
            "width": intrinsics.width,
            "height": intrinsics.height,
            "fps": int(self.fps)
        }

        print("Opening Camera: ", self.serial_number)