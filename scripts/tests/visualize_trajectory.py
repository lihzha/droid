import h5py
import cv2
import numpy as np

import h5py
import cv2
import numpy as np

def save_hdf5_images_as_videos(hdf5_path, output_path, fps=30):
    """
    Loads images from three camera views in an HDF5 file and saves them as one MP4 video
    in a triangular layout: two views on top, one view centered below.

    Parameters:
        hdf5_path (str): Path to the HDF5 file.
        output_path (str): Path to save the output MP4 video.
        fps (int): Frames per second for the output video.
    """
    # Open the HDF5 file
    with h5py.File(hdf5_path, 'r') as f:
        # Access the 'observation' and 'images' group
        images_dict = f['observation']['image']
        camera_views = list(images_dict.keys())
        
        # Assume the first three views correspond to the three cameras
        view1 = np.array(images_dict[camera_views[0]])  # (traj_len, H, W, C)
        view2 = np.array(images_dict[camera_views[1]])  # (traj_len, H, W, C)
        view3 = np.array(images_dict[camera_views[2]])  # (traj_len, H, W, C)

        # Verify that all views have the same traj_len, height, width, and channels
        assert view1.shape == view2.shape == view3.shape, "All views must have the same shape."

        traj_len, H, W, C = view1.shape

        # Create video writer for the combined video
        # The width of the combined video will be max(W, 2*W) (two views side by side or one view wide)
        # The height will be 2*H (two rows: one for the first two views, one for the third view)
        combined_width = 2 * W
        combined_height = 2 * H
        
        video_writer = cv2.VideoWriter(
            output_path, 
            cv2.VideoWriter_fourcc(*'mp4v'),  # Codec for MP4
            fps,  # Frames per second
            (combined_width, combined_height)  # Frame size (width, height)
        )

        # Write each combined frame to the video
        for t in range(traj_len):
            # Get frames from each view at time step t
            frame1 = view1[t]
            frame2 = view2[t]
            frame3 = view3[t]

            # Concatenate the first two views horizontally (side by side)
            top_row = np.hstack((frame1, frame2))

            # Create a blank frame to fill the sides of the bottom row
            blank_frame = np.ones_like(frame3) * 255

            # Center the third view in the bottom row
            bottom_row = np.hstack((blank_frame, frame3, blank_frame))[:, W//2:W//2 + 2*W]

            # Stack the two rows vertically
            combined_frame = np.vstack((top_row, bottom_row))

            # Write the combined frame to the video
            video_writer.write(combined_frame)
        
        # Release the video writer
        video_writer.release()
        print(f"Saved combined video to {output_path}")

for i in range(1, 5):
    trajectory_folderpath = "/home/lab/droid/traj_data"
    h5_filepath = trajectory_folderpath + f"/{i}.h5"

    output_path = f'{i}.mp4'
    save_hdf5_images_as_videos(h5_filepath, output_path)
