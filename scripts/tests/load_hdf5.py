import h5py

def load_hdf5_to_dict(hdf5_file_path, keys_to_load = {
        'action/cartesian_position',
        'action/gripper_position',
        'observation/robot_state/joint_positions',
        'observation/robot_state/joint_velocities',
        'observation/image'
    }):
    """
    Load an HDF5 file into a nested dictionary, including only specified keys.

    Parameters:
    hdf5_file_path (str): Path to the HDF5 file.

    Returns:
    dict: A dictionary representation of the HDF5 file.
    """
    # Define keys to load in hierarchical form
    
    def should_load(path):
        """Check if the current path should be loaded based on the specified keys."""
        return any(path == key or path.startswith(f"{key}/") for key in keys_to_load)

    def recursive_load(group, path=''):
        """Recursively load a group into a dictionary, including only specified keys."""
        result = {}
        for key, item in group.items():
            current_path = f"{path}/{key}".strip('/')
            if isinstance(item, h5py.Group):
                # If the item is a group, recurse into it
                nested_result = recursive_load(item, current_path)
                if nested_result:  # Only add if there's valid data in the nested result
                    result[key] = nested_result
            else:
                # If the item is a dataset, store it as a numpy array if it's a key to load
                if should_load(current_path):
                    result[key] = item[()]
        return result

    with h5py.File(hdf5_file_path, 'r') as file:
        return recursive_load(file)
    
if __name__ == '__main__':
    f = load_hdf5_to_dict('/home/lab/droid/traj_data/41.h5')
    breakpoint()