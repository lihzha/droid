import numpy as np
from scipy.spatial.transform import Rotation as R


def random_transformation_matrix():
    # Generate a random rotation matrix
    random_rotation = R.random().as_matrix()

    # Generate a random translation vector (e.g., from a uniform distribution)
    random_translation = np.random.uniform(-1, 1, size=(3,))

    # Create a 4x4 transformation matrix
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = random_rotation  # Top-left 3x3 is rotation
    transformation_matrix[:3, 3] = random_translation  # Top-right 3x1 is translation

    return transformation_matrix


def vec_to_reorder_mat(vec):
    X = np.zeros((len(vec), len(vec)))
    for i in range(X.shape[0]):
        ind = int(abs(vec[i])) - 1
        X[i, ind] = np.sign(vec[i])
    return X


# Generate a random transformation matrix
rot_mat = random_transformation_matrix()
vr_to_global_mat = random_transformation_matrix()

rmat_reorder_rot: list = [-2, -1, -3, 4]
global_to_env_mat = vec_to_reorder_mat(rmat_reorder_rot)

print(global_to_env_mat @ vr_to_global_mat @ rot_mat)
print(vr_to_global_mat @ global_to_env_mat @ rot_mat)
