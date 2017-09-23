import numpy as np
import scipy.linalg

def inv(frame):
    '''
    Inverts a given frame array.
    Input: the frame array
    Output: the inverted frame array
    '''
    r_inv = scipy.linalg.inv(frame[0])
    return [r_inv, -r_inv.dot(frame[1])]


def compose(a, b):
    '''
    Creates a composition of two input frames.
    Inputs: two frames a and b
    Outputs: A transformed frame representing the composition of a and b.
    '''
    return [a[0].dot(b[0]), a[0].dot(b[1]) + a[1]]


def transform3D(a, f):
    """
    Runs a frame transformation on given point cloud a.
    Input: point cloud a and frame f
    Output: the resulting transformed point cloud
    """
    return f[0].dot(a) + f[1]


def get_transform(a, b):
    '''
    Determines the rotation and translation components of the transformation
    between the two provided point cloud frames a and b.
    Input: Two point cloud frames a and b, each a numpy array of 3D points.
    Output: R, the rotation matrix, and t, the translation matrix, such that
    R*a + t = b.
    '''
    a_centroid = np.mean(a, axis=1, keepdims=True)
    b_centroid = np.mean(b, axis=1, keepdims=True)

    a_local = a - a_centroid
    b_local = b - b_centroid

    # get dot product of localized a and b
    H = a_local.dot(b_local.T)

    # Use Single Value Decomposition to determine the rotation matrix
    U, S, V = np.linalg.svd(H)
    u = U.T
    v_t = V.T

    rot = np.dot(v_t, u)

    # Special case of Arun's Method for reflection matrices
    if scipy.linalg.det(rot) < 0:
        v_t[2,:] *= -1
        rot = np.dot(v_t, u)

    # determine the translation matrix
    trans = b_centroid - rot.dot(a_centroid)

    return [rot, trans]
