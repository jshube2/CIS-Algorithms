from numpy import *

'''
Determines the rotation and translation components of the transformation
between the two provided point cloud frames a and b.
Input: Two point cloud frames a and b, each a numpy array of 3D points.
Output: R, the rotation matrix, and t, the translation matrix, such that
R*a + t = b.
'''
def get_transform(a, b):
    # Given two 3x3 matrices containing the x,y,z coordinates for two sets of 3 points
    # Compute the R and p using Arun's method that define the transform between the point sets

    N = len(a)

    # find the mean [x,y,z] values for a and b
    a_centroid = mean(a, axis=0)
    b_centroid = mean(b, axis=0)

    # subtract centroid values to localize points a and b
    a_local = a - tile(a_centroid, (N,1))
    b_local = b - tile(b_centroid, (N,1))

    # get dot product of localized a and b
    H = dot(transpose(a_local), b_local)

    # Use Single Value Decomposition to determine the rotation matrix
    U, S, V = linalg.svd(H)
    R = dot(V.T, U.T)

    if linalg.det(R) < 0:
        # Special case of Arun's Method for reflection matrices
        V[2,:] *= -1
        R = dot(V.T, U.T)

    # determine the translation matrix
    t = add(dot(-R, a_centroid.T), b_centroid.T)

    # round each component to 3 decimal points
    R = R.round(3)
    t = t.round(3)

    # return the rotation and translation matrices
    return (R, t)

