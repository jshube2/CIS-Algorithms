import numpy as np

def icp_get_d(num_frames, a_Frames, b_Frames, A_body, B_body, A_tip):
    '''
    Calculates the set of points d around the mesh to be used in ICP matching
    Each point d is the position of the pointer tip with respect to rigid body
    B for the current frame of data.
    Inputs:
    num_frames -> Number of data frames
    a_Frames -> set of data frames of optically tracked sensor data on the mobile tool
    b_frames -> set of data frames of optically tracked sensor data on the embedded tool
    A_body -> set of points describing the locations of the LEDs on tool A in the A body frame
    B_body -> set of points describing the locations of the LEDs on tool B in the B body frame
    A_tip -> location of the tip of tool A in the A body frame
    Outputs:
    d -> set of points in the B body frame around the edge of the mesh / bone
    that are used to perform ICP matching / registration. Each point d[k] corresponds
    to the place the tip of A was placed on the kth frame
    '''
    d = []
    for k in range(num_frames):
        # Iterate through the data frames
        # Calculate F_Ak and F_Bk for each frame
        F_Ak = get_transform(a_Frames[k], A_body)
        F_Bk = get_transform(b_Frames[k], B_body)

        # Calculate d using the frame transformations
        # e.g. Calculate A_tip in the B body frame for the kth data frame
        d.append(transform3D(get_inverse(F_Bk),transform3D(F_Ak, A_tip)))

    return d


def icp_match(M,d,method='brute'):
    '''
    Finds the matches between a set of points 'a' and a 3D structure defined by 'M'
    Inputs:
    M -> Mesh of triangles in list form. Each element is a set of triangle vertices
    d -> A set of points on a rigid body
    method -> string to determine the search method for matching the set 'd' to the mesh 'M'
    Outputs:
    c -> Set of points on mesh triangles that most closely corresponded to F_reg * d
    '''

    c = []
    # F_reg is Identity matrix for PA3
    F_reg = np.array(([1,0,0],
                      [0,1,0],
                      [0,0,1]))

    if method is 'brute':
        # Perform brute force search/matching
        for i in range(len(d)):
            # print "d[i]: ", d[i]
            c_0 = np.dot(F_reg,d[i])
            c.append(findClosestPoint_brute(c_0, M))
    # Later on in PA4 add additional 'if' statements for other methods of performing ICP matching
    # For now, only brute force search
    return c


def findClosestPoint_brute(s, mesh):
    '''
    Searches through the triangles composing a 3D mesh to find the closest position on the
    3D mesh to a given point s
    Inputs:
    s -> a single point near the 3D structure
    mesh -> Mesh of triangles in list form. Each element is a set of triangle vertices
    Outputs:
    c_min -> closest point to s on a mesh triangle
    '''
    x = np.zeros((2,3))
    c = []
    for triangle in mesh:
        # p, q, r are the three vertices that define the current triangle
        p = np.array(triangle[0])
        q = np.array(triangle[1])
        r = np.array(triangle[2])

        # q - p and r - p define the plane of the triangle
        x[0] = q - p
        x[1] = r - p
        # s - p is a vector that will be projected onto the triangle
        # in order to find the closest point on the triangle to s
        y = s - p

        z,g1,g2,g3 = np.linalg.lstsq(np.transpose(x),y)
        # g1, g2, g3 are garbage return variables from lstq. Not Used!
        lam = z[0]
        mu = z[1]

        lam = max(0,min(lam,1))
        # First 3 'if' conditions account for cases when current s is outside the boundaries of the triangle
        # The 'else' condition accounts for when s is inside the triangle / the 3-D prism defined by the triangle
        if lam < 0:
            c_i = r + lam*(p - r)
            c.append(c_i)
        elif mu < 0:
            c_i = p + lam * (q - p)
            c.append(c_i)
        elif lam + mu > 1:
            c_i = q + lam * (r - q)
            c.append(c_i)
        else:
            c_i = p + lam * (q - p) + mu * (r - p)
            c.append(c_i)

    dist = []
    for ci in c:
        # For each c (closest point to s for a given triangle) compute the distance between c and s
        dist.append(np.linalg.norm(s - ci))

    # Find the c that corresponds to the shortest distance between ci and s
    c_min = c[np.argmin(np.array(dist))]

    return c_min


def transform3D(F, a):
    '''
    Simple 3D transformation
    R*a + p
    '''
    return np.dot(F[0],a) - F[1]



def get_transform(a, b):
    '''
    Determines the rotation and translation components of the transformation
    between the two provided point cloud frames a and b.
    Input: Two point cloud frames a and b, each a numpy array of 3D points.
    Output: R, the rotation matrix, and t, the translation matrix, such that
    R*a + t = b.
    '''

    # Given two 3x3 matrices containing the x,y,z coordinates for two sets of 3 points
    # Compute the R and p using Arun's method that define the transform between the point sets

    N = len(a)

    # find the mean [x,y,z] values for a and b
    a_centroid = np.mean(a, axis=0)
    b_centroid = np.mean(b, axis=0)

    # subtract centroid values to localize points a and b
    a_local = a - np.tile(a_centroid, (N,1))
    b_local = b - np.tile(b_centroid, (N,1))

    # get dot product of localized a and b
    H = np.dot(np.transpose(a_local), b_local)

    # Use Single Value Decomposition to determine the rotation matrix
    U, S, V = np.linalg.svd(H)
    R = np.dot(V.T, U.T)

    if np.linalg.det(R) < 0:
        # Special case of Arun's Method for reflection matrices
        V[2,:] *= -1
        R = np.dot(V.T, U.T)

    # determine the translation matrix
    t = np.add(np.dot(-R, a_centroid.T), b_centroid.T)

    # round each component to 3 decimal points
    R = R.round(3)
    t = t.round(3)

    # return the rotation and translation matrices
    return (R, t)


def get_inverse(F):
    '''
    Returns the inverse of a frame transformation.
    Input: F -> a frame, with rotation and translation components
    Outputs:
    R_inv -> the inverted rotation matrix
    p_inv -> the inverted translation matrix
    '''
    R = F[0]
    p = F[1]

    # F_inv = [R_inv, p_inv]
    R_inv = np.linalg.inv(R)
    p_inv = -1*np.dot(R_inv, p)

    return R_inv, p_inv


def get_distance(d, c):
    '''
    Returns the euclidian distance between two 3D points.
    '''
    distance = np.linalg.norm(d - c)
    return distance
