import numpy as np
from transformation import get_transform


def opt2em_calibrate_pivot(H, D, d, num_frames):
    # Inputs:
    # H -> Set of multiple frames of data where the tool is in contact with a fiducial pin in the Optical Tracker Frame
    # D -> Set of multiple frames of data of the distance between the Optical and EM tracker
    # d -> Set of optical trackers on the EM frame
    # num_frames -> The number of data frames in H
    # Outputs:
    # p_tip -> The vector from the tool frame to the end of the tool in contact with the post
    # p_post -> The vector from the EM base frame to the top of the fiducial post

    # Calculate a transform between the EM and the Optical tracker
    # for each frame of data
    H = np.array(H)
    D = np.array(D)
    d = np.array(d)
    F_d = []
    n_H = len(H)/num_frames
    n_D = len(D)/num_frames

    for i in range(0, num_frames):
        F_d.append(get_transform(d, D[i*(n_D):(i+1)*(n_D), :]))

    # Transform each H into the Em tracker frame using these F_d
    # for each frame of data
    H2 = []
    for i in range(0, num_frames):
        for j in range(0, n_H):
            H2.append(transform_3D(F_d[i], H[i*n_H + j]))
    H2 = np.array(H2)

    # Return the coordinates of the post the optically tracked
    # tool is being calibrated on

    return calibrate_pivot(H2, num_frames)


def transform_3D(F, a):
    # Inputs:
    # F -> frame transformation
    # a -> vector to be transformed
    # Outputs:
    # b -> 'a' transformed by F
    b = np.dot(F[0],a) - F[1]
    return b


def calibrate_pivot(G, num_frames):
    # Inputs:
    # G -> Set of multiple frames of data where the tool is in contact with a fiducial pin
    # num_frames -> The number of data frames in G
    # Outputs:
    # p_tip -> The vector from the tool frame to the end of the tool in contact with the post
    # p_post -> The vector from the EM base frame to the top of the fiducial post

    # Given a set of EM tracker readings
    # Produce a transform between F_G to F_g on the probe
    # Use this frame transformation to produce a set of points g
    # Which are G localized to the probe

    G = np.array(G)
    F_Gg = []
    n = len(G)/num_frames
    G_0 = compute_midpoint(G[0:n, :])

    g_k = localize(G_0, G[0:n, :])
    for i in range(0, num_frames):

        F_Gg.append(get_transform(g_k, G[i*(n):(i+1)*(n), :]))

    # Then solve for b_post and b_tip?
    p_post, p_tip = pivot_calibration(F_Gg, num_frames)

    return p_post, p_tip



def localize(A_0, A):
    # Inputs:
    # A_0 -> median of A
    # A -> Set of points
    # Outputs:
    # a -> set of points localized to A_0.
    # e.g. a_i = A_i - A_0
    a = []
    for i in range(0,len(A)):
        ax = A[i][0] - A_0[0]
        ay = A[i][1] - A_0[1]
        az = A[i][2] - A_0[2]
        a.append((ax,ay,az))
    a = np.array(a)
    return a


def compute_midpoint(A):
    # Inputs:
    # A -> set of points
    # Outputs:
    # A_0 -> median point of A

    A_0 = []
    for j in range(0,len(A[0])):
        s = 0
        for i in range(0, len(A)):
            s += A[i][j]
        s /= len(A)
        A_0.append(s)
    A_0 = np.array(A_0)
    return A_0


def pivot_calibration(F, n):
    # Inputs:
    # F -> A list of frame transformations between the same two frames collected with different data
    # n -> The number of frames of data collected for the pivot calibration
    # Outputs:
    # p_tip -> The vector from the tool frame to the end of the tool in contact with the post
    # p_post -> The vector from the EM base frame to the top of the fiducial post

    # I is the identity matrix
    I = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    # R and p will contain all the Rk and pk according to slide 25 of
    # Frames.pptx

    R = F[0][0]
    p = -1*F[0][1]

    # Each row of the the R matrix will be R_k - I
    # All the R_k - I will be concatenated vertically
    # The p vector will be all p_k concatenated vertically
    R = np.concatenate((R, -1*I), axis=1)

    for i in range(1, n):
        R_k = F[i][0]
        p_k = -1*F[i][1]
        R_k = np.concatenate((R_k, -1*I), axis=1)
        R = np.concatenate((R, R_k), axis=0)
        p = np.concatenate((p, p_k), axis=0)
    # Solve Rb =p
    # First 3 rows in b will be the vector from F_G to the dimpled post
    # The last 3 rows in b will be vector from F_g to the end of the probe
    b, residuals, rank, s = np.linalg.lstsq(R,p)
    #print b
    p_tip = b[0:3]
    p_post = b[3:]
    return p_post, p_tip
