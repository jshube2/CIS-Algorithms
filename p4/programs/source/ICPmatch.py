import numpy as np
import math
import ICPread as icpf
import triangle as tr
import cov_tree_node as ctn
import data_methods as dm
import sphere as bs
import sys


def icp_get_d(a_Frames, b_Frames, led_A, tip_A, led_B):
    '''
    Calculates the set of points d around the triangle mesh to be used in ICP matching
    Each point d is the position of the pointer tip with respect to rigid body
    Inputs:
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
    d = np.zeros([3, len(a_Frames)])
    # Iterate through the data frames
    for i in range(len(a_Frames)):
        # Calculate F_Ak and F_Bk for each frame
        F_Ak = dm.get_transform(led_A, a_Frames[i])
        F_Bk = dm.get_transform(led_B, b_Frames[i])
        F = dm.compose(dm.inv(F_Bk), F_Ak)
        # Calculate d using the frame transformations
        d_k = F[0].dot(tip_A) + F[1]
        for j in range(0, 3):
            d[j][i] = d_k[j]

    return d


def icp_match_iterator(tri_coords, tri_indices, d, icp_type):
    '''
    Iterative version of iterative closest point matching, generates triangle
    mesh and determines points on this mesh closest to points d. Runs for
    30 iterations or until F_reg change is below threshold.
    Inputs:
    tri_coords -> triangle coordinates for mesh generation
    tri_indices -> triangle indices for mesh generation
    d -> A set of points on a rigid body
    icp_type -> either linear, sphere, or tree, designating what method of
    ICP to run for the program
    Outputs:
    c_matching -> Set of points on mesh triangles that most closely corresponded
    to F_reg * d
    F_reg -> the final F_reg value after n iterations
    '''
    F_reg = [np.identity(3), np.zeros([3, 1])]
    num_iters = 0
    triangles = []
    # generate triangles from coords and inds
    for i in range(tri_indices.shape[1]):
        t = tr.Triangle(tri_coords[:, tri_indices[:, i]])
        triangles.append(t)
    triangles = np.array(triangles)


    # build the covariance tree
    if icp_type == "tree":
        print "Building The Covariance Tree"
        tree = ctn.cov_tree_node(triangles)

    # generate the bounding spheres
    elif icp_type == "sphere":
        spheres = bs.create_sphere(tri_coords, tri_indices)

    # initialize previous error for error difference calculations
    error_prev = 0
    # if error is below tolerance, exit the iteration loop
    tolerance = 0.000001

    print('Running ICP:')
    while num_iters < 30:

        s = dm.transform3D(d, F_reg)

        if num_iters == 0:
            c_previous = s + 1000

        c_matching = np.zeros([3, np.shape(s)[1]])
        old = None
        closest_pts = None
        if icp_type == "tree":
            old = c_previous
            closest_pts = c_previous

        for i in range(np.shape(s)[1]):

            # if linear, run closest_point_linear and return the resulting c
            if icp_type == "linear":
                c = closest_point_linear(s[:, i], tri_coords, tri_indices)
                c_matching[:, i] = c[:]

            # if sphere, run closest_point_sphere and return the resulting c
            elif icp_type == "sphere":
                c = closest_point_sphere(s[:, i], tri_coords, tri_indices, spheres)
                c_matching[:, i] = c[:]

            # if tree, run find_closest_point_tree and return the resulting c
            elif icp_type == "tree":
                c_prev = closest_pts[:, i]
                c_difference = np.linalg.norm(c_prev - s[:, i])
                closest = [c_prev]
                bound = [c_difference]
                tree.find_closest_point_tree(s[:, i], bound, closest)
                c_matching[:, i] = closest[0][:]

            else:
                print "Please enter a valid ICP procedure: either 'linear', 'sphere', or 'tree'"
                sys.exit(0)

        # get the F delta between calculated c and s
        F_change = dm.get_transform(s, c_matching)
        # compose with current F_reg to get F_new
        F_new = dm.compose(F_change, F_reg)

        # if error below threshold, exit iterations and return c and F_new
        error = get_error(F_reg, F_new, error_prev)
        if error < tolerance or abs(error - error_prev) < tolerance:
            return c_matching, F_new


        print "Iteration: " + str(num_iters)
        print "Error: " + str(error)
        # reset variables for next iteration
        F_reg = F_new
        error_prev = error
        c_previous = c_matching

        num_iters += 1

    return c_matching, F_reg


def get_error(F_reg, F_regNew, prev_error):
    '''
    Summates the squared difference in each element of the previous and current F_reg.
    Inputs:
    F_reg -> the previous frame transform
    F_new -> the new frame transform
    error_prev -> the error from the last iteration
    Output: error -> the sum of squared difference between F_reg and F_new
    '''
    err = 0
    for i in range(0, 3):
        err += math.pow(F_reg[1][i] - F_regNew[1][i], 2)
        for j in range(0, 3):
            err += math.pow(F_reg[0][i][j] - F_regNew[0][i][j], 2)

    return err


def closest_point_linear(s, tri_coords, tri_ind):
    '''
    Searches through the triangles composing a 3D mesh to find the closest position on the
    3D mesh to a given point s, using the linear method.
    Inputs:
    s -> a single point near the 3D structure
    mesh -> Mesh of triangles in list form. Each element is a set of triangle vertices
    Outputs:
    closest_point -> closest point to s on a mesh triangle
    '''
    c = np.zeros([3, np.shape(tri_ind)[1]])

    for i in range(np.shape(tri_ind)[1]):
        # set corners of triangle
        p = tri_coords[:, int(tri_ind[0][i])]
        q = tri_coords[:, int(tri_ind[1][i])]
        r = tri_coords[:, int(tri_ind[2][i])]

        # find closest point on triangle to s
        c_min = closest_point_triangle(s, p, q, r)
        c[:, i] = c_min[:]

    # calculate norm distance between s and c
    min_dist = np.linalg.norm(s - c[:, 0])
    closest_point = c[:, 0]

    for i in range(np.shape(c)[1]):
        d = np.linalg.norm(s - c[:, i])
        # if new smallest distance, reset min distance and closest point
        if (d < min_dist):
            min_dist = d
            closest_point = c[:, i]

    return closest_point


def closest_point_sphere(s, tri_coords, tri_ind, spheres):
    '''
    Searches through the triangles composing a 3D mesh to find the closest position on the
    3D mesh to a given point s, using the bounding sphere method.
    Inputs:
    s -> a single point near the 3D structure
    mesh -> Mesh of triangles in list form. Each element is a set of triangle vertices
    Outputs:
    closest_point -> closest point to s on any of the triangles
    '''
    # set corners for all triangles
    p = tri_coords[:, int(tri_ind[0][0])]
    q = tri_coords[:, int(tri_ind[1][0])]
    r = tri_coords[:, int(tri_ind[2][0])]

    closest_point = closest_point_triangle(s, p, q, r)
    min_dist = np.linalg.norm(s - closest_point)

    for i in range(1, np.shape(tri_ind)[1]):
        # only look at those within bounding sphere radius
        if np.linalg.norm(s - spheres[i].center) - spheres[i].radius < np.linalg.norm(s - closest_point):
            # set corners of triangle
            p = tri_coords[:, int(tri_ind[0][i])]
            q = tri_coords[:, int(tri_ind[1][i])]
            r = tri_coords[:, int(tri_ind[2][i])]
            # find closest point on triangle to p
            c_min = closest_point_triangle(s, p, q, r)
            # calculate norm distance between s and c
            d = np.linalg.norm(s - c_min)
            # if new smallest distance, reset distance and closest point
            if d < min_dist:
                min_dist = d
                closest_point = c_min

    return closest_point


def closest_point_triangle(s, p, q, r):
    '''
    Finds the point on the triangle with corners p,q,r closest to the
    point s.
    Inputs:
    s -> the point to match with the minimum triangle point
    (p, q, r) -> the corners of the triangle
    Output:
    c_min -> the closest point on the triangle face
    '''
    x = np.zeros([3, 2])

    for j in range(0, 3):
        # define the plane of the triangle
        x[j][0] = q[j] - p[j]
        x[j][1] = r[j] - p[j]

    z = np.linalg.lstsq(x, s - p)
    # lam and mu from least squares of s-p
    lam = z[0][0]
    mu = z[0][1]

    c = p + lam * (q - p) + mu * (r - p)
    c_min = np.zeros(3)

    # check case where current s inside boundaries of triangle
    if lam >= 0 and mu >= 0 and lam + mu <= 1:
        c_min = c
    # check cases where current s outside boundaries of triangle
    elif lam < 0:
        # set value of c min
        l = np.float64(((c-r).dot(p-r))) / np.float64(((p-r).dot(p-r)))
        lambda_min = max(0, min(l, 1))
        c_min = p + lambda_min*(p-r)
    elif mu < 0:
        # set value of c_min
        l = np.float64(((c-p).dot(q-p))) / np.float64(((q-p).dot(q-p)))
        lambda_min = max(0, min(l, 1))
        c_min = p + lambda_min*(q-p)
    elif lam + mu > 1:
        l = np.float64(((c-q).dot(r-q))) / np.float64(((r-q).dot(r-q)))
        lambda_min = max(0, min(l, 1))
        c_min = p + lambda_min*(r-q)
    return c_min


def get_distance(c, d):
    '''
    Returns the norm distance between generated d and closest point c
    Inputs:
    c -> the point generated by ICP
    d -> the complimentary true point
    Outputs: the norm distance between the two
    '''
    dist = np.zeros(np.shape(c)[1])
    for i in range(np.shape(c)[1]):
        dist[i] = np.linalg.norm(d[:, i] - c[:, i])

    return dist
