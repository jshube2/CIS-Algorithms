import numpy as np
import data_methods as dm
import scipy.linalg

class cov_tree_node:
    """
    Covariance tree node class.
    """
    def __init__(self, triangles):
        '''
        Creates a new covariance tree node.
        Inputs:
        triangles -> the list of triangles within the bounds of this node.
        Outputs: A new covariance tree node
        '''
        self.triangles = triangles
        self.num_triangles = len(triangles)
        self.bounds = None
        self.frame = self.create_cov_frame(self.num_triangles)
        self.has_subtrees = False
        self.subtrees = [None, None]
        self.build_subtrees()
        self.find_bounding_box(self.num_triangles)


    def update_closest(self, triangle, point, bound, closest):
        '''
        Determines if the given point is in the bounding sphere, and if so
        finds the closest point on the triangle to the given point.
        Inputs:
        triangle -> the bounded triangle
        point -> the s point that we wish the find the closest triangle point to
        bound -> the current bound to check if s can be found
        closest -> the previous closest point found for s
        '''
        #print "Running update closest!"
        if np.linalg.norm(point - triangle.sphere.center) - triangle.sphere.radius > bound[0]:
            return bound, closest
        closest_point = triangle.get_closest_point(point)
        dist = np.linalg.norm(closest_point - point)
        if dist < bound[0]:
            bound[0] = dist
            closest[0] = closest_point


    def find_closest_point_tree(self, point, bound, closest):
        '''
        Finds the closest point in the covariance tree to the given point,
        recursively calling on its subtrees.
        Inputs:
        point -> the s point that we wish the find the closest triangle point to
        bound -> the current bound to check if s can be found
        closest -> the previous closest point found for s
        '''
        b = (dm.inv(self.frame)[0].dot(point.reshape((3, 1))) + dm.inv(self.frame)[1]).flatten()
        if np.any(b.reshape((3, 1)) < (self.bounds[0] - bound[0])) or (np.any(b.reshape((3, 1)) > (self.bounds[1] + bound[0]))):
            return

        # if subtrees, recursively run until reaching a leaf node
        if self.has_subtrees:
            self.subtrees[0].find_closest_point_tree(point, bound, closest)
            self.subtrees[1].find_closest_point_tree(point, bound, closest)

        # if a leaf node is reached, call update_closest
        else:
            for i in range(self.num_triangles):
                self.update_closest(self.triangles[i], point, bound, closest)


    def create_cov_frame(self, num_tri):
        '''
        Generates the initial frame of the covariance tree node.
        Inputs:
        num_tri -> the number of triangles
        '''
        points = None
        num_triangles = num_tri
        # first, construct a list of triangles
        for i in range(num_triangles):
            if i == 0:
                points = self.triangles[i].points.tolist()
            else:
                for p in self.triangles[i].points.tolist():
                    points.append(p)

        # use this triangle list to find the covariance frame
        points = np.array(points).squeeze().T
        return self.find_cov_frame(num_triangles * 3, points)


    def find_cov_frame(self, num_tri, tri_list):
        '''
        Finds the frame of the covariance tree node.
        Inputs:
        num_tri: the number of triangles
        tri_list: the list of triangles in the covariance tree node
        Outputs:
        [rot, trans] -> the rotation and translation components of the frame
        '''
        points = tri_list[:, 0:num_tri]
        # determine the translation matrix
        trans = np.mean(points, axis=1, keepdims=True)
        u = points - trans
        # form outer product matrix
        A = u.dot(u.T)
        # determine eigenvalues and vectors of A
        eig_vals, eig_vecs = np.linalg.eig(A)
        eig_inds = eig_vals.argsort()[::-1]
        # store the maximum eigenvector
        max_eig_vec = eig_vecs[eig_inds[0]]

        # Use Single Value Decomposition to determine the rotation matrix
        coeffs = np.zeros((3, 9))
        coeffs[(0, 1, 2), (0, 3, 6)] = 1.0
        H = np.linalg.lstsq(coeffs, max_eig_vec)[0].reshape((3, 3))
        u, s, v = np.linalg.svd(H)
        U = u.T
        V = v.T

        rot = np.dot(V, U)
        # Special case of Arun's Method for reflection matrices
        if scipy.linalg.det(rot) < 0:
            V[2,:] *= -1
            rot = np.dot(V, U)

        return [rot, trans]


    def find_bounding_box(self, num_triangles):
        '''
        Finds the bounds of the bounding box around the given covariance tree node.
        Inputs:
        num_triangles -> the number of triangles in the covariance tree node
        Outputs:
        bounds -> the lower and upper bounds of the tree node
        '''
        # find the lower bound of bounding box
        LB = dm.transform3D(self.triangles[0].sort_point(), dm.inv(self.frame))
        bounds = [LB, LB]
        # enlarge bounds and return
        for i in range(num_triangles):
            bounds = self.triangles[i].enlarge_bounds(self.frame, bounds)
        self.bounds = bounds
        return bounds


    def build_subtrees(self):
        '''
        Constructs subtrees based on the list of triangles in this covariance tree node.
        Determines if there is a possible split.
        '''
        possible_split = None

        tri_points = []
        # split and sort triangles based on their x coordinate
        for i in range(self.num_triangles):
            tri_points.append(dm.transform3D(self.triangles[i].sort_point(), dm.inv(self.frame)).tolist())
        tri_points = np.array(tri_points).squeeze().T

        indexes = np.argsort(tri_points[0, :])
        #indexes = np.argsort(tri_points[0])

        tri_points = tri_points[:,indexes]
        #tri_points = tri_points[indexes]
        self.triangles = self.triangles[indexes]

        # signbit returns true if any element is negative
        split_check = np.any(np.diff(np.signbit(tri_points[0, :])))
        if split_check:
            # find instances where signbit returns true
            possible_split = np.where(np.diff(np.signbit(tri_points[0, :])))[0][0]

        if possible_split is not None:
            if possible_split == 0 or possible_split == self.num_triangles:
                return
            else:
                self.has_subtrees = True
                # generate subtrees for cov tree node
                self.subtrees[0] = cov_tree_node(self.triangles[0:possible_split])
                self.subtrees[1] = cov_tree_node(self.triangles[possible_split:self.num_triangles])
