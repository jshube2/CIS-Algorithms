import numpy as np
import sphere as sphere
import ICPmatch as icpm
import data_methods as dm

class Triangle:
    """
    A triangle with a bounded sphere.
    """
    def __init__(self, points):
        '''
        Initializes the triangle by setting its 3 corner points.
        Inputs: points -> the three coordinate points of the triangle
        '''
        self.points = points
        p, q, r = self.points[:, 0], self.points[:, 1], self.points[:, 2]
        # calculate the triangle sphere's center and radius
        center, radius = sphere.calc_cr(p, q, r)
        # the bounding sphere of the triangle
        self.sphere = sphere.Sphere(center, radius)


    def sort_point(self):
        '''
        Calculates the mean of the triangle coordinates.
        Outputs: the mean point as a numpy array
        '''
        return np.mean(self.points, axis=1, keepdims=True)


    def get_closest_point(self, vector):
        '''
        Determines the closest point to a given vector.
        Inputs: vector -> a numpy array vector
        Outputs: the closest point on the triangle as a numpy array
        '''
        p, q, r = self.points[:, 0], self.points[:, 1], self.points[:, 2]
        return icpm.closest_point_triangle(vector, p, q, r)


    def enlarge_bounds(self, frame, bounds):
        '''
        Enlarges the triangle's bounding box.
        Inputs:
        frame -> the frame of the bounding box as a numpy array
        bounds -> the expansion boundaries as a numpy array
        Output: bounds -> the expanded bounds
        '''
        # calculate f as frame transformation onl triangle points
        f = dm.transform3D(self.points, dm.inv(frame))
        for i in range(3):
            # reset the triangle's bounding box
            bounds[0] = np.amin(np.hstack((bounds[0], f[:, i].reshape((3, 1)))), axis=1, keepdims=True)
            bounds[1] = np.amax(np.hstack((bounds[1], f[:, i].reshape((3, 1)))), axis=1, keepdims=True)
        return bounds
