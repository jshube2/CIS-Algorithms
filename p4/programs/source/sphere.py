import numpy as np

class Sphere:
    """
    A triangle's bounding sphere, holding a center and radius.
    """
    def __init__(self, c, r):
        '''
        Create the bounding sphere and set the center and radius.
        Inputs:
        c -> the center as a numpy array
        r -> the radius as a numpy float
        '''
        self.center = c
        self.radius = r

def calc_cr(x, y, z):
    '''
    Calculates the center and radius of the bounding sphere with given triangle vertices.
    Inputs: v -> the triangle vertices as numpy arrays
    Outputs:
    center -> the center of the sphere as a numpy array
    radius -> the radius of the sphere as a numpy float
    '''
    # get middle of longest side
    side_mid = (x + y)/2
    u = x - side_mid
    v = z - side_mid
    d = np.cross(np.cross(u, v), u)
    l = max(0, ((v.dot(v) - u.dot(u))/(2*d).dot(v-u)))
    # calculate bounding sphere center
    center = side_mid + l*d
    # calculate bounding sphere radius
    radius = np.linalg.norm(center - x)
    return center, radius


def create_sphere(tri_coords, tri_ind):
    """
    Creates a list of bounding spheres for array of triangles defined with vertex coordinates.
    Inputs:
    tri_coords -> the triangle coordinates
    tri_ind -> the triangles indices
    Outputs:
    sphere -> the list of generated spheres
    """
    sphere = []
    # iterate through triangles to generate bounding spheres
    for i in range(np.shape(tri_ind)[1]):
        # get 3 corners of triangle
        a = tri_coords[:, int(tri_ind[0][i])]
        b = tri_coords[:, int(tri_ind[1][i])]
        c = tri_coords[:, int(tri_ind[2][i])]
        # calculate center and radius of sphere
        center, radius = calc_cr(a, b, c)
        sphere.append(Sphere(center, radius))
    return sphere
