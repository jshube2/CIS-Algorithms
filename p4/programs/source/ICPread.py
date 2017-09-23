import numpy as np

def read_rigid_body(filename):
    '''
    Reads in the rigid body file, and saves the number of led markers,
    their locations, and the tip locations.
    Inputs:
    filename -> the rigid body file
    Outputs:
    num_markers -> the number of markers on the rigid body
    led_markers -> the led markers as a list of x,y,z
    tip -> the tip x,y,z coordinates
    '''

    rigid_data = open(filename)
    # set number of markers to iterate over
    first_line = rigid_data.readline().split()
    num_markers = int(first_line[0])

    led_markers = np.zeros([3, num_markers])
    # iterate thourgh markers, one per line
    for i in range(num_markers):
        # set led_markers from file input
        current_marker = rigid_data.readline().split()
        for j in range(0, 3):
            led_markers[j][i] = np.float64(current_marker[j])
    # set tip from file input
    tip = np.zeros([3, 1])
    tip_line = rigid_data.readline().split()
    for j in range(0, 3):
        tip[j] = np.float64(tip_line[j])

    return num_markers, led_markers, tip



def read_sample(filename, num_A, num_B):
    '''
    Parses through the sample readings for each frame.
    Inputs:
    filename -> the sample readings filename
    num_A -> the number of led markers on A
    num_B -> the number of led markers on B
    Outputs:
    a_Frames -> the x,y,z A coordinates for each frame
    b_Frames -> the x,y,z B coordinates for each frame
    '''
    sample_readings = open(filename)

    first_line = sample_readings.readline().split(",")
    num_markers = int(first_line[0].strip())
    num_Samples = int(first_line[1].strip())
    num_d = num_markers - num_A - num_B

    a_Frames = []
    b_Frames = []
    # iterate over samples, setting frame markers
    for i in range(num_Samples):
        a_current_markers = np.zeros([3, num_A])
        b_current_markers = np.zeros([3, num_B])
        # set a_markers from input file
        for j in range(num_A):
            point = sample_readings.readline().split()
            for k in range(0, 3):
                a_current_markers[k][j] = point[k].strip(',')
        # set b_markers from input file
        for j in range(num_B):
            point = sample_readings.readline().split()
            for k in range(0, 3):
                b_current_markers[k][j] = point[k].strip(',')

        # disregard these lines
        for j in range(num_d):
            sample_readings.readline()
        # append markers to a_Frames and b_Frames
        a_Frames.append(a_current_markers)
        b_Frames.append(b_current_markers)

    return a_Frames, b_Frames



def read_mesh(filename):
    '''
    Reads in the mesh file and saves the triangle vertices as a list.
    Inputs:
    filename -> the mesh file
    Outputs:
    tri_coords -> An array that holds the coordinates of each vertex
    tri_ind -> An array that holds the indices of the coordinates for each triangle
    '''

    mesh_data = open(filename)
    # get file information from first line
    num_Vertices = int(mesh_data.readline().strip())
    tri_coords = np.zeros([3, num_Vertices])
    # fill triangle coordinates array
    for i in range(num_Vertices):
        vertex = mesh_data.readline().split()
        for j in range(3):
            tri_coords[j][i] = vertex[j]

    num_Triangles = int(mesh_data.readline().strip())
    tri_ind = np.zeros([3, num_Triangles], dtype=int)

    # fill triangle indices array
    for i in range(num_Triangles):
        triangle = mesh_data.readline().split()
        for j in range(3):
            tri_ind[j][i] = int(triangle[j])
    return tri_coords, tri_ind
