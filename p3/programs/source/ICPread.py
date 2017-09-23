import numpy as np
import sys

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
    # get file information from first line
    first_line = rigid_data.readline().split()
    num_markers = int(first_line[0].strip())
    led_markers = []
    # iterate through the markers, one per line
    for i in range(num_markers):
        current_marker = rigid_data.readline().split()
        x = float(current_marker[0].strip())
        y = float(current_marker[1].strip())
        z = float(current_marker[2].strip())
        # save coordinates into a list
        led_markers.append([x, y, z])

    # save tip coordinates to a list
    tip_line = rigid_data.readline().split()
    tip_x = float(tip_line[0].strip())
    tip_y = float(tip_line[1].strip())
    tip_z = float(tip_line[2].strip())
    tip = [tip_x, tip_y, tip_z]
    return [num_markers, led_markers, tip]


def read_sample_readings(filename, a_markers, b_markers):
    '''
    Parses through the sample readings for each frame.
    Inputs:
    filename -> the sample readings filename
    a_markers -> the number of led markers on A
    b_markers -> the number of led markers on B
    Outputs:
    num_samples -> the number of sample Frames
    a_Frames -> the x,y,z A coordinates for each frame
    b_Frames -> the x,y,z B coordinates for each frame
    '''

    sample_readings = open(filename)
    # get file information from first line
    first_line = sample_readings.readline().split(",")
    num_s = int(first_line[0].strip())
    num_a = a_markers
    num_b = b_markers
    num_d = num_s - num_a - num_b
    num_samples = int(first_line[1].strip())
    a_Frames = []
    b_Frames = []
    d = []
    # iterate through each sample frame
    for i in range(num_samples):
        a_current_markers = []
        b_current_markers = []
        # get A marker coordinates for each frame
        for j in range(num_a):
            current_marker = sample_readings.readline().split(",")
            x = float(current_marker[0].strip())
            y = float(current_marker[1].strip())
            z = float(current_marker[2].strip())
            a_current_markers.append([x, y, z])

        # get B marker coordinates for each frame
        for j in range(num_b):
            current_marker = sample_readings.readline().split(",")
            x = float(current_marker[0].strip())
            y = float(current_marker[1].strip())
            z = float(current_marker[2].strip())
            b_current_markers.append([x, y, z])

        # disregard these lines
        for j in range(num_d):
            sample_readings.readline()

        a_Frames.append(a_current_markers)
        b_Frames.append(b_current_markers)

    return [num_samples, a_Frames, b_Frames]


def read_mesh(filename):
    '''
    Reads in the mesh file and saves the triangle vertices as a list.
    Inputs:
    filename -> the mesh file
    Outputs:
    vertices -> a dictionary of all the vertices, with indices as keys
    triangles -> a list of triangles, each with 3 vertices
    '''

    mesh_data = open(filename)
    # get file information from first line
    num_vertices = int(mesh_data.readline().strip())
    vertices = {}
    for i in range(num_vertices):
        current_vertices = mesh_data.readline().split()
        x = float(current_vertices[0].strip())
        y = float(current_vertices[1].strip())
        z = float(current_vertices[2].strip())
        # add vertex coordinates to dictionary
        vertices[i] = [x, y, z]

    triangles = []
    num_indices = int(mesh_data.readline().strip())
    for i in range(num_indices):
        current_indices = mesh_data.readline().split()
        i1 = int(current_indices[0].strip())
        i2 = int(current_indices[1].strip())
        i3 = int(current_indices[2].strip())
        # get vertexes from vertex dictionary, create triangle list
        triangles.append([vertices[i1], vertices[i2], vertices[i3]])

    return [vertices, triangles]
