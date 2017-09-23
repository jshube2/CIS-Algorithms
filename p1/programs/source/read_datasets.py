import numpy as np

'''
Parses through the cal-readings file and returns numpy arrays
for D, A, and C.
Inputs: the file to parse through
Outputs: the D, A, and C values for each frame of data as numpy arrays,
and the number of frames
'''
def read_cal_readings(filename):
    D = []
    A = []
    C = []
    cal_readings = open(filename, 'r')
    # get list of elements divided by commas
    first_line = cal_readings.readline().split(",")
    # strip whitespace out of elements
    for x in first_line:
        x.strip()

    # determine the number of elements in each frame
    num_D = int(first_line[0])
    num_A = int(first_line[1])
    num_C = int(first_line[2])
    # determine the number of frames
    num_repeat = int(first_line[3])
    num_frames = num_repeat

    for k in range(1, num_repeat + 1):

        offset = k - 1

        # iterate through the next num_D elements
        for i in range(offset*num_D, k*num_D):
            # split on commas and remove spaces
            current_line = cal_readings.readline().split(",")
            for x in current_line:
                x.strip('\n')

            x_pos = float(current_line[0])
            y_pos = float(current_line[1])
            z_pos = float(current_line[2])
            point = [x_pos, y_pos, z_pos]
            # append 1x3 array
            D.append(point)

        # iterate through the next num_A elements
        for i in range(offset*num_A, k*num_A):
            # split on commas and remove spaces
            current_line = cal_readings.readline().split(",")
            for x in current_line:
                x.strip()

            x_pos = float(current_line[0])
            y_pos = float(current_line[1])
            z_pos = float(current_line[2])
            point = [x_pos, y_pos, z_pos]
            # append 1x3 array
            A.append(point)

        # iterate through the next num_C elements
        for i in range(offset*num_C, k*num_C):
            current_line = cal_readings.readline().split(",")
            for x in current_line:
                x.strip()

            x_pos = float(current_line[0])
            y_pos = float(current_line[1])
            z_pos = float(current_line[2])
            point = [x_pos, y_pos, z_pos]
            # append 1x3 array
            C.append(point)

    cal_readings.close()
    # convert to numpy arrays for future transformations
    D = np.array(D)
    A = np.array(A)
    C = np.array(C)
    return [D, A, C, num_frames]


'''
Parses through the cal-body file and returns numpy arrays
for d, a, and c.
Input: the file to be parsed through
Outputs: the d, a, and c values for the frame
'''
def read_cal_body(filename):
    d = []
    a = []
    c = []
    cal_body = open(filename, 'r')
    # get list of elements divided by commas
    first_line = cal_body.readline().split(",")
    # strip whitespace out of elements
    for x in first_line:
        x.strip()

    # determine the number of points in each frame
    num_d = int(first_line[0])
    num_a = int(first_line[1])
    num_c = int(first_line[2])

    # iterate through the num_d points
    for i in range(0,num_d):
        current_line = cal_body.readline().split(",")
        for x in current_line:
            x.strip()

        x_pos = float(current_line[0])
        y_pos = float(current_line[1])
        z_pos = float(current_line[2])
        point = [x_pos, y_pos, z_pos]
        d.append(point)

        # iterate through the a points
    for i in range(0,num_a):
        current_line = cal_body.readline().split(",")
        for x in current_line:
            x.strip()

        x_pos = float(current_line[0])
        y_pos = float(current_line[1])
        z_pos = float(current_line[2])
        point = [x_pos, y_pos, z_pos]
        a.append(point)

    # iterate through the c points
    for i in range(0,num_c):
        current_line = cal_body.readline().split(",")
        for x in current_line:
            x.strip()

        x_pos = float(current_line[0])
        y_pos = float(current_line[1])
        z_pos = float(current_line[2])
        point = [x_pos, y_pos, z_pos]
        c.append(point)

    # convert to numpy arrays for future transformations
    d = np.array(d)
    a = np.array(a)
    c = np.array(c)

    cal_body.close()
    return [d, a, c]  


'''
Parses through the em-pivot file and returns the array G.
Input: the file to be parsed
Outputs: The G frames of EM probe data, and number of frames
'''
def read_em_pivot(filename):
    G = []
    em_pivot = open(filename, 'r')

    first_line = em_pivot.readline().split(",")
    for x in first_line:
        x.strip

    num_g = int(first_line[0])
    num_repeat = int(first_line[1])
    num_frames = num_repeat

    # iterate through the frames
    for k in range(1,num_frames + 1):
        offset = k - 1;

        # iterate through the points
        for i in range(offset*num_g, k*num_g):
            current_line = em_pivot.readline().split(",")
            for x in current_line:
                x.strip()
            x_pos = float(current_line[0])
            y_pos = float(current_line[1])
            z_pos = float(current_line[2])
            point = [x_pos, y_pos, z_pos]
            G.append(point)

    em_pivot.close()
    return [G, num_frames]


'''
Parses through the opt-pivot file and returns the D and H arrays.
Input: the file to be parsed
Outputs: The optical probe measurements D and H, and the number 
of frames.
'''
def read_opt_pivot(filename):
    D = []
    H = []
    opt_pivot = open(filename, 'r')

    first_line = opt_pivot.readline().split(",")
    for x in first_line:
        x.strip

    num_D = int(first_line[0])
    num_H = int(first_line[1])
    num_repeat = int(first_line[2])
    num_frames = num_repeat

    # iterate through the frames
    for k in range(1, num_frames + 1):
        offset = k - 1

        # iterate through the next num_D points
        for i in range(offset*num_D, k*num_D):
            current_line = opt_pivot.readline().split(",")
            for x in current_line:
                x.strip()
            x_pos = float(current_line[0])
            y_pos = float(current_line[1])
            z_pos = float(current_line[2])            
            point = [x_pos, y_pos, z_pos]
            D.append(point)

        # iterate through the next num_H points
        for i in range(offset*num_H, k*num_H):
            current_line = opt_pivot.readline().split(",")
            for x in current_line:
                x.strip()
            x_pos = float(current_line[0])
            y_pos = float(current_line[1])
            z_pos = float(current_line[2])            
            point = [x_pos, y_pos, z_pos]
            H.append(point)

    opt_pivot.close()
    return [D, H, num_frames]