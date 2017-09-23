# Computer Integrated Surgery, EN.600.645
# Joshua Shubert, Nathan Smith
# Programming Assignment 1

import sys
import numpy as np 
import math
import get_transform

# Rip data from calibration object txt file and build calibration object object
# CalReadings object with a init function that takes in data, creates D, A, C, frame variables

# Rip data from calibration readings txt file and build calibration readings object
# CalBody object with an init function that takes in dta, creates d, a, c variables

# Rip data from empivot txt file and build empivot object
# EmPivot object with an init function that takes in dta, creates G and frame variables

# Rip data from optipivot txt file and build optipivot object

# Function to calculate transformation matrix F between two sets of 3d points
# See Rigid3d3d class slides
# d (and also D) should be a 3x3 matrix
# of the form [[x1 y1 z1], [x2 y2 z2], [x3 y3 z3]]
#d = np.concatenate(cal_body.d[0],cal_body.d[1],cal_body.d[2])
# For cal_body.d I am assuming it is a Nx3 matrix where each column corresponds to x,y,z and each row is the ith point
#D = np.concatenate(cal_reading.D[0][0],cal_reading.D[0][1],cal_reading.D[0][1])
# For cal_reading.D I am assuming it is a MxNx3 cube matrix where each column corresponds to x,y,z and each row is the ith point
# And each level of the matrix corresponds to a data frame at a time 't'
# Pass get_transform two 3x3 matrices where each row the matrix is a x,y,z vector for a point
#F_D = get_transform(D,d)

# Compute expected C_i using F_d^-1 * F_a * c_i

# Function that compares vector of calculated C_i with given C_i
# Perhaps print avg and max error ABC


