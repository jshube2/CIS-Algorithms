import numpy as np
from numpy import matrix
from numpy import linalg
import sys
import math
import os.path
import linecache
from read_datasets import read_cal_readings, read_cal_body, read_em_pivot, read_opt_pivot
from transformation import get_transform
import calibrate_pivot
from calibrate_pivot import calibrate_pivot, opt2em_calibrate_pivot

'''
Runs the full program for assignment 1. For each of the a-k datasets,
reads in the data for readings, body, and pivots, and performs point
cloud transformations and pivot calibration functions. Outputs results
in the output folder, with one file per dataset. Also outputs the difference
between C and C_expected for each dataset, output as -output-difference
for each dataset
'''
def main():
    # the total number of datasets to be run on
    num_datasets = 11
    # the initial dataset letter
    file_letter = 'a'

    for dataset_num in range(num_datasets):

        # strings for building the requested file address
        file_starter = "../data/pa1-debug-"
        file_name = file_starter + file_letter + "-output1.txt"
        open_test = os.path.isfile(file_name)
        # if file is not debug, change starter to unknown
        if not open_test:
            file_starter = "../data/pa1-unknown-"

        # file endings for each dataset
        cal_readings = file_starter + file_letter + "-calreadings.txt"
        cal_body = file_starter + file_letter + "-calbody.txt"
        em_pivot = file_starter + file_letter + "-empivot.txt"
        opt_pivot = file_starter + file_letter + "-optpivot.txt"

        # run parser functions for each dataset, saving return values
        # as numpy arrays
        [D, A, C, frames] = read_cal_readings(cal_readings)
        [d, a, c] = read_cal_body(cal_body)
        [G, G_frames] = read_em_pivot(em_pivot)
        [D_pivot, H, H_frames] = read_opt_pivot(opt_pivot)

        # returns the [x,y,z] of em pivot as a vector
        em_pivot_calibration = calibrate_pivot(G, G_frames) 
        # returns the [x,y,z] of opt pivot as a vector
        opt_pivot_calibration = opt2em_calibrate_pivot(H, D_pivot, d, H_frames)

        # the current frame of d and a to be transformed
        dstart = 0
        astart = 0;
        c_expected_list = []
        for i in range(0, frames):
            # get the rotation and translation matrices between the
            # reading and body values for D and A
            [RD, TD] = get_transform(D[dstart:dstart+len(d)], d)
            [RA, TA] = get_transform(A[dstart:dstart+len(d)], a)

            # first apply the A transformation to body frame c
            ca_expected = transform(RA, c.T, TA)
            RDi = np.transpose(RD)
            tDi = -1*np.dot(RDi, TD)
            # then apply the inverse D transform
            c_expected = transform(RDi, ca_expected, tDi)
            # append the result
            c_expected_list.append(c_expected)

            # iterate to the next frame
            dstart = dstart + len(d)
            astart = astart + len(a)
            
        # strings for building the output file    
        output_file = "../output/pa1-" + file_letter + "-output-1.txt"
        output_file_name = "pa1-" + file_letter + "-output-1.txt"
        output = open(output_file, 'w+')

        # writes the expected em and opt pivot positions
        output.write("EM pivot post est position: ")
        output.write(str(em_pivot_calibration[0][0]) + ",\t" + str(em_pivot_calibration[0][1]) + ",\t" + str(em_pivot_calibration[0][2]) + "\n") 
        output.write("Optical pivot post est position: ")
        output.write(str(opt_pivot_calibration[0][0]) + ",\t" + str(opt_pivot_calibration[0][1]) + ",\t" + str(opt_pivot_calibration[0][2]) + "\n") 

        # iterates through expected c frames and prints them to output file
        for c in c_expected_list:
            numrow, numcol = c.shape
            for row in range(numcol):
                outstring = ''
                for col in range(numrow):
                    outstring += str(c[col, row]) + ',   '
                output.write(outstring)
                output.write('\n')
        
        output.close()

        # Print the difference between each C and C_expected
        output_diff_file = "../output/pa1-" + file_letter + "-output-difference.txt"
        open_test = os.path.isfile(file_name)
            
        output_diff = open(output_diff_file, 'w+')

        # iterate through c_expected
        c_num = 0
        total_diff = 0
        for c in c_expected_list:
            for row in range(numcol):
                outstring = ''
                for col in range(numrow):
                    # find difference in C
                    c_diff = c[col, row] - C[c_num, col]
                    # add to total to find average difference
                    total_diff += abs(c_diff)
                    # print "Next C: " + str(C[c_num, col])
                    outstring += str(c_diff) + ',   '
                output_diff.write(outstring)
                output_diff.write('\n')
                c_num += 1

        average_diff = total_diff / (c_num * 3)
        # Print the average C error
        output_diff.write("Average difference: " + str(average_diff))

        # Increment to next letter dataset
        file_letter = chr(ord(file_letter) + 1)
        output_diff.close()

    print("Program ran successfully! Check output folder for output files for each dataset.")


# Returns resultant matrix from rotation and transformation
def transform(R, x, t):
    rx = np.dot(R,x)

    for i in range(3):
        for j in range(rx[0].size):
            rx[i][j] -= t[i]

    return rx


if __name__ == "__main__":
    main()