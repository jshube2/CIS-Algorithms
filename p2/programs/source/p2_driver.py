import numpy as np
import sys
import math
import os.path
from read_datasets import read_cal_readings, read_cal_body, read_em_pivot, read_opt_pivot, read_ct_fiducials
from calibrate_pivot import *
from correct_distortion import correct_distortion, calibrate_distortion

'''
Runs the full program for assignment 2. For each of the a-j datasets,
reads in the data for readings, body, and pivots, and performs point
cloud transformations and pivot calibration functions. Outputs results
in the output folder, with one file per dataset. Also outputs output-2
files, the positions of the probe tip in CT coordinates.
'''
def main():
    num_datasets = 10
    file_letter = 'a'

    for dataset_num in range(num_datasets):
        file_starter = "../data/pa2-debug-"
        file_name = file_starter + file_letter + "-output1.txt"
        open_test = os.path.isfile(file_name)
        if not open_test:
            file_starter = "../data/pa2-unknown-"

        cal_readings = file_starter + file_letter + "-calreadings.txt"
        cal_body = file_starter + file_letter + "-calbody.txt"
        em_pivot = file_starter + file_letter + "-empivot.txt"
        opt_pivot = file_starter + file_letter + "-optpivot.txt"
        ct_fiducials = file_starter + file_letter + "-ct-fiducials.txt"
        fiducials = file_starter + file_letter + "-em-fiducialss.txt"
        em_nav = file_starter + file_letter + "-EM-nav.txt"

        # Read all input files
        [D, A, C, readings_frames] = read_cal_readings(cal_readings)
        [d, a, c] = read_cal_body(cal_body)
        [G, G_frames] = read_em_pivot(em_pivot)
        [D_pivot, H, H_frames] = read_opt_pivot(opt_pivot)
        [Fid, Fid_frames] = read_em_pivot(fiducials)
        CTFid = read_ct_fiducials(ct_fiducials)
        [EMNav, EMNav_frames] = read_em_pivot(em_nav)


        # returns the [x,y,z] of em pivot as a vector
        em_pivot_calibration = calibrate_pivot(G, G_frames) 
        # returns the [x,y,z] of opt pivot as a vector
        opt_pivot_calibration = opt2em_calibrate_pivot(H, D_pivot, d, H_frames)


        # Part 1: Generate expected C
        dstart = 0
        astart = 0;
        c_expected_list = []
        for i in range(0, readings_frames):
            [RD, TD] = get_transform(D[dstart:dstart+len(d)], d)
            [RA, TA] = get_transform(A[dstart:dstart+len(d)], a)
            ca_expected = transform(RA, c.T, TA)
            #print 'ca_expected: ' + str(ca_expected)
            RDi = np.transpose(RD)
            tDi = -1*np.dot(RDi, TD)
            c_expected = transform(RDi, ca_expected, tDi)
            #print "c expected: " + str(c_expected)
            c_expected_list.append(c_expected)

            dstart = dstart + len(d)
            astart = astart + len(a)


        C_calc = []
        
        for i in range(readings_frames):
            for j in range(len(c_expected_list[0][0])):
                point = [None] * 3
                point[0] = c_expected_list[i][0][j]
                point[1] = c_expected_list[i][1][j]
                point[2] = c_expected_list[i][2][j]
                C_calc.append(point)
        
        
        # Part 2:
        # Calculate distortion correction function
        [polynomial_coeff, boundbox] = calibrate_distortion(C, C_calc, 5)
        # print "polynomial_coeff: " + str(polynomial_coeff)
        

        # Part 3:
        # Repeat pivot calibration with distortion correction        
        G_corrected = np.array(G)
        n_g = len(G)/G_frames

        # Correct for distortion frame by frame
        G_corrected = correct_distortion(G, polynomial_coeff, boundbox)
        # print "G: " + str(G)
        # print "G_corrected: " + str(G_corrected)

        # Return x,y,z of EM pivot in a vector
        em_pivot_calibration, p_tip = calibrate_pivot(G_corrected, G_frames)


        # Part 4:
        # Calculate positions of the fiducial pins
        n_fid = len(Fid)/Fid_frames
        fiducial_locations = []

        for i in range(Fid_frames):
            G_corr_frame = G_corrected[i*n_fid:(i+1)*n_fid]
            FidData = Fid[i*n_fid:(i+1)*n_fid]
            FrameTrans = get_transform(G_corr_frame, FidData)
            # print 'Frame Trans: ', FrameTrans
            # fiducial_locations.append(calibrate_pivot_one_frame(Fid[i*n_fid:(i+1)*n_fid], p_tip))
            fiducial_locations.append(transform_3D(FrameTrans, em_pivot_calibration))

        
        # Part 5:
        # Calculate the registration transformation between CT image and EM tracker
        CTFid = np.array(CTFid)
        F_reg = get_transform(fiducial_locations, CTFid)

        
        # Part 6:
        # Compute the EM tracked tool's coordinates in the CT image
        v = []
        EMNav_corrected = np.array(EMNav)
        n_nav = len(EMNav)/EMNav_frames
        EMNav_corrected = correct_distortion(EMNav, polynomial_coeff, boundbox)

        for i in range(0, EMNav_frames):
            # Correct for distortion frame by frame
            G_corr_frame = G_corrected[i*n_nav:(i+1)*n_nav]
            EMNAV_frame = EMNav_corrected[i*n_nav:(i+1)*n_nav, :]
            v.append(get_tip_coordinates(em_pivot_calibration, EMNAV_frame, G_corr_frame,F_reg))



        # Create output 1 file
        output_file = "../output/pa2-" + file_letter + "-output-1.txt"
        output_file_name = "pa2-" + file_letter + "-output-1.txt"
        output = open(output_file, 'w+')

        output.write(output_file_name + "\n")
        output.write("EM pivot post est position: ")
        output.write(str(em_pivot_calibration[0]) + ",\t" + str(em_pivot_calibration[1]) + ",\t" + str(em_pivot_calibration[2]) + "\n") 
        output.write("Optical pivot post est position: ")
        output.write(str(opt_pivot_calibration[0][0]) + ",\t" + str(opt_pivot_calibration[0][1]) + ",\t" + str(opt_pivot_calibration[0][2]) + "\n") 
       
        for c in c_expected_list:
            numrow, numcol = c.shape
            for row in range(numcol):
                outstring = ''
                for col in range(numrow):
                    outstring += str(c[col, row]) + ',   '
                output.write(outstring)
                output.write('\n')
        
        output.close()


        # Create output 2 file
        output_file = "../output/pa2-" + file_letter + "-output-2.txt"
        output_file_name = "pa2-" + file_letter + "-output-2.txt"
        output = open(output_file, 'w+')

        output.write(str(len(v)) + ", " + output_file_name + "\n")
        for v_point in v:
            outstring = ''
            for v_val in v_point:
                outstring += str(v_val.round(2)) + ',   '
            output.write(outstring)
            output.write('\n')

        file_letter = chr(ord(file_letter) + 1)
        output.close()

    print("Program completed successfully!")


'''
Applies transform, returns R*x + t
'''
def transform(R, x, t):
    rx = np.dot(R,x)
    for i in range(3):
        for j in range(rx[0].size):
            rx[i][j] -= t[i]

    return rx


if __name__ == "__main__":
    main()