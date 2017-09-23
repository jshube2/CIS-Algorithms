import unittest

import numpy as np
from numpy import matrix
from numpy import linalg
from numpy import testing
import sys
import math
import os.path
from read_datasets import read_cal_readings, read_cal_body, read_em_pivot, read_opt_pivot, read_ct_fiducials
from transformation import get_transform
from calibrate_pivot import calibrate_pivot, opt2em_calibrate_pivot, localize, compute_midpoint, transform_3D, get_tip_coordinates
from correct_distortion import correct_distortion, calibrate_distortion
from p2_driver import transform

class p2_tests(unittest.TestCase):

    '''
    Tests that the datasets are properly loaded into numpy arrays.
    '''
    def test_readings(self):
        cal_readings = "../data/pa2-unknown-h-calreadings.txt"
        [D, A, C, frames] = read_cal_readings(cal_readings)
        self.assertEqual(frames, 125)
        d1 = np.array([-12.30, -5.28, -1499.94])
        self.assertEqual(D[0].all(), d1.all())
        a1 = np.array([89.53, 96.38, -1400.45])
        self.assertEqual(A[0].all(), a1.all())
        c1 = np.array([97.77, 100.38, 226.55])
        self.assertEqual(C[0].all(), c1.all())


    '''
    Tests that the get_transform function correctly determines the
    rotation and translation components of the registration.
    '''
    def test_transform(self):
        a = np.array([[5,7,1], [6,3,2], [6,9,1], [1,3,7], [8,5,3]])
        rotation = np.array([[1,0,0], [0,1,0], [0,0,1]])
        translation = np.array([6,3,5])
        ra = np.dot(rotation, a.T)
        b = np.add(ra, np.vstack(translation))
        [R, t] = get_transform(a, b.T)
        self.assertEqual(R.all(), rotation.all())
        self.assertEqual(t.all(), translation.all())


    '''
    Perform get_transform from matrix a to matrix b, as well as from
    matrix b to matrix a. Then, multiply these resultant matrices together
    and add both of their translations. If the resultant matrix is the 
    identity, we have verified get_transform is working in both directions.
    '''
    def test_transform_2(self):
        a = np.array([[5,7,1], [6,3,2], [6,9,1], [1,3,7], [8,5,3]])
        rotation = np.array([[1,0,0], [0,1,0], [0,0,1]])
        translation = np.array([6,3,5])
        identity = np.array([[1,0,0], [0,1,0], [0,0,1]])
        ra = np.dot(rotation, a.T)
        b = np.add(ra, np.vstack(translation))
        [R1, t1] = get_transform(a, b.T)
        [R2 ,t2] = get_transform(b.T, a)
        result = np.add(np.add(np.dot(R1, R2), t1), t2)
        self.assertEqual(result.all(), identity.all())


    '''
    Runs localize function to ensure it is working properly.
    '''
    def test_localize(self):
        a = np.array([[1,3,5], [5,7,9], [4,3,2]])
        a_mean = np.array([2,2,2])
        result = localize(a_mean, a)
        true_result = np.array([[-1,1,3], [3,5,7], [2,1,0]])
        self.assertEqual(result.all(), true_result.all())


    '''
    Runs compute midpoint function to ensure it is working properly.
    '''
    def test_compute_midpoint(self):
        a = np.array([[1,3,5], [5,7,8], [6,2,2]])
        a_mid = compute_midpoint(a)
        a_mid_true = np.array([4, 4, 5])
        self.assertEqual(a_mid.all(), a_mid_true.all())



    '''
    Runs calibrate_distortion and correct_distorion, and compares the resulting
    corrected G values against the ground truth, ensuring each point is within an
    error threshold.
    '''
    def test_correct_distortion(self):
        file_letter = "a"
        file_starter = "../data/pa2-debug-"
        file_name = file_starter + file_letter + "-output1.txt"

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

        ''' CORRECT DISTORTION TEST '''
        g_num = 0
        # iterate through the points in G
        for g_c in G_corrected:
            g_test = G[g_num]
            point_num = 0
            for g_point in g_c:
                # check that corrected in certain error threshold of ground truth G
                g_diff = abs(g_point - g_test[point_num])
                #print "g_diff: " + str(g_diff)
                diff_ratio = g_diff / g_test[point_num]
                #print "diff_ratio: " + str(diff_ratio)
                point_num += 1
                self.assertTrue(diff_ratio < 0.25)
            g_num += 1


        # Part 4:
        # Calculate positions of the fiducial pins
        n_fid = len(Fid)/Fid_frames
        fiducial_locations = []

        for i in range(Fid_frames):
            G_corr_frame = G_corrected[i*n_fid:(i+1)*n_fid]
            FidData = Fid[i*n_fid:(i+1)*n_fid]
            FrameTrans = get_transform(G_corr_frame, FidData)
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

        p2_output = "../data/pa2-debug-" + file_letter + "-output2.txt"
        p2_output_name = "pa2-debug-" + file_letter + "-output2.txt"
        output = open(p2_output, 'r')
        output_val = []

        first_line = output.readline()
        for i in range(4):
            current_line = output.readline().split(",")
            for x in current_line:
                x.strip()

            x_pos = float(current_line[0])
            y_pos = float(current_line[1])
            z_pos = float(current_line[2])
            point = [x_pos, y_pos, z_pos]
            output_val.append(point)   
            

        ''' TEST THAT FINAL OUTPUT WITHIN THRESHOLD '''
        for i in range(4):
            for j in range(3):
                output_point = output_val[i][j]
                v_point = v[i][j]    
                point_diff = abs(output_point - v_point)
                diff_ratio = point_diff / output_point
                # print "diff ratio: " + str(diff_ratio)  
                self.assertTrue(diff_ratio < 0.15)
        




if __name__ == "__main__":
    unittest.main()
