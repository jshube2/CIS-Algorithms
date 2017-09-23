import unittest

import numpy as np
from numpy import matrix
from numpy import linalg
from numpy import testing
import sys
import math
import os.path
from read_datasets import read_cal_readings, read_cal_body, read_em_pivot, read_opt_pivot
from transformation import get_transform
from calibrate_pivot import calibrate_pivot, opt2em_calibrate_pivot, localize, compute_midpoint

class p1_tests(unittest.TestCase):

    '''
    Tests that the datasets are properly loaded into numpy arrays.
    '''
    def test_readings(self):
        cal_readings = "../data/pa1-unknown-h-calreadings.txt"
        [D, A, C, frames] = read_cal_readings(cal_readings)
        self.assertEqual(frames, 8)
        d1 = np.array([12.17, 3.54, -1499.95])
        self.assertEqual(D[0].all(), d1.all())
        a1 = np.array([223.42, 210.40, -1286.82])
        self.assertEqual(A[0].all(), a1.all())
        c1 = np.array([199.23, 238.58, 219.64])
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

if __name__ == "__main__":
    unittest.main()
