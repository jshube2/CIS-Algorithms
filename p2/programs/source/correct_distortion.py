import numpy as np
import math

'''
Inputs:
q -> set of points with errors
c -> Bernstein polynomial coefficient matrix

Outputs:
p -> points q with distortion corrected
'''
def correct_distortion(q, c, boundbox):
    n = 5
    q_min = boundbox[0]
    q_max = boundbox[1]

    u = ScaleToBox(q, q_min, q_max)

    p = np.zeros([len(q), 3])
    s = 0
    for u_s in u:
        u_x = u_s[0]
        u_y = u_s[1]
        u_z = u_s[2]
        for i in range(n+1):
            for j in range(n+1):
                for k in range(n+1):
                    index = (n+1)**2 * i + (n+1) * j + k
                    p[s] += c[index]*Bern(i, n, u_x)*Bern(j, n, u_y)*Bern(k, n, u_z)
        s += 1

    F_inverse = np.linalg.pinv(p)

    return p


'''
Inputs:
p -> set of 'ground truth' points
e.g. C_k expected
q -> set of calculated, error-ridden points
e.g. C_k calculated from other frame transformations
n -> The order of the bernstein polynomial to calculate
Default is 5 for this assignment

Outputs:
c -> n^3x3 5th order Bernstein polynomial coefficient matrix
'''
def calibrate_distortion(p, q, n=5):

    q_min = get_min(q)
    q_max = get_max(q)
    boundbox = [q_min, q_max]
    u = ScaleToBox(q, q_min, q_max)

    F = []
    for u_s in u:
        F_s=[]
        u_x = u_s[0]
        u_y = u_s[1]
        u_z = u_s[2]
        for i in range(n+1):
            for j in range(n+1):
                for k in range(n+1):
                    F_s.append(Bern(i, n, u_x)*Bern(j, n, u_y)*Bern(k, n, u_z))
        F.append(F_s)

    F = np.array(F)

    F_inverse = np.linalg.pinv(F)
    c = np.dot(F_inverse, p)

    return [c, boundbox]


'''
Inputs:
q, q_min, q_max -> distorted points

Outputs:
u -> returns new bounded set from 0 to 1
'''
def ScaleToBox(q, q_min, q_max):
    u = (q - q_min) / (q_max - q_min)
    return u


'''
Computes the nth order Bernstein polynomial
nth order with x bound between 0 and 1
'''
def Bern(a, n, x):
    b = (np.math.factorial(n))/(np.math.factorial(a)*np.math.factorial(n-a))
    B = b*(x**a)*(1-x)**(n-a)
    return B


'''
Returns min value for x, y, z
'''
def get_min(q):
    q_min = np.zeros(3)
    # print "q: " + str(q[0:5])
    # print "q[0,:]: " + str([point[0] for point in q])
    q_min[0] = min([point[0] for point in q])
    q_min[1] = min([point[1] for point in q])
    q_min[2] = min([point[2] for point in q])
    return q_min


'''
Returns max value for x, y, z 
'''
def get_max(q):
    q_max = np.zeros(3)
    q_max[0] = max([point[0] for point in q])
    q_max[1] = max([point[1] for point in q])
    q_max[2] = max([point[2] for point in q])
    return q_max

