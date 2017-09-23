import numpy as np

def EMCalibration(G, frames):
    G = np.array(G)
    num_readings = len(G) / frames
    G1 = G[0:num_readings]
    sum_g = np.zeros((3,1))
    for i in range(0, len(G1)):
        sum_g = np.add(sum_g, G1[i])

    Go = sum_g / len(G1)

    for i in range(0:len(G1)):
        g1[i] = point3D(G1[i][0] - Go[0], G1[i][1] - Go[1], G1[i][2] - Go[2])

    FG = np.zeros((1, frames))
    for k in range(0, frames):
        Gk = G[num_readings*k:num_readings*(k+1)]
        FG[k] = getTransformation(g1, Gk)

    for k in range(0, frames):
        

