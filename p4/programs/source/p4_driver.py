import numpy as np
import sys, os
import time
import ICPmatch as icpm
import ICPread as icpf
import data_methods as dm


def main():
    """
    Main method, reads input and generates finalized output of selected ICP method.
    """

    start_time = time.time()

    # prompt for file name, data name, and icp type
    if len(sys.argv) != 3:
        print "Please input as follows: python p4_driver.py sample_readings (linear/sphere/tree)"
        sys.exit(0)

    # get the data name and icp type as user input
    sample_data = sys.argv[1]
    icp_type = sys.argv[2]

    # set data file paths
    rigid_a_file = '../data/Problem4-BodyA.txt'
    rigid_b_file = '../data/Problem4-BodyB.txt'
    mesh_data_file = '../data/Problem4MeshFile.sur'
    sample_readings_file = '../data/PA4-' + sample_data + '-SampleReadingsTest.txt'

    # get triangle mesh, body, and frame data
    tri_coords, tri_ind = icpf.read_mesh(mesh_data_file)
    nledA, ledA, tipA = icpf.read_rigid_body(rigid_a_file)
    nledB, ledB, tipB = icpf.read_rigid_body(rigid_b_file)
    aFrames, bFrames = icpf.read_sample(sample_readings_file, nledA, nledB)

    # calculate the set of points d for icp matching
    d = icpm.icp_get_d(aFrames, bFrames, ledA, tipA, ledB)

    # get closest triangle point and final F_reg
    c, F_reg = icpm.icp_match_iterator(tri_coords, tri_ind, d, icp_type)

    # get distance between c and s for output file
    s = dm.transform3D(d, F_reg)
    dist = icpm.get_distance(s, c)

    # set output file paths
    outfile_dir = '../output/PA4-' + sample_data + '-Output.txt'
    outfile_name = 'PA4-' + sample_data + '-Output.txt'
    outfile = open(outfile_dir, 'w')

    # write output file header
    outfile.write(str(np.shape(d)[1]) + ' ' + outfile_name + '\n')

    # Write formatted output to file
    total_diff = 0
    for i in range(np.shape(d)[1]):
        total_diff += dist[i]

    average_diff = total_diff / (np.shape(d)[1])

    for i in range(np.shape(d)[1]):
        outfile.write('{: >8}{: >9}{: >9}'.format(format(d[0][i], '.2f'), format(d[1][i], '.2f'), format(d[2][i], '.2f')))
        outfile.write('{: >13}{: >9}{: >9}'.format(format(c[0][i], '.2f'), format(c[1][i], '.2f'), format(c[2][i], '.2f')))
        outfile.write('{: >10}\n'.format(format(dist[i], '.3f')))

    outfile.close()
    print "Total time: " + str(time.time() - start_time)

if __name__ == '__main__':
    main()
