import numpy as np
import sys
from ICPmatch import icp_get_d, icp_match, get_distance
from ICPread import read_rigid_body, read_sample_readings, read_mesh

def main():
    '''
    Program driver, taking sample readings letter and data type, write output
    file to output folder.
    '''

    # input sample readings letter and type
    if (len(sys.argv) != 2):
        print("Please input as follows: python p3_driver.py  sample_readings")
        sys.exit(0)

    # set all file paths for data loading
    sample_data = sys.argv[1]
    rigid_a_file = '../data/Problem3-BodyA.txt'
    rigid_b_file = '../data/Problem3-BodyB.txt'
    mesh_data_file = '../data/Problem3MeshFile.sur'
    sample_readings_file = '../data/PA3-' + sample_data + '-SampleReadingsTest.txt'

    # load data for rigid bodies, mesh file, and sample readings
    [num_a_markers, led_a_markers, a_tip] = read_rigid_body(rigid_a_file)
    [num_b_markers, led_b_markers, b_tip] = read_rigid_body(rigid_b_file)
    [num_frames, a_Frames, b_Frames] = read_sample_readings(sample_readings_file, num_a_markers, num_b_markers)
    [vertices, triangles] = read_mesh(mesh_data_file)

    # Calculate set of points d for ICP matching
    d = icp_get_d(num_frames, a_Frames, b_Frames, led_a_markers, led_b_markers, a_tip)
    # Find matches between set of points d and mesh file
    c = icp_match(triangles,d)

    # Create output file
    outfile_dir = '../output/PA3-' + sample_data + '-Output.txt'
    outfile_name = 'PA3-' + sample_data + '-Output.txt'
    outfile = open(outfile_dir, 'w')
    outfile.write(str(len(d)) + ' ' + outfile_name + '\n')

    # Write formatted output to file
    for i in range(len(d)):
        outfile.write('{: >8}{: >9}{: >9}'.format(format(d[i][0], '.2f'), format(d[i][1], '.2f'), format(d[i][2], '.2f')))
        outfile.write('{: >13}{: >9}{: >9}'.format(format(c[i][0], '.2f'), format(c[i][1], '.2f'), format(c[i][2], '.2f')))
        outfile.write('{: >10}\n'.format(format(get_distance(d[i], c[i]),  '.3f')))

    outfile.close()


if __name__ == '__main__':
    main()
