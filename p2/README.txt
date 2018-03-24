--EXECUTING PROGRAM--
To run the program, navigate to programs/source/p2_driver.py
Enter the following:
python p2_driver.py

For unit testing, enter the following:
python p2_tests.py

This will generate all output files outlined in the program report. These files
can be found at programs/output, with names -output1 and -output-difference for
each of the provided datasets. Input datasets may be found at programs/data.


--REPORT--
The program report may be found in the root directory, as p1_report.pdf.


--FILE LISTING--
A listing and description of each of the source files is as follows, and may
also be found in the program report:

p2_driver.py: The main program driver, taking in each input dataset and producing outputs.

p2_tests.py: The unit test driver, running tests on the core subroutines of the program.

read_datasets.py: Contains methods for loading each of the dataset parts into the program.

transformation.py: Contains the get_transform method for returns the R and t components of the point cloud transformation.

calibrate_pivot.py: Contains all pivot calibration functions for finding the estimated em and opt_pivot coordinates.

correct_distortion.py: Contains all distortion calibration and correction functions for assignment 2.


