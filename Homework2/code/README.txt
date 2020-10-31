
=======================================
ASEN 5519 ALGORITHMIC MOTION PLANNING 
HOMEWORK 2
JOE MICELI
=======================================

NOTES ON SOLUTION LOCATIONS:
- Solutions to problems 1, 2, 3a, 4, and 6 were provided as pdf 
- Solution to 3b is provided in Homework2_Exercise3b_Solution.txt
- Solutions to 5a and 5b are shown in the figures Exercise_5_a.png and Exercise_5_b.png respectively
	- These figures can be generated using Minkowski.py and Minkowski2.py respectively
- Solution to problem 7 (part 1) is 3LinkManipulator_fwd.py
- Solution to problem 7 (part 2) is 3LinkManipulator_reverse.py

DEPENDENCIES:
- Python3

RUNNING THE CODE:
- Neither script takes any arguments but both will prompt the user for inputs
- Simply follow the onscreen directions
- Syntax for 3LinkManipulator_fwd.py
	$ python 3LinkManipulator_fwd.py
- Syntax for 3LinkManipulator_reverse.py
	$ python 3LinkManipulator_reverse.py




EDIT (12/10/2020): SUBMISSION OF PROBLEM 8
- I am resubmitting the code of this assignment to include my work for Probem 8
- I used eclipse and it's internal tool chain to build this and unfortunately, could not get it to run outside of eclipse
- The code is included in this submission
- Plots from parts a,b, and c are included as well in code\Exercise8\solutions
- File Descriptions (in \code\Exercise8\src):
	- *.hpp and *.cpp are the source files for c_space constructor
	- c_space.csv is the log file containing the set of comma separated c-space coordinates of each discretized obstacle
	- workspace.csv is the log file contianing the set of comma separated workspace coordinates of each discretized obstacle
	- plot_spaces.py will generate plots of both the workspace and c-space using the .csv log files
	- Remaining files and directories are artifacts from the eclipse IDE


