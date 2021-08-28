=======================================

ASEN 5519 ALGORITHMIC MOTION PLANNING
FINAL PROJECT
JOE MICELI
13Dec2020

=======================================


==== DEPENDENCIES ====
- Python3 (for plotting the results)
- Linux OS (for running the executable)


==== RUNNING THE CODE ====
- Execution has only been tested on a virtual machine running Ubuntu 14.04
- Executable is provided as "FinalProject" 
- Can be executed from command line using
	./FinalProject

==== PLOTTING RESULTS ====
- Code is currently configured to run SST_DEFAULT (OMPL's defualt configuration of SST) and a benchmark of 10 different SST configurations defined (typically takes ~2 hours to execute)
	- Results from SST_DEFAULT:
		- Solution trajectory and controls saved as SST_PATH.csv 
		- Can plot the results using OMPL_PlotPath.py using the command
		python OMPL_PlotPath.py SST_Path.csv
	- Results from benchmark:
		- Saved as <timestamp>.log
		- Connvert to .db file using ompl_benchmark_statistics.py using the command
		python ompl_benchmark_statistics.py <timestamp.log> -d myDatabase.db
		- Can then use http://plannerarena.org/ to plot data from the database file
