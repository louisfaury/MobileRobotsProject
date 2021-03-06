=================== MOBILE ROBOTS PROJECT ================
==================     Gr4 - README       ================
==========================================================

Our project was hosted on GitHub. Versioning was done using the git command line tool. Check out the code's history at https://github.com/louisfaury/MobileRobotsProject.


/!\ We wrote most of our project with c++11 standards. To comply with the project's main CMakeFile that would be common for all groups, we finally decided to switch back to c++98 standards. Hence, some of our code had to be transformed back and this has led to some sub-optimalities in our reviewed code. 
Namely, we used many constexpr variables and functions, hence speeding up many operations carried out at run time. The concerned functions are now back to normal (now special keyworks), and instead of transforming constexpr variables to preprocessors, we decided, to be coherent with the rest of the code, that they should remain only const (hence losing the compile time optimization). 

====================================================================

Concerning the algorithms we chose to develop :

-We implemented an Extended Kalman Filter for localization, based on odometry and landmark-based triangulation. Particular care was taken to avoid updating kalman with corrupted triangulation measurements (./localization).

-Path-planning is based on a A* algorithm running on a grid decomposed map. Map is divised into cells at launch time (SearchGraph initialization). Path planning is performed as soon as strategy module requires it. A* star weights takes distance travelled as well as turns into account (not admissible so not distance optimal but time optimal). Chosen heuristical score is distance to target. (./path)

-Path following is an open loop with regular checks. In case of anomaly, A* replanning is performed. LinePath and CurvePath classes are path abstractions that allow the robot to follow a trajectory while constantly respecting its speed and acceleration constraints.

- Strategy is based on a finite state machine. It is adapted to opponents avoidance (by A* star replanning if necessary), in wall avoidance and resilience to possible shocks. Particular care was taken to avoid deadlocks. Picked target is chosen according to a score function taking the form of a genetically-enhanced neural network for decision taking into account : distance of the target to opponent, distance of the target to the robot, number of targets currently carried by the robot, target value, distance to the other closest target (./strategy). 

======================================================================

We created the following files. They are comments expressing their purposes in our code, however we will quickly remind here their roles. 

- ./config: 
	config_file_gr4.h : contains many needed consts variables 
- ./path : 
	Cell_gr4.h, Cell_gr4.cpp : basic representation of a cell
	SearchCell_gr4.h, SearchCell_gr4.cpp : inherits from Cell, and countains attributes to allow search on a graph made of cells 
	Link_gr4.h, Link_gr4.cpp : representation of a link between two SearchCell (neighborhood definition)
	MapHandler_gr4.h, MapHandler_gr4.cpp : defines the map and contains static and dynamic (opponents) obstacles
	SearchGraph_gr4.h, SearchGraph_gr4.cpp : definition of a cell graph with best-width search functions 

- ./regulation : 
	Path_gr4.h, Path_gr4.cpp : basic representation of a path 
	LinePath_gr4.h, LinePath_gr4.cpp : inherits from path, concept of a straight line with regulation functions (acc. and speed constraints compliance)
	CurvePath_gr4.h, CurvePath_gr4.cpp : inherits from path, concept of a zero-radius curve  with regulation functions (acc. and speed constraints compliance)

- ./useful : 
	geometric_objects_gr4.h, geometric_objects_gr4.cpp : Concepts of geometrical shapes and functions to compute intersections. Mostly used for the map. 





 



