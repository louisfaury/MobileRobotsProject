=================== MOBILE ROBOTS PROJECT ================
==================     Gr4 - README       ================
==========================================================

Our project was hosted on GitHub. Versioning was done using the git command line tool. Check out the code's history at https://github.com/louisfaury/MobileRobotsProject.

/!\ We wrote most of our project with c++11 standards. To comply with the project's main CMakeFile that would be common for all groups, we finally decided to switch back to c++98 standards. Hence, some of our code had to be transformed back and this has lead to some sub-optimalities in our review code. 
Namely, we used many constexpr variables and functions, hence speeding up many operations carried out at run time. The concerned functions are now back to normal (now special keyworks), and instead of transforming constexpr variables to preprocessors, we decided, to be coherent with the rest of the code, that they should remain only const (hence losing the compile time optimization). 

We created the following files. There are comments expressing their purposes in our code, however we will quickly remind here their roles. 
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



################################
########   ALGORITHMS  #########
################################

Concerning the algorithms we chose to develop :

- Localization is performed by extended kalman filter taking into account odometry measurements as well as triangulation. Particular care was taken to avoid updating kalman with corrupted triangulation measurements (./localization).

-Path planning was made by A* algorithm. Map is divised into cells at launch time (SearchGraph initialization). Path planning is performed as soon as strategy module requires it. A* star weights takes distance travelled as well as turns into account (not admissible so not distance optimal but time optimal). Chosen heuristical score is distance to target. (./path)

-Path following is an open loop with regular checks. In case of anomaly, A* replanning is performed. LinePath and CurvePath classes are written to always respect actuators constraints (continuity of actuator commands)(./regulation).

- Strategy is a finite state machine. It is adapted to opponents avoidance (by A* star replanning if necessary), in wall avoidance and resilience to possible shocks. Particular care was taken to avoid deadlocks. Picked target is chosen according to a score function taking the form of a simple neural network taking into account : distance of the target to opponent, distance of the target to the robot, number of targets currently carried by the robot, target value, distance to the other closest target. Training was made thanks to a genetic algorithm trained against various strategies like picking best value or closest target (./strategy). 
 



