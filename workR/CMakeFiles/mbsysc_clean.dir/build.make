# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/louis/Documents/Robotics/MobileRobotsProject/workR

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/louis/Documents/Robotics/MobileRobotsProject/workR

# Utility rule file for mbsysc_clean.

# Include the progress variables for this target.
include CMakeFiles/mbsysc_clean.dir/progress.make

CMakeFiles/mbsysc_clean:
	/usr/bin/cmake -E chdir /home/louis/Documents/Robotics/MobileRobotsProject/mbsysCopy/cmake_aux/scripts ./mbsysc_build /home/louis/Documents/Robotics/MobileRobotsProject/mbsysCopy clean

mbsysc_clean: CMakeFiles/mbsysc_clean
mbsysc_clean: CMakeFiles/mbsysc_clean.dir/build.make

.PHONY : mbsysc_clean

# Rule to build all files generated by this target.
CMakeFiles/mbsysc_clean.dir/build: mbsysc_clean

.PHONY : CMakeFiles/mbsysc_clean.dir/build

CMakeFiles/mbsysc_clean.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/mbsysc_clean.dir/cmake_clean.cmake
.PHONY : CMakeFiles/mbsysc_clean.dir/clean

CMakeFiles/mbsysc_clean.dir/depend:
	cd /home/louis/Documents/Robotics/MobileRobotsProject/workR && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/louis/Documents/Robotics/MobileRobotsProject/workR /home/louis/Documents/Robotics/MobileRobotsProject/workR /home/louis/Documents/Robotics/MobileRobotsProject/workR /home/louis/Documents/Robotics/MobileRobotsProject/workR /home/louis/Documents/Robotics/MobileRobotsProject/workR/CMakeFiles/mbsysc_clean.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/mbsysc_clean.dir/depend

