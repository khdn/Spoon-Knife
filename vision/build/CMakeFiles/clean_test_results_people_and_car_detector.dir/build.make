# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/z/kamaz_vision/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/z/kamaz_vision/build

# Utility rule file for clean_test_results_people_and_car_detector.

# Include the progress variables for this target.
include CMakeFiles/clean_test_results_people_and_car_detector.dir/progress.make

CMakeFiles/clean_test_results_people_and_car_detector:
	/usr/bin/python /opt/ros/indigo/share/catkin/cmake/test/remove_test_results.py /home/z/kamaz_vision/build/test_results/people_and_car_detector

clean_test_results_people_and_car_detector: CMakeFiles/clean_test_results_people_and_car_detector
clean_test_results_people_and_car_detector: CMakeFiles/clean_test_results_people_and_car_detector.dir/build.make
.PHONY : clean_test_results_people_and_car_detector

# Rule to build all files generated by this target.
CMakeFiles/clean_test_results_people_and_car_detector.dir/build: clean_test_results_people_and_car_detector
.PHONY : CMakeFiles/clean_test_results_people_and_car_detector.dir/build

CMakeFiles/clean_test_results_people_and_car_detector.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_people_and_car_detector.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clean_test_results_people_and_car_detector.dir/clean

CMakeFiles/clean_test_results_people_and_car_detector.dir/depend:
	cd /home/z/kamaz_vision/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/z/kamaz_vision/src /home/z/kamaz_vision/src /home/z/kamaz_vision/build /home/z/kamaz_vision/build /home/z/kamaz_vision/build/CMakeFiles/clean_test_results_people_and_car_detector.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clean_test_results_people_and_car_detector.dir/depend

