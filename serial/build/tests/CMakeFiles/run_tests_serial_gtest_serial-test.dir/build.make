# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build

# Utility rule file for run_tests_serial_gtest_serial-test.

# Include any custom commands dependencies for this target.
include tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/compiler_depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/progress.make

tests/CMakeFiles/run_tests_serial_gtest_serial-test:
	cd /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build/tests && ../catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build/test_results/serial/gtest-serial-test.xml "/home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build/devel/lib/serial/serial-test --gtest_output=xml:/home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build/test_results/serial/gtest-serial-test.xml"

run_tests_serial_gtest_serial-test: tests/CMakeFiles/run_tests_serial_gtest_serial-test
run_tests_serial_gtest_serial-test: tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/build.make
.PHONY : run_tests_serial_gtest_serial-test

# Rule to build all files generated by this target.
tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/build: run_tests_serial_gtest_serial-test
.PHONY : tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/build

tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/clean:
	cd /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build/tests && $(CMAKE_COMMAND) -P CMakeFiles/run_tests_serial_gtest_serial-test.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/clean

tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/depend:
	cd /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/tests /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build/tests /home/kaizhen/TTC_yolov7/stm2ros_wk/src/serial/build/tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tests/CMakeFiles/run_tests_serial_gtest_serial-test.dir/depend

