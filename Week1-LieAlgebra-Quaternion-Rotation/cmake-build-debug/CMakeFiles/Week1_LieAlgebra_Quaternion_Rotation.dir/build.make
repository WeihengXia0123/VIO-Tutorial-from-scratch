# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /home/weiheng/Opt/clion-2020.3/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/weiheng/Opt/clion-2020.3/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/flags.make

CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.o: CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/flags.make
CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.o -c /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/main.cpp

CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/main.cpp > CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.i

CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/main.cpp -o CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.s

# Object files for target Week1_LieAlgebra_Quaternion_Rotation
Week1_LieAlgebra_Quaternion_Rotation_OBJECTS = \
"CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.o"

# External object files for target Week1_LieAlgebra_Quaternion_Rotation
Week1_LieAlgebra_Quaternion_Rotation_EXTERNAL_OBJECTS =

Week1_LieAlgebra_Quaternion_Rotation: CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/main.cpp.o
Week1_LieAlgebra_Quaternion_Rotation: CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/build.make
Week1_LieAlgebra_Quaternion_Rotation: CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Week1_LieAlgebra_Quaternion_Rotation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/build: Week1_LieAlgebra_Quaternion_Rotation

.PHONY : CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/build

CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/clean

CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/depend:
	cd /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/cmake-build-debug /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/cmake-build-debug /home/weiheng/Code/EURECOM/VIO-Tutorial-from-scratch/Week1-LieAlgebra-Quaternion-Rotation/cmake-build-debug/CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Week1_LieAlgebra_Quaternion_Rotation.dir/depend

