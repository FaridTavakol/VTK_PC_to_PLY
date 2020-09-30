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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/test_build

# Include any dependencies generated for this target.
include CMakeFiles/NeuroKinematics.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/NeuroKinematics.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/NeuroKinematics.dir/flags.make

CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.o: CMakeFiles/NeuroKinematics.dir/flags.make
CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.o: ../NeuroKinematics/NeuroKinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/test_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.o -c /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/NeuroKinematics/NeuroKinematics.cpp

CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/NeuroKinematics/NeuroKinematics.cpp > CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.i

CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/NeuroKinematics/NeuroKinematics.cpp -o CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.s

CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.o: CMakeFiles/NeuroKinematics.dir/flags.make
CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.o: ../include/ForwardKinematics/ForwardKinematics.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/test_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.o -c /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/include/ForwardKinematics/ForwardKinematics.cpp

CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/include/ForwardKinematics/ForwardKinematics.cpp > CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.i

CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/include/ForwardKinematics/ForwardKinematics.cpp -o CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.s

# Object files for target NeuroKinematics
NeuroKinematics_OBJECTS = \
"CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.o" \
"CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.o"

# External object files for target NeuroKinematics
NeuroKinematics_EXTERNAL_OBJECTS =

libNeuroKinematics.a: CMakeFiles/NeuroKinematics.dir/NeuroKinematics/NeuroKinematics.cpp.o
libNeuroKinematics.a: CMakeFiles/NeuroKinematics.dir/include/ForwardKinematics/ForwardKinematics.cpp.o
libNeuroKinematics.a: CMakeFiles/NeuroKinematics.dir/build.make
libNeuroKinematics.a: CMakeFiles/NeuroKinematics.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/test_build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libNeuroKinematics.a"
	$(CMAKE_COMMAND) -P CMakeFiles/NeuroKinematics.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/NeuroKinematics.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/NeuroKinematics.dir/build: libNeuroKinematics.a

.PHONY : CMakeFiles/NeuroKinematics.dir/build

CMakeFiles/NeuroKinematics.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/NeuroKinematics.dir/cmake_clean.cmake
.PHONY : CMakeFiles/NeuroKinematics.dir/clean

CMakeFiles/NeuroKinematics.dir/depend:
	cd /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/test_build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/test_build /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/test_build /home/aimlab/Documents/NRI_Project/VTK/test_ply_writer/test_build/CMakeFiles/NeuroKinematics.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/NeuroKinematics.dir/depend
