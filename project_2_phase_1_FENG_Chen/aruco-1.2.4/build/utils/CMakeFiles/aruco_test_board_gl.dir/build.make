# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.21

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
CMAKE_COMMAND = /home/albert/.local/lib/python2.7/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/albert/.local/lib/python2.7/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/albert/ELEC5660/proj2phase1/aruco-1.2.4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_test_board_gl.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include utils/CMakeFiles/aruco_test_board_gl.dir/compiler_depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_test_board_gl.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_test_board_gl.dir/flags.make

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o: utils/CMakeFiles/aruco_test_board_gl.dir/flags.make
utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o: ../utils/aruco_test_board_gl.cpp
utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o: utils/CMakeFiles/aruco_test_board_gl.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o"
	cd /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o -MF CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o.d -o CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o -c /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/utils/aruco_test_board_gl.cpp

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.i"
	cd /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/utils/aruco_test_board_gl.cpp > CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.i

utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.s"
	cd /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/utils && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/utils/aruco_test_board_gl.cpp -o CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.s

# Object files for target aruco_test_board_gl
aruco_test_board_gl_OBJECTS = \
"CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o"

# External object files for target aruco_test_board_gl
aruco_test_board_gl_EXTERNAL_OBJECTS =

utils/aruco_test_board_gl: utils/CMakeFiles/aruco_test_board_gl.dir/aruco_test_board_gl.cpp.o
utils/aruco_test_board_gl: utils/CMakeFiles/aruco_test_board_gl.dir/build.make
utils/aruco_test_board_gl: src/libaruco.so.1.2.4
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_dnn.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_highgui.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_ml.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_objdetect.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_shape.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_stitching.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_superres.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_videostab.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_viz.so.3.4.8
utils/aruco_test_board_gl: /usr/lib/x86_64-linux-gnu/libGL.so
utils/aruco_test_board_gl: /usr/lib/x86_64-linux-gnu/libGLU.so
utils/aruco_test_board_gl: /usr/lib/x86_64-linux-gnu/libglut.so
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_calib3d.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_features2d.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_flann.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_photo.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_video.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_videoio.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_imgcodecs.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_imgproc.so.3.4.8
utils/aruco_test_board_gl: /home/albert/opencv-3.4.8/build/lib/libopencv_core.so.3.4.8
utils/aruco_test_board_gl: utils/CMakeFiles/aruco_test_board_gl.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable aruco_test_board_gl"
	cd /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_test_board_gl.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_test_board_gl.dir/build: utils/aruco_test_board_gl
.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/build

utils/CMakeFiles/aruco_test_board_gl.dir/clean:
	cd /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_test_board_gl.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/clean

utils/CMakeFiles/aruco_test_board_gl.dir/depend:
	cd /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/albert/ELEC5660/proj2phase1/aruco-1.2.4 /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/utils /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/utils /home/albert/ELEC5660/proj2phase1/aruco-1.2.4/build/utils/CMakeFiles/aruco_test_board_gl.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_test_board_gl.dir/depend

