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
CMAKE_SOURCE_DIR = /home/mika/realsense

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mika/realsense

# Include any dependencies generated for this target.
include CMakeFiles/gui.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gui.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gui.dir/flags.make

CMakeFiles/gui.dir/BGR_sample.cpp.o: CMakeFiles/gui.dir/flags.make
CMakeFiles/gui.dir/BGR_sample.cpp.o: BGR_sample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mika/realsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gui.dir/BGR_sample.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gui.dir/BGR_sample.cpp.o -c /home/mika/realsense/BGR_sample.cpp

CMakeFiles/gui.dir/BGR_sample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gui.dir/BGR_sample.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mika/realsense/BGR_sample.cpp > CMakeFiles/gui.dir/BGR_sample.cpp.i

CMakeFiles/gui.dir/BGR_sample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gui.dir/BGR_sample.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mika/realsense/BGR_sample.cpp -o CMakeFiles/gui.dir/BGR_sample.cpp.s

CMakeFiles/gui.dir/BGR_sample.cpp.o.requires:

.PHONY : CMakeFiles/gui.dir/BGR_sample.cpp.o.requires

CMakeFiles/gui.dir/BGR_sample.cpp.o.provides: CMakeFiles/gui.dir/BGR_sample.cpp.o.requires
	$(MAKE) -f CMakeFiles/gui.dir/build.make CMakeFiles/gui.dir/BGR_sample.cpp.o.provides.build
.PHONY : CMakeFiles/gui.dir/BGR_sample.cpp.o.provides

CMakeFiles/gui.dir/BGR_sample.cpp.o.provides.build: CMakeFiles/gui.dir/BGR_sample.cpp.o


# Object files for target gui
gui_OBJECTS = \
"CMakeFiles/gui.dir/BGR_sample.cpp.o"

# External object files for target gui
gui_EXTERNAL_OBJECTS =

gui: CMakeFiles/gui.dir/BGR_sample.cpp.o
gui: CMakeFiles/gui.dir/build.make
gui: /usr/local/lib/libopencv_ml.so.3.2.0
gui: /usr/local/lib/libopencv_objdetect.so.3.2.0
gui: /usr/local/lib/libopencv_shape.so.3.2.0
gui: /usr/local/lib/libopencv_stitching.so.3.2.0
gui: /usr/local/lib/libopencv_superres.so.3.2.0
gui: /usr/local/lib/libopencv_videostab.so.3.2.0
gui: /usr/local/lib/librealsense.so
gui: /usr/local/lib/libopencv_calib3d.so.3.2.0
gui: /usr/local/lib/libopencv_features2d.so.3.2.0
gui: /usr/local/lib/libopencv_flann.so.3.2.0
gui: /usr/local/lib/libopencv_highgui.so.3.2.0
gui: /usr/local/lib/libopencv_photo.so.3.2.0
gui: /usr/local/lib/libopencv_video.so.3.2.0
gui: /usr/local/lib/libopencv_videoio.so.3.2.0
gui: /usr/local/lib/libopencv_imgcodecs.so.3.2.0
gui: /usr/local/lib/libopencv_imgproc.so.3.2.0
gui: /usr/local/lib/libopencv_core.so.3.2.0
gui: CMakeFiles/gui.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mika/realsense/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gui"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gui.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gui.dir/build: gui

.PHONY : CMakeFiles/gui.dir/build

CMakeFiles/gui.dir/requires: CMakeFiles/gui.dir/BGR_sample.cpp.o.requires

.PHONY : CMakeFiles/gui.dir/requires

CMakeFiles/gui.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gui.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gui.dir/clean

CMakeFiles/gui.dir/depend:
	cd /home/mika/realsense && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mika/realsense /home/mika/realsense /home/mika/realsense /home/mika/realsense /home/mika/realsense/CMakeFiles/gui.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gui.dir/depend

