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
CMAKE_SOURCE_DIR = /home/rel/git/CRANE_v2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rel/git/CRANE_v2

# Include any dependencies generated for this target.
include src/CMakeFiles/sample.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/sample.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/sample.dir/flags.make

src/CMakeFiles/sample.dir/sample.cpp.o: src/CMakeFiles/sample.dir/flags.make
src/CMakeFiles/sample.dir/sample.cpp.o: src/sample.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rel/git/CRANE_v2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/sample.dir/sample.cpp.o"
	cd /home/rel/git/CRANE_v2/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sample.dir/sample.cpp.o -c /home/rel/git/CRANE_v2/src/sample.cpp

src/CMakeFiles/sample.dir/sample.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sample.dir/sample.cpp.i"
	cd /home/rel/git/CRANE_v2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rel/git/CRANE_v2/src/sample.cpp > CMakeFiles/sample.dir/sample.cpp.i

src/CMakeFiles/sample.dir/sample.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sample.dir/sample.cpp.s"
	cd /home/rel/git/CRANE_v2/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rel/git/CRANE_v2/src/sample.cpp -o CMakeFiles/sample.dir/sample.cpp.s

src/CMakeFiles/sample.dir/sample.cpp.o.requires:

.PHONY : src/CMakeFiles/sample.dir/sample.cpp.o.requires

src/CMakeFiles/sample.dir/sample.cpp.o.provides: src/CMakeFiles/sample.dir/sample.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/sample.dir/build.make src/CMakeFiles/sample.dir/sample.cpp.o.provides.build
.PHONY : src/CMakeFiles/sample.dir/sample.cpp.o.provides

src/CMakeFiles/sample.dir/sample.cpp.o.provides.build: src/CMakeFiles/sample.dir/sample.cpp.o


# Object files for target sample
sample_OBJECTS = \
"CMakeFiles/sample.dir/sample.cpp.o"

# External object files for target sample
sample_EXTERNAL_OBJECTS =

exe/sample: src/CMakeFiles/sample.dir/sample.cpp.o
exe/sample: src/CMakeFiles/sample.dir/build.make
exe/sample: src/CMakeFiles/sample.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rel/git/CRANE_v2/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../exe/sample"
	cd /home/rel/git/CRANE_v2/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sample.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/sample.dir/build: exe/sample

.PHONY : src/CMakeFiles/sample.dir/build

src/CMakeFiles/sample.dir/requires: src/CMakeFiles/sample.dir/sample.cpp.o.requires

.PHONY : src/CMakeFiles/sample.dir/requires

src/CMakeFiles/sample.dir/clean:
	cd /home/rel/git/CRANE_v2/src && $(CMAKE_COMMAND) -P CMakeFiles/sample.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/sample.dir/clean

src/CMakeFiles/sample.dir/depend:
	cd /home/rel/git/CRANE_v2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rel/git/CRANE_v2 /home/rel/git/CRANE_v2/src /home/rel/git/CRANE_v2 /home/rel/git/CRANE_v2/src /home/rel/git/CRANE_v2/src/CMakeFiles/sample.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/sample.dir/depend

