# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/will/humanoid_ws/src/yamlutils

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/will/humanoid_ws/build/yamlutils

# Include any dependencies generated for this target.
include CMakeFiles/test_yamlutils.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/test_yamlutils.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_yamlutils.dir/flags.make

CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.o: CMakeFiles/test_yamlutils.dir/flags.make
CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.o: /home/will/humanoid_ws/src/yamlutils/tests/gtest_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/will/humanoid_ws/build/yamlutils/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.o -c /home/will/humanoid_ws/src/yamlutils/tests/gtest_main.cpp

CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/will/humanoid_ws/src/yamlutils/tests/gtest_main.cpp > CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.i

CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/will/humanoid_ws/src/yamlutils/tests/gtest_main.cpp -o CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.s

CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.o: CMakeFiles/test_yamlutils.dir/flags.make
CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.o: /home/will/humanoid_ws/src/yamlutils/tests/test_yaml_tools.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/will/humanoid_ws/build/yamlutils/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.o -c /home/will/humanoid_ws/src/yamlutils/tests/test_yaml_tools.cpp

CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/will/humanoid_ws/src/yamlutils/tests/test_yaml_tools.cpp > CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.i

CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/will/humanoid_ws/src/yamlutils/tests/test_yaml_tools.cpp -o CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.s

CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.o: CMakeFiles/test_yamlutils.dir/flags.make
CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.o: /home/will/humanoid_ws/src/yamlutils/tests/test_yaml_eigen.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/will/humanoid_ws/build/yamlutils/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.o -c /home/will/humanoid_ws/src/yamlutils/tests/test_yaml_eigen.cpp

CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/will/humanoid_ws/src/yamlutils/tests/test_yaml_eigen.cpp > CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.i

CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/will/humanoid_ws/src/yamlutils/tests/test_yaml_eigen.cpp -o CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.s

# Object files for target test_yamlutils
test_yamlutils_OBJECTS = \
"CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.o" \
"CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.o" \
"CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.o"

# External object files for target test_yamlutils
test_yamlutils_EXTERNAL_OBJECTS =

test_yamlutils: CMakeFiles/test_yamlutils.dir/tests/gtest_main.cpp.o
test_yamlutils: CMakeFiles/test_yamlutils.dir/tests/test_yaml_tools.cpp.o
test_yamlutils: CMakeFiles/test_yamlutils.dir/tests/test_yaml_eigen.cpp.o
test_yamlutils: CMakeFiles/test_yamlutils.dir/build.make
test_yamlutils: /usr/local/lib/libgtest.a
test_yamlutils: /usr/lib/x86_64-linux-gnu/libyaml-cpp.so.0.6.2
test_yamlutils: CMakeFiles/test_yamlutils.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/will/humanoid_ws/build/yamlutils/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable test_yamlutils"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_yamlutils.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_yamlutils.dir/build: test_yamlutils

.PHONY : CMakeFiles/test_yamlutils.dir/build

CMakeFiles/test_yamlutils.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_yamlutils.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_yamlutils.dir/clean

CMakeFiles/test_yamlutils.dir/depend:
	cd /home/will/humanoid_ws/build/yamlutils && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/will/humanoid_ws/src/yamlutils /home/will/humanoid_ws/src/yamlutils /home/will/humanoid_ws/build/yamlutils /home/will/humanoid_ws/build/yamlutils /home/will/humanoid_ws/build/yamlutils/CMakeFiles/test_yamlutils.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_yamlutils.dir/depend

