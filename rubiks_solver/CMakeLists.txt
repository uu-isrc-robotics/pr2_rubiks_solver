cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_gensrv()

rosbuild_add_library(solvelib src/solvelib/combinat.cpp src/solvelib/Timer.cpp src/solvelib/cubepars.cpp  src/solvelib/movetabl.cpp  src/solvelib/solver.cpp src/solvelib/cube.cpp      src/solvelib/facecube.cpp  src/solvelib/kocicube.cpp  src/solvelib/pruningt.cpp  src/solvelib/vector.cpp)

rosbuild_add_executable(solve_cmd src/kcube.cpp)
target_link_libraries(solve_cmd solvelib)

rosbuild_add_executable(solve_node src/solver_node.cpp)
target_link_libraries(solve_node solvelib)

