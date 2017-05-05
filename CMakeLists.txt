cmake_minimum_required(VERSION 2.8.3)
project(mrasl_mav_traj)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

option(USE_OOQP     "Compile with OOQP"     ON)
option(USE_GUROBI   "Compile with GUROBI"   OFF)
option(USE_IPOPT    "Compile with Ipot"     OFF)
option(USE_QPOASES  "Compile with qpOASES"  OFF)
option(USE_NLOPT    "Compile with NLopt"    OFF)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  trajectory_msgs
  geometry_msgs
)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp trajectory_msgs geometry_msgs
  DEPENDS
)

###########
## Build ##
###########
set(MAV_TRAJ_INCLUDE_DIRS include)
if(USE_OOQP)
  add_definitions(-DUSE_OOQP)
  find_package(catkin REQUIRED COMPONENTS ooqp_eigen_interface)
  set(OOQP_LIBS ooqpbase ooqpdense ooqpgensparse ooqpmehrotra ooqpbound
          ooqpgendense ooqpgondzio ooqpsparse blas ma27 gfortran)
endif()
if(USE_GUROBI)
  add_definitions(-DUSE_OOQP)
  find_package(GUROBI REQUIRED)
  include_directories(${GUROBI_INCLUDE_DIRS} ${EIGEN_GUROBI_INCLUDE_DIR})
endif()
if(USE_IPOPT)
  add_definitions(-DUSE_IPOPT)
  find_package(IPOPT REQUIRED)
  include_directories(${IPOPT_INCLUDE_DIRS})
endif()
if(USE_QPOASES)
  add_definitions(-DUSE_QPOASES)
  find_package(qpOASES REQUIRED)
  include_directories(${QPOASES_INCLUDE_DIR})
endif()
if(USE_NLOPT)
  add_definitions(-DUSE_NLOPT)
  find_package(Nlopt REQUIRED)
  include_directories(${NLOPT_INCLUDE_DIRS})
endif()
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

## Declare a C++ library
add_library(alglib
  src/alglib/alglibinternal.cpp
  src/alglib/alglibmisc.cpp
  src/alglib/ap.cpp
  src/alglib/linalg.cpp
  src/alglib/optimization.cpp
  src/alglib/solvers.cpp
)
target_include_directories(alglib PUBLIC include/alglib)
target_compile_definitions(alglib PUBLIC -DAE_CPU=AE_INTEL)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/TrajectoryMath.cpp
  src/${PROJECT_NAME}/TrajectoryConstraint.cpp
  src/${PROJECT_NAME}/TrajectorySegment.cpp
        src/mrasl_mav_traj/TrajectoryGenerator.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
if(USE_OOQP)
  target_link_libraries(${PROJECT_NAME} ${OOQP_LIBS})
endif()



## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/mrasl_mav_traj_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} #${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_main.cpp
  test/test_trajectory_math.cpp
  test/test_trajectory_generation.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
