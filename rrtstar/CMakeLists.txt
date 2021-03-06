cmake_minimum_required(VERSION 2.8.3)
project(rrtstar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  add_definitions(-DNDEBUG)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  rospy
  std_msgs
  message_generation
  mpl_common
	collision
  models
)

find_package(GTest REQUIRED)
find_package(benchmark REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${benchmark_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
FIND_PACKAGE(PythonInterp REQUIRED)
FIND_PACKAGE(PythonLibs REQUIRED)
if(${PYTHON_VERSION_MAJOR} EQUAL 3)
    FIND_PACKAGE(Boost COMPONENTS python3 system core)
elseif()
    FIND_PACKAGE(Boost COMPONENTS python system core)
endif()

find_package(PkgConfig)
PKG_CHECK_MODULES(lapacke REQUIRED lapacke)
PKG_CHECK_MODULES(nanoflann REQUIRED nanoflann)

# for now, just dont use cuda, TODO: fix
#find_package(CUDA REQUIRED VERSION 10)
#if(CUDA_FOUND)
#  IF(CMAKE_BUILD_TYPE MATCHES Debug)
#    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++14; -lineinfo; -O0; -G; -g; --expt-relaxed-constexpr" )
#  ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
#    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++14; -O3; --expt-relaxed-constexpr" )
#    add_definitions(-DNDEBUG)
#  ELSE()
#    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++14; -lineinfo; -O1; --expt-relaxed-constexpr" )
#  ENDIF()
##  set(CUDA_NVCC_FLAGS
##    ${CUDA_NVCC_FLAGS};
##    -gencode arch=compute_30,code=sm_30
##    -gencode arch=compute_35,code=sm_35;
##    -gencode arch=compute_50,code=[sm_50,compute_50]
##    -gencode arch=compute_52,code=[sm_52,compute_52]
##    -gencode arch=compute_61,code=sm_61
##    -gencode arch=compute_62,code=sm_62
##    )
#  message("CUDA_NVCC_FLAGS" ${CUDA_NVCC_FLAGS})
#  add_definitions(-DGPU)
#endif()


add_message_files(
  FILES
  State.msg
  StampedState.msg
  )

add_service_files(
  FILES
  TrajectoryService.srv
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

generate_messages(
  DEPENDENCIES
  std_msgs  # and/or other packages containing depended messages
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrtstar_integrator2d
  CATKIN_DEPENDS roscpp visualization_msgs message_runtime
  DEPENDS GTest
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 include/rrtstar
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
)

#add_library(integrator2d_model include/integrator2d.hpp include/integrator.hpp include/fixedtimelqr.hpp include/rootfinder.hpp include/statespace.hpp include/statespacesolver.hpp)
#target_link_libraries(integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(integrator2d_model PROPERTIES LINKER_LANGUAGE CXX)

#add_library(rrtstar include/integrator2drrt.hpp include/rrtstar.hpp include/pointcloud.hpp include/kdtree.hpp include/rrtvisual.hpp include/states.hpp include/environment.hpp include/collision.hpp include/random.hpp src/rrtstar/intergrator2drrt.cpp)
add_library(rrtstar_integrator2d src/rrtstar/integrator2drrt.cpp)
target_link_libraries(rrtstar_integrator2d integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
#set_target_properties(rrtstar PROPERTIES LINKER_LANGUAGE CXX)

add_executable(${PROJECT_NAME}_integrator2d_node src/integrator2d_node.cpp)
add_executable(${PROJECT_NAME}_integrator2d_dynamic_test src/integrator2d_dynamic_test.cpp)
add_executable(${PROJECT_NAME}_integrator2d_test src/integrator2d_test.cpp)
add_executable(${PROJECT_NAME}_integrator2d_dataset src/integrator2d_dataset_node.cpp)
add_executable(${PROJECT_NAME}_geometric3d src/geometric3d_demo.cpp)
add_executable(${PROJECT_NAME}_geometric2d src/geometric2d_demo.cpp)
add_executable(${PROJECT_NAME}_integrator2d_trajectory src/integrator2d_trajectory.cpp)
#add_executable(${PROJECT_NAME}_unittest test/unittest.cpp)
add_executable(${PROJECT_NAME}_benchmark test/benchmark.cpp)
add_executable(${PROJECT_NAME}_custom_benchmark test/custom_benchmark.cpp)

add_dependencies(${PROJECT_NAME}_integrator2d_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

if(CUDA_FOUND)
  cuda_add_library(integrator2d_cuda_model include/integrator2d.cuhpp include/fixedtimelqr.cuhpp include/util.cuh)
  target_link_libraries(integrator2d_cuda_model integrator2d_model cuda cudart)
  cuda_add_library(cuda_rrtstar include/integrator2drrt.cuhpp include/environment.cuhpp include/util.cuh)
  target_link_libraries(cuda_rrtstar integrator2d_cuda_model rrtstar_integrator2d cuda cudart)
  cuda_add_executable(${PROJECT_NAME}_batch_integrator2d_dynamic_test src/integrator2d_batch_dynamic_test.cu)
  cuda_add_executable(${PROJECT_NAME}_batch_integrator2d_test src/integrator2d_batch_test.cu)
endif()

target_link_libraries(${PROJECT_NAME}_benchmark ${BENCHMARK_LIBRARIES} ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} benchmark pthread rrtstar_integrator2d integrator2d_model)
target_link_libraries(${PROJECT_NAME}_custom_benchmark ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} pthread rrtstar_integrator2d integrator2d_model)
#target_link_libraries(${PROJECT_NAME}_unittest rrtstar_integrator2d integrator2d_model ${GTEST_LIBRARIES} ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_integrator2d_dynamic_test rrtstar_integrator2d integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_integrator2d_trajectory integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_integrator2d_dataset rrtstar_integrator2d integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_integrator2d_test rrtstar_integrator2d integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_integrator2d_node rrtstar_integrator2d integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_geometric3d rrtstar_integrator2d integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_geometric2d rrtstar_integrator2d integrator2d_model ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
if(CUDA_FOUND)
  target_link_libraries(${PROJECT_NAME}_batch_integrator2d_dynamic_test ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} cuda_rrtstar integrator2d_cuda_model)
  target_link_libraries(${PROJECT_NAME}_batch_integrator2d_test ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} cuda_rrtstar integrator2d_cuda_model)
endif()

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/rrt_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_add_gtest(${PROJECT_NAME}-test test/unittest.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test rrtstar_integrator2d integrator2d_model ${GTEST_LIBRARIES} ${lapacke_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
endif()
