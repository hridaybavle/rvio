cmake_minimum_required(VERSION 2.8.3)
project(rvio)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    cv_bridge
    camera_models
    image_transport)

find_package(OpenCV REQUIRED)
find_package(G2O REQUIRED)

FILE(GLOB_RECURSE rvio_source_files
    ${CMAKE_CURRENT_SOURCE_DIR}/src/featureTracker/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    )

FILE(GLOB_RECURSE rvio_header_files
    ${CMAKE_CURRENT_SOURCE_DIR}/src/featureTracker/*.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h
    )

find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  SYSTEM PUBLIC 
  ${G2O_INCLUDE_DIR}	
)

catkin_package(INCLUDE_DIRS src
  LIBRARIES vins_lib
  CATKIN_DEPENDS roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf
  cv_bridge
  camera_models
  image_transport)

add_library(rvio_lib rvio_source_files)
target_link_libraries(rvio_lib 
		      ${catkin_LIBRARIES} 
		      ${OpenCV_LIBS}
		      ${G2O_CORE_LIBRARY}
                      ${G2O_STUFF_LIBRARY}
		      ${G2O_SOLVER_CHOLMOD})

add_executable(${PROJECT_NAME}_node ${CMAKE_CURRENT_SOURCE_DIR}/src/rvio_node.cpp)
target_link_libraries(${PROJECT_NAME}_node rvio_lib)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
