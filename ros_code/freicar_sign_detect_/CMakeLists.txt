cmake_minimum_required(VERSION 3.1)
project(freicar_sign_detect_)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra")

# C++14 used for make_unique. if not available, use C++11 & 
# reimplement make_unique in a header file or use default syntax

set(CMAKE_CXX_STANDARD 14)
find_package(Threads REQUIRED)
find_package(Threads REQUIRED)

find_program(LSB_RELEASE_EXEC lsb_release)
execute_process(COMMAND ${LSB_RELEASE_EXEC} -ds
    OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if (${LSB_RELEASE_ID_SHORT} STREQUAL "Ubuntu 20.04.1 LTS")
	message("Ubuntu 20.04 detected...")
        find_package(OpenCV REQUIRED)
else()
	message("Ubuntu < 20.04 detected....")
        find_package(OpenCV 3 REQUIRED)
endif()


#find_package (Eigen3 REQUIRED NO_MODULE)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  image_transport
  cv_bridge
  freicar_common
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nn_raiscar
  CATKIN_DEPENDS 
  image_transport
  sensor_msgs
  cv_bridge
  roscpp
  freicar_common
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN_INCLUDES}
)


add_executable(${PROJECT_NAME}_node src/freicar_sign_detect_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CMAKE_THREAD_LIBS_INIT}
   pthread
   Eigen3::Eigen
)
