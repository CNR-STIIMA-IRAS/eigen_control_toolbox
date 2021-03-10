cmake_minimum_required(VERSION 3.1)
project(eigen_state_space_systems)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS roscpp eigen_matrix_utils rosparam_utilities)
find_package(Eigen3 REQUIRED COMPONENTS Core Dense)

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp eigen_matrix_utils rosparam_utilities
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(test_state_space_matrix src/test_state_space_matrix.cpp)
add_dependencies(test_state_space_matrix ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_state_space_matrix
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)

add_executable(test_filters src/test_filters.cpp)
add_dependencies(test_filters ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_filters
#                       ${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)


add_executable(        test_controllers src/test_pid.cpp)
add_dependencies(      test_controllers ${catkin_EXPORTED_TARGETS})
target_link_libraries( test_controllers
#                       ${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)
# install(TARGETS  ${PROJECT_NAME} test_state_space_matrix test_filters
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# 
# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)


# catkin_add_gtest(${PROJECT_NAME}-test test/test_eigen_state_space_systems.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
