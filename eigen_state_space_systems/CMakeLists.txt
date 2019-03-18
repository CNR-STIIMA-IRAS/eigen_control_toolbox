cmake_minimum_required(VERSION 2.8.3)
project(eigen_state_space_systems)

#add_compile_options(-std=c++11 -funroll-loops  -Ofast)
add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  roscpp cmake_modules eigen_matrix_utils
)
find_package(Eigen3 REQUIRED COMPONENTS Core Dense)

catkin_package(
  INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp eigen_matrix_utils
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/eigen_state_space_systems.cpp
#   src/${PROJECT_NAME}/eigen_controllers.cpp
#   src/${PROJECT_NAME}/eigen_iir_filters.cpp
#   src/${PROJECT_NAME}/eigen_fir_filters.cpp
#   src/${PROJECT_NAME}/eigen_sg_filter.cpp
# )
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(${PROJECT_NAME}
#                       ${catkin_LIBRARIES}
#                       ${Eigen_LIBRARIES}
# )


add_executable(test_state_space_matrix src/test_state_space_matrix.cpp)
add_dependencies(test_state_space_matrix ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_state_space_matrix
#                       ${PROJECT_NAME}
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
