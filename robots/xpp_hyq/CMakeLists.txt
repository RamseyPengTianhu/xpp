cmake_minimum_required(VERSION 2.8.3)
project(xpp_hyq)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xpp_vis
)
# find_package(towr 1.3.2 REQUIRED)
#   add_executable(xpp_hyq src/IK_a1.cc) # Your custom variables, costs and constraints added to TOWR
#   target_link_libraries(main PUBLIC towr::towr) # adds include directories and libraries

###################################
## catkin specific configuration ##
###################################
catkin_package()


###########
## Build ##
###########
## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /home/tianhu/TO/src/xpp/robots  # Adjust this line
)



# Declare a C++ library
add_library(${PROJECT_NAME}
  src/hyqleg_inverse_kinematics.cc
  src/A1leg_inverse_kinematics.cc
  src/inverse_kinematics_hyq1.cc
  src/inverse_kinematics_hyq2.cc
  src/inverse_kinematics_hyq4.cc
  src/inverse_kinematics_a1.cc
)

## URDF visualizers for all HyQ variants
add_executable(urdf_visualizer_hyq1 src/exe/urdf_visualizer_hyq1.cc)
target_link_libraries(urdf_visualizer_hyq1
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_hyq2 src/exe/urdf_visualizer_hyq2.cc)
target_link_libraries(urdf_visualizer_hyq2
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_hyq4 src/exe/urdf_visualizer_hyq4.cc)
target_link_libraries(urdf_visualizer_hyq4
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(urdf_visualizer_a1 src/exe/urdf_visualizer_a1.cc)
target_link_libraries(urdf_visualizer_a1
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# add_executable(IK_a1 src/IK_a1.cc)
# set_target_properties(IK_a1 PROPERTIES OUTPUT_NAME "IK_a1")
# target_link_libraries(IK_a1
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )


#############
## Install ##
#############
# Mark library for installation
install(
  TARGETS ${PROJECT_NAME} urdf_visualizer_hyq1 urdf_visualizer_hyq2 urdf_visualizer_hyq4 urdf_visualizer_a1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY launch rviz meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
