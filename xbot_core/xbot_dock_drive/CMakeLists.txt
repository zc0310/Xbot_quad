##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(xbot_dock_drive)

##############################################################################
# Find Packages
##############################################################################

find_package(catkin
    REQUIRED
    COMPONENTS
        ecl_build
        ecl_threads
        ecl_geometry
        ecl_linear_algebra
)

##############################################################################
# Exports
##############################################################################

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES xbot_dock_drive
   CATKIN_DEPENDS ecl_threads ecl_geometry ecl_linear_algebra
)

##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
include_directories(include ${catkin_INCLUDE_DIRS})

##############################################################################
# Sources
##############################################################################

add_library(xbot_dock_drive src/dock_drive.cpp src/dock_drive_states.cpp src/dock_drive_debug.cpp)
target_link_libraries(xbot_dock_drive ${catkin_LIBRARIES})

##############################################################################
# Installs
##############################################################################

install(TARGETS xbot_dock_drive
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
