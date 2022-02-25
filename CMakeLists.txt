cmake_minimum_required(VERSION 3.16.0)
project("vicon_datastream" VERSION 1.0)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

###############################################################################
# CMake build rules
###############################################################################
set(IS_TOP_LEVEL TRUE)
if(PROJECT_SOURCE_DIR)
    set(IS_TOP_LEVEL FALSE)
endif()


include_directories(external/PX4-Matrix)

###############################################################################
# Load installed packages
###############################################################################

find_package(MAVSDK REQUIRED)

###############################################################################
# Add subdirectories
###############################################################################

add_subdirectory(external)
add_subdirectory(mocap_px4_apps)


