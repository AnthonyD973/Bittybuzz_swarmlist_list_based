cmake_minimum_required(VERSION 3.0)

project(swarmlist_list_based)

#
# Set flags
#
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -std=c99   -O2 -march=native -flto")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -O2 -march=native -flto")


#
# Find packages and include them
#

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)

# Find Lua
find_package(Lua52 REQUIRED)

# Set include directories
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${LUA_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/controllers)

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})


#
# Descend into subdirectories
#
add_subdirectory(controllers)
add_subdirectory(loops)
add_subdirectory(experimenters)
