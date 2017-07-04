cmake_minimum_required(VERSION 3.0)

project(kilobot_list_based)

#
# Set flags
#
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -Wall -O4")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O4 -std=c++11")


#
# Find packages and include them
#

# Find the ARGoS package, make sure to save the ARGoS prefix
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(ARGOS_PREFIX ${ARGOS_PREFIX} CACHE INTERNAL "")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
set(KILOLIB_INCLUDE_DIR "${ARGOS_INCLUDE_DIRS}/argos3/plugins/robots/kilobot/control_interface")

# Find Lua
find_package(Lua52 REQUIRED)

# Set include directories
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${KILOLIB_INCLUDE_DIR} ${LUA_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})


#
# Function definitions
#

# Function to add a target for ARGoS for kilobots
function (add_argos_kilobot_executable _TARGET)
    add_executable(${_TARGET} ${ARGN})
    target_link_libraries(${_TARGET}
        argos3core_simulator
        argos3plugin_simulator_kilobot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_kilolib
        rt
    )
    set_target_properties(${_TARGET} PROPERTIES LINKER_LANGUAGE CXX)
endfunction()


#
# Descend into subdirectories
#
add_subdirectory(behaviors)
add_subdirectory(loops)
add_subdirectory(experimenters)