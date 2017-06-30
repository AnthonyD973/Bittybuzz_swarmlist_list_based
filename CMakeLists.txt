cmake_minimum_required(VERSION 3.0)

project(kilobot_list_based)

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


file(GLOB HEADERS "*.h")
file(GLOB C_SOURCES "*.c")
file(GLOB CXX_SOURCES "*.cpp")
set(FILES ${HEADERS} ${C_SOURCES} ${CXX_SOURCES})

set(TARGET kilobot_list_based)
add_executable(${TARGET} ${FILES})
target_link_libraries(${TARGET}
    argos3core_simulator
    argos3plugin_simulator_kilobot
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_kilolib
    rt
)
set_target_properties(${TARGET} PROPERTIES LINKER_LANGUAGE CXX)
