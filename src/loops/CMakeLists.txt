add_library(exp_loop_func SHARED
    ExpLoopFunc.cpp
    ExpLoopFunc.h
    RobotPlacer.cpp
    RobotPlacer.h
    ExpState.cpp
    ExpState.h)

target_link_libraries(exp_loop_func
    argos3core_simulator
    argos3plugin_simulator_footbot
    argos3plugin_simulator_genericrobot
)
