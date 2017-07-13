add_library(exp_loop_func SHARED
    ExpLoopFunc.cpp
    ExpLoopFunc.h
    KilobotProcess.h
    KilobotProcess.cpp)

target_link_libraries(exp_loop_func
    argos3core_simulator
    argos3plugin_simulator_kilobot
    rt
)

install(TARGETS exp_loop_func DESTINATION ${ARGOS_PREFIX}/lib/argos3)