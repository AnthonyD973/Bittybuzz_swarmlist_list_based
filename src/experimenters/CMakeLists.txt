add_custom_target(run
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/run_exp" "run" "${CMAKE_BINARY_DIR}")
add_custom_target(submit
    COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/run_exp" "submit" "${CMAKE_BINARY_DIR}")
