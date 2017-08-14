add_custom_target(statistics
    COMMAND jupyter notebook "${CMAKE_CURRENT_SOURCE_DIR}/analysis.ipynb")
