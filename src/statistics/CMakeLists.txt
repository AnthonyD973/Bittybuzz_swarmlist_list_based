add_custom_target(statistics EXCLUDE_FROM_ALL
    COMMAND jupyter notebook "${CMAKE_CURRENT_SOURCE_DIR}/analysis.ipynb")