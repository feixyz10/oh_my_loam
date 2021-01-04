message(STATUS "oh_my_loam dir: " ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(base)
add_subdirectory(extractor)
# add_subdirectory(visualizer)
# add_subdirectory(odometry)
# add_subdirectory(mapper)
# add_subdirectory(solver)

# aux_source_directory(. SRC_FILES)

# add_library(oh_my_loam SHARED ${SRC_FILES})

