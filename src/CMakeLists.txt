add_subdirectory(extractor)
add_subdirectory(visualizer)

aux_source_directory(. SRC_FILES)

add_library(oh_my_loam SHARED ${SRC_FILES})

