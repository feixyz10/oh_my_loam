add_subdirectory(base)
add_subdirectory(extractor)
add_subdirectory(visualizer)
add_subdirectory(odometer)
add_subdirectory(mapper)
add_subdirectory(solver)

aux_source_directory(. SRC_FILES)

add_library(oh_my_loam SHARED ${SRC_FILES})

