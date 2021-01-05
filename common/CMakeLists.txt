file(GLOB SRC_FILES **/*.cc)

add_library(common SHARED ${SRC_FILES})
