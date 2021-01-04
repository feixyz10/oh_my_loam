file(GLOB SRC_FILES **/*.cc)

message(STATUS "common dir: " ${CMAKE_CURRENT_SOURCE_DIR})

# include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_library(common SHARED ${SRC_FILES})
