aux_source_directory(. SRC_FILES)

message(STATUS ${SRC_FILES})

add_library(extractor SHARED ${SRC_FILES})
