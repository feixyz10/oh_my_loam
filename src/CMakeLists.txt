include_directories(
  ${PROJECT_SOURCE_DIR}/common
)

aux_source_directory(. SRC_FILES)

add_library(OhMyLoam SHARED ${SRC_FILES})
