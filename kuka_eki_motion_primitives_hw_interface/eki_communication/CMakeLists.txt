cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

add_subdirectory(eki_common)

file(GLOB SOURCES src/*.cpp)
file(GLOB SOURCES_CORE src/core/*.cpp)
file(GLOB SOURCES_XML src/xml/*.cpp)
file(GLOB SOURCES_TINYXML2 third_party/tinyxml2/*.cpp)

add_library(eki_communication SHARED
  ${SOURCES}
  ${SOURCES_CORE}
  ${SOURCES_XML}
  ${SOURCES_TINYXML2}
)

target_include_directories(eki_communication PUBLIC
  include
  third_party
)

target_link_libraries(eki_communication PUBLIC
  eki_common
)

install(TARGETS eki_communication
  EXPORT eki_communicationTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
