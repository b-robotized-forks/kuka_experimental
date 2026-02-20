cmake_minimum_required (VERSION 3.5 FATAL_ERROR)

file (GLOB SOURCES src/*.cpp)
file (GLOB SOURCES_CORE src/core/*.cpp)
file (GLOB SOURCES_EVENTS src/events/*.cpp)
file (GLOB SOURCES_TYPES src/types/*.cpp)
file (GLOB SOURCES_TOOLS src/tools/*.cpp)

add_library(eki_common SHARED
  ${SOURCES}
  ${SOURCES_CORE}
  ${SOURCES_EVENTS}
  ${SOURCES_TYPES}
  ${SOURCES_TOOLS}
)

target_include_directories(eki_common PUBLIC
  include
)

find_package(Threads REQUIRED)

target_link_libraries(eki_common PUBLIC
  ${CMAKE_THREAD_LIBS_INIT}
)
