cmake_minimum_required (VERSION 3.1 FATAL_ERROR)

file (GLOB SOURCES src/*.cpp)
file (GLOB SOURCES_CORE src/core/*.cpp)
file (GLOB SOURCES_EVENTS src/events/*.cpp)
file (GLOB SOURCES_TYPES src/types/*.cpp)
file (GLOB SOURCES_TOOLS src/tools/*.cpp)

add_library(common SHARED
  ${SOURCES}
  ${SOURCES_CORE}
  ${SOURCES_EVENTS}
  ${SOURCES_TYPES}
  ${SOURCES_TOOLS}
)

target_include_directories(common PUBLIC
  include
)

find_package(Threads)

target_link_libraries(common PUBLIC
  ${CMAKE_THREAD_LIBS_INIT}
)
