cmake_minimum_required(VERSION 3.18)

project(AndorNetwork)

link_directories("lib/")
include_directories("include/")

set(CMAKE_CXX_STANDARD 98)
set(CMAKE_VERBOSE_MAKEFILE ON)

set(ICE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Andor.ice")

list(APPEND ICE_GENERATED_FILES
    "Andor.cpp"
    "Andor.h"
    )

find_package(Ice 3.6.0 
    REQUIRED COMPONENTS
    Ice
    IceUtil
    )

foreach(SLICE_DIR ${Ice_SLICE_DIRS})
    list(APPEND SLICE_INCLUDE_ARGS "-I${SLICE_DIR}")
endforeach()

add_custom_command(
    OUTPUT ${ICE_GENERATED_FILES}
    COMMAND ${Ice_SLICE2CPP_EXECUTABLE}
    ARGS "--underscore" "--checksum" ${SLICE_INCLUDE_ARGS} ${ICE_FILE}
    MAIN_DEPENDENCY ${ICE_FILE}
    COMMENT "Generating Ice files"
    VERBATIM
    )

add_executable(AndorNetwork 
    "src/main.cpp"
    )

target_include_directories(AndorNetwork PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")

target_link_libraries(AndorNetwork
    PRIVATE
    "atmcd64m.lib"
    Ice::Ice
    Ice::IceUtil
    )

target_sources(AndorNetwork
    PRIVATE
    ${ICE_GENERATED_FILES}
    )

