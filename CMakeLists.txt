cmake_minimum_required(VERSION 3.18)

project(AndorNetwork)

link_directories("lib/")
include_directories("include/")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_VERBOSE_MAKEFILE ON)

find_program(NUGET nuget)

if(NOT NUGET)
    message(FATAL_ERROR "Could not find nuget command line tool.")
else()
    execute_process(COMMAND
        ${NUGET} install zeroc.ice.v140 -Version 3.7.8 -OutputDirectory ${CMAKE_SOURCE_DIR}/packages
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
    set(Ice_HOME ${CMAKE_SOURCE_DIR}/packages/zeroc.ice.v140.3.7.8/)
endif()

if(NOT EXISTS "${Ice_HOME}")
    message(FATAL_ERROR "Could not find installed zeroc.ice.v140 package at ${Ice_HOME}!")
endif()

set(ICE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/Andor.ice")

list(APPEND ICE_GENERATED_FILES
    "Andor.cpp"
    "Andor.h"
    )

set(Ice_DEBUG ON)

find_package(Ice 3.7.0 
    REQUIRED COMPONENTS
    Ice
    Ice++11
    IceDiscovery
    IceLocatorDiscovery
    IceSSL
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

#add_definitions(ICE_CPP11_MAPPING)

target_link_libraries(AndorNetwork
    PRIVATE
    "atmcd64m.lib"
    Ice::Ice
    Ice::Ice++11
    Ice::IceDiscovery
    Ice::IceLocatorDiscovery
    Ice::IceSSL
    )

target_sources(AndorNetwork
    PRIVATE
    ${ICE_GENERATED_FILES}
    )

