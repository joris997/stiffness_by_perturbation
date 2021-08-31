cmake_minimum_required(VERSION 3.2)
project(leg_stiffness_cpp)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

list(APPEND CMAKE_PREFIX_PATH "/home/none/opensim/master/RelWithDebInfo-install/lib/cmake")
#list(APPEND CMAKE_PREFIX_PATH "opt/opensim-moco-build/lib/cmake")
find_package(OpenSim REQUIRED)
find_package(Simbody REQUIRED)

add_library(common_options_target INTERFACE)
target_link_libraries(common_options_target INTERFACE
  SimTKcommon
  SimTKmath
  SimTKsimbody
  osimLepton
  osimCommon
  osimSimulation
  osimActuators
  osimAnalyses
  osimTools)

target_compile_options(common_options_target INTERFACE
  "$<$<CXX_COMPILER_ID:GNU>:-Wno-unknown-pragmas>"
  "$<$<CXX_COMPILER_ID:Clang>:-Wno-unknown-pragmas>")


add_executable(${PROJECT_NAME} "main.cpp")

target_link_libraries(${PROJECT_NAME} common_options_target)
set_target_properties(leg_stiffness_cpp PROPERTIES FOLDER "Examples")
