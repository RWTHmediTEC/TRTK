# Last changed on 2016-07-14

cmake_minimum_required(VERSION 2.8)


#####################################################################
# Project name
#####################################################################

project(TRTK_EXAMPLES)


#####################################################################
# Setup compiler flags
#####################################################################

# OpenMP

include(FindOpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Object Level Parallelism (parallel builds)

if(MSVC)
    add_definitions(/MP)
endif()


#####################################################################
# Find dependencies and add them later to the linker (see below)
#####################################################################

# TRTK

include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_BINARY_DIR}/bin)
set(TRTK_LIBLIST optimized ${TARGET_NAME} debug ${TARGET_NAME}${CMAKE_DEBUG_POSTFIX})


#####################################################################
# Examples
#####################################################################

add_executable(example_pivot_calibration_1 pivot_calibration_1.cpp)
target_link_libraries(example_pivot_calibration_1 ${TRTK_LIBLIST})

add_executable(example_pivot_calibration_2 pivot_calibration_2.cpp)
target_link_libraries(example_pivot_calibration_2 ${TRTK_LIBLIST})
