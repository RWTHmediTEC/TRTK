# Last changed on 2014-07-11

cmake_minimum_required(VERSION 2.8)

# set(CMAKE_DEBUG_POSTFIX d)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)


#####################################################################
# Application name
#####################################################################

project(TRTK-Test)
set(TARGET_NAME ${PROJECT_NAME})


#####################################################################
# Find dependencies
#####################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(Eigen3 REQUIRED)
message(${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE(Flann REQUIRED)
include_directories(${FLANN_INCLUDE_DIR})
set(LIBS ${LIBS} ${FLANN_LIBRARY})

FIND_PACKAGE(OpenCV)
set(LIBS ${LIBS} ${OpenCV_LIBS})


############################################
# OpenMP
############################################

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


############################################
# Project
############################################

# add_definitions(-DNOMINMAX)

set(HEADERS
    src/main.hpp
)

set(SRC
    src/main.cpp
)

add_executable(
    ${TARGET_NAME}
    ${HEADERS}
    ${SRC}
)

target_link_libraries(${TARGET_NAME} ${LIBS})
