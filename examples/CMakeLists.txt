# Last changed on 2019-05-20

cmake_minimum_required(VERSION 3.1)


#####################################################################
# Project name
#####################################################################

project(TRTK_EXAMPLES)


#####################################################################
# Setup compiler flags
#####################################################################

# Object Level Parallelism (parallel builds)

if(MSVC)
    add_definitions(/MP)
endif()


#####################################################################
# Find dependencies and add them later to the linker (see below)
#####################################################################

# TRTK

include_directories(${CMAKE_SOURCE_DIR}/include
                    ${FLANN_INCLUDE_DIRS}
)

link_directories(${CMAKE_BINARY_DIR}/bin
                 ${FLANN_LIBRARIES}
)

# set(TRTK_LIBLIST optimized ${TARGET_NAME} debug ${TARGET_NAME}${CMAKE_DEBUG_POSTFIX})
set(LIBS ${TARGET_NAME})

if(FLANN_FOUND)
    include_directories(${FLANN_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${FLANN_LIBRARIES})
    add_definitions(-DFLANN_FOUND)
endif()


#####################################################################
# Examples
#####################################################################

add_executable(example_pinhole_camera_model_1 pinhole_camera_model_1.cpp)
target_link_libraries(example_pinhole_camera_model_1 ${LIBS})

add_executable(analyze_pinhole_camera_model pinhole_camera_model_2.cpp)
target_link_libraries(analyze_pinhole_camera_model ${LIBS})

add_executable(example_pivot_calibration_1 pivot_calibration_1.cpp)
target_link_libraries(example_pivot_calibration_1 ${LIBS})

add_executable(example_pivot_calibration_2 pivot_calibration_2.cpp)
target_link_libraries(example_pivot_calibration_2 ${LIBS})

add_executable(example_pivot_calibration_ransac pivot_calibration_ransac.cpp)
target_link_libraries(example_pivot_calibration_ransac ${LIBS})
