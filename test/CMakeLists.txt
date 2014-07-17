# Last changed on 2014-07-04

cmake_minimum_required(VERSION 2.6)

project(TRTK_UNIT_TESTS)

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

# Setup the Visual Studio project properly

set(VSXPROJ_USER ${CMAKE_CURRENT_BINARY_DIR}/unit_tests.vcxproj.user)
if(NOT EXISTS ${VSXPROJ_USER})
	string(REGEX REPLACE "/" "\\\\" WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
	configure_file(${CMAKE_CURRENT_SOURCE_DIR}/misc/unit_tests.vcxproj.user.in ${VSXPROJ_USER} @ONLY)
endif()

#####################################################################
# Find Dependencies
#####################################################################

# Boost

find_package(Boost)
if(Boost_FOUND)
    include_directories (${Boost_INCLUDE_DIRS})
endif()

# TRTK

include_directories(${CMAKE_SOURCE_DIR}/include)
link_directories(${CMAKE_BINARY_DIR}/bin)
set(TRTK_LIBLIST optimized ${TARGET_NAME} debug ${TARGET_NAME}${CMAKE_DEBUG_POSTFIX})

#####################################################################

set(SOURCE_FILES
    unit_test.cpp
    unit_test_Circle.cpp
    unit_test_Clock.cpp
    unit_test_Coordinate.cpp
    unit_test_Diffusion.cpp
    unit_test_ErrorObj.cpp
    unit_test_EstimateAffineTransformation2D.cpp
    unit_test_EstimateAffineTransformation3D.cpp
    unit_test_EstimateAffineTransformationFromPlaneTo3D.cpp
    unit_test_EstimateProjectiveTransformation2D.cpp
    unit_test_EstimateProjectiveTransformation3D.cpp
    unit_test_EstimateRigidTransformation2D.cpp
    unit_test_EstimateRigidTransformation3D.cpp
    unit_test_EstimateSimilarityTransformation2D.cpp
    unit_test_EstimateSimilarityTransformation3D.cpp
    unit_test_FitCircle.cpp
    unit_test_FitCircle3D.cpp
    unit_test_FitCircleInOrigin.cpp
    unit_test_FitLine.cpp
    unit_test_FitLine3D.cpp
    unit_test_FitPlane.cpp
    unit_test_FitSphere.cpp
    unit_test_GenericPolynomial.cpp
    unit_test_Icp.cpp
    unit_test_Iterator.cpp
    unit_test_Optimization.cpp
    unit_test_PivotCalibration.cpp
    unit_test_Range.cpp
    unit_test_RegionGrowing2D.cpp
    unit_test_RegionGrowing3D.cpp
    unit_test_Signals.cpp
    unit_test_SurfaceExtraction3D.cpp
    unit_test_Timestamp.cpp
    unit_test_Tools.cpp
    unit_test_Transform2D.cpp
    unit_test_Transform3D.cpp
    unit_test_TrivariateQuadraticPolynomial.cpp
)

if(OPENCV_FOUND)
    list(APPEND SOURCE_FILES unit_test_CorrelationCircle.cpp)
    add_definitions(-DOPENCV_FOUND)
endif(OPENCV_FOUND)

add_executable(unit_tests ${SOURCE_FILES})

# target_link_libraries(unit_tests ${Boost_LIBS} ${FLANN_LIBLIST} ${OPENCV_LIBLIST} ${TRTK_LIBLIST})
target_link_libraries(unit_tests ${LIBS} ${TRTK_LIBLIST})
