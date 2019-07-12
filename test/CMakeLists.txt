# Last changed on 2019-06-12

cmake_minimum_required(VERSION 3.1)

project(TRTK_UNIT_TESTS)


#####################################################################
# Compiler specific settings
#####################################################################

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


# FLANN

if(FLANN_FOUND)
    include_directories(${FLANN_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${FLANN_LIBRARIES})
    add_definitions(-DFLANN_FOUND)
endif()


#####################################################################
# Unit Tests
#####################################################################

set(SOURCE_FILES
    unit_test.cpp
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
    unit_test_Iterator.cpp
    unit_test_Optimization.cpp
    unit_test_PinholeCameraModel.cpp
    unit_test_PivotCalibration.cpp
    unit_test_Range.cpp
    unit_test_RegionGrowing2D.cpp
    unit_test_RegionGrowing3D.cpp
    unit_test_Signals.cpp
    unit_test_SurfaceExtraction3D.cpp
    unit_test_Timestamp.cpp
    unit_test_Tools.cpp
    unit_test_TrivariateQuadraticPolynomial.cpp
)

if(FLANN_FOUND)
    list(APPEND SOURCE_FILES
        unit_test_Icp.hpp
        unit_test_Transform2D.hpp
        unit_test_Transform3D.hpp
    )
endif()

if(FFTW_FOUND3)
    list(APPEND SOURCE_FILES
	unit_test_CorrelationSphere.cpp
    )
endif()

add_executable(unit_tests ${SOURCE_FILES})

target_link_libraries(unit_tests ${LIBS} ${TRTK_LIBLIST} ${FLANN_LIBRARIES} ${FFTW3_LIBLIST})
set_target_properties(unit_tests PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/test/")


#####################################################################
# Pinhole Camera Analysis
#####################################################################

add_executable(pinhole_camera_model_analysis pinhole_camera_model_analysis.cpp)
target_link_libraries(pinhole_camera_model_analysis ${LIBS})
