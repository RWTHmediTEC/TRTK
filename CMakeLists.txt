# Last changed on 2014-07-14

cmake_minimum_required(VERSION 2.8)

set(CMKE_DEBUG_POSTFIX d)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)


#####################################################################
# Library name
#####################################################################

project(TRTK)


#####################################################################
# Version declaration
# Target file will be named with version suffix
#####################################################################

set(MAJOR_VERSION  0)
set(MINOR_VERSION 15)
set(PATCH_VERSION  1)

set(FULL_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})

set(TARGET_NAME ${PROJECT_NAME}-${FULL_VERSION})

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/misc/configure.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/misc/configure.cmake @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/misc/Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/src/doc/Doxyfile)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/misc/install.bat.in ${CMAKE_CURRENT_BINARY_DIR}/misc/install.bat @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/misc/Version.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/include/TRTK/Version.hpp)


#####################################################################
# Options
# Set default options for this build
#####################################################################

option(BUILD_DOC "Build and install the API documentation" OFF)
option(BUILD_TEST "Build the unit tests" OFF)


#####################################################################
# Find dependencies
#####################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

FIND_PACKAGE(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

FIND_PACKAGE(Flann REQUIRED)
include_directories(${FLANN_INCLUDE_DIR})
set(LIBS ${LIBS} ${FLANN_LIBRARY})

FIND_PACKAGE(OpenCV)

if(OPENCV_FOUND)

    list(APPEND PUBLIC_HEADERS
        include/TRTK/CorrelationCircle.hpp
    )

    list(APPEND SRC
        src/CorrelationCircle.cpp
    )
    
    set(LIBS ${LIBS} ${OpenCV_LIBS})
    
endif(OPENCV_FOUND)

# Project

include_directories(${CMAKE_PROJECT_DIR}include/${PROJECT_NAME})

set(PUBLIC_HEADERS
    include/TRTK/Circle.hpp
    include/TRTK/ClientServer.hpp
    include/TRTK/Clock.hpp
    include/TRTK/Coordinate.hpp
    include/TRTK/Definitions.hpp
    include/TRTK/Diffusion.hpp
    include/TRTK/ErrorObj.hpp
    include/TRTK/EstimateAffineTransformation2D.hpp
    include/TRTK/EstimateAffineTransformation3D.hpp
    include/TRTK/EstimateAffineTransformationFromPlaneTo3D.hpp
    include/TRTK/EstimateProjectiveTransformation2D.hpp
    include/TRTK/EstimateProjectiveTransformation3D.hpp
    include/TRTK/EstimateRigidTransformation2D.hpp
    include/TRTK/EstimateRigidTransformation3D.hpp
    include/TRTK/EstimateSimilarityTransformation2D.hpp
    include/TRTK/EstimateSimilarityTransformation3D.hpp
    include/TRTK/EstimateTransformation.hpp
    include/TRTK/EstimateTransformation2D.hpp
    include/TRTK/EstimateTransformation3D.hpp
    include/TRTK/Fit.hpp
    include/TRTK/Fit2D.hpp
    include/TRTK/Fit3D.hpp
    include/TRTK/FitCircle.hpp
    include/TRTK/FitCircle3D.hpp
    include/TRTK/FitCircleInOrigin.hpp
    include/TRTK/FitLine.hpp
    include/TRTK/FitLine3D.hpp
    include/TRTK/FitPlane.hpp
    include/TRTK/FitSphere.hpp
    include/TRTK/GenericPolynomial.hpp
    include/TRTK/Icp.hpp
    include/TRTK/Interpolation2D.hpp
    include/TRTK/Interpolation3D.hpp
    include/TRTK/Iterator.hpp
    include/TRTK/Optimization.hpp
    include/TRTK/PivotCalibration.hpp
    include/TRTK/Polynomial.hpp
    include/TRTK/Range.hpp
    include/TRTK/Ransac.hpp
    include/TRTK/RansacGenericFittingModel.hpp
    include/TRTK/RegionGrowing2D.hpp
    include/TRTK/RegionGrowing3D.hpp
    include/TRTK/Signals.hpp
    include/TRTK/SurfaceExtraction3D.hpp
    include/TRTK/Timestamp.hpp
    include/TRTK/Tools.hpp
    include/TRTK/Transform2D.hpp
    include/TRTK/Transform3D.hpp
    include/TRTK/TrivariateQuadraticPolynomial.hpp
    include/TRTK/Version.hpp
)

set(PRIVATE_HEADERS
    ""
)

set(SRC
    src/Circle.cpp
    src/ClientServer.cpp
    src/Clock.cpp
    src/Coordinate.cpp
    src/ErrorObj.cpp
    src/Signals.cpp
    src/Timestamp.cpp
    src/Tools.cpp
)

add_library(${TARGET_NAME} STATIC ${SRC} ${PUBLIC_HEADERS} ${PRIVATE_HEADERS})


#####################################################################
# Add dependencies to the linker
#####################################################################

target_link_libraries(${TARGET_NAME} ${LIBS})


#vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
##vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
### NO CHANGES EVER NECESSARY BELOW THIS POINT!!
##vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv


#####################################################################
# Installation
#####################################################################

install(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(
    DIRECTORY ${CMAKE_PROJECT_DIR}include/TRTK
    DESTINATION include
)


#####################################################################
# Unit tests
#####################################################################

if(BUILD_TEST)
    add_subdirectory(test)
endif()


#####################################################################
# Doxygen API documentation
#####################################################################

if(BUILD_DOC)
    find_package(Doxygen REQUIRED)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doc/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile @ONLY)
        add_custom_target(doc ALL
                        ${DOXYGEN_EXECUTABLE} 
                        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
                        COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
                DESTINATION $ENV{MEDITEC_LIBS}/${PROJECT_NAME}/tags/${MAJOR_VERSION}.${MINOR_VERSION}
        )
    endif(DOXYGEN_FOUND)
endif()
