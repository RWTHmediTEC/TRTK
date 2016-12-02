# Last changed on 2016-07-27

cmake_minimum_required(VERSION 2.8)

set(CMAKE_DEBUG_POSTFIX d)
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
set(MINOR_VERSION 16)
set(PATCH_VERSION  1)
set(FULL_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})
set(TARGET_NAME ${PROJECT_NAME}-${FULL_VERSION})

file(TO_CMAKE_PATH "$ENV{MEDITEC_LIBS}" MEDITEC_LIBS_PATH)
 
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/configure.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/configure.cmake @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/src/doc/Doxyfile)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/install.bat.in ${CMAKE_CURRENT_BINARY_DIR}/cmake/install.bat @ONLY)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Version.hpp.in ${CMAKE_CURRENT_SOURCE_DIR}/include/TRTK/Version.hpp)


#####################################################################
# Options
# Set default options for this build
#####################################################################

option(BUILD_DOC "Build and install the API documentation" OFF)
option(BUILD_TEST "Build the unit tests" OFF)
option(BUILD_EXAMPLES "Build examples" ON)


#####################################################################
# Find dependencies and add them later to the linker (see below)
#####################################################################

# If you change the version numbers of the included libraries, do not
# forget to adapt the configure.cmake.in file in the cmake folder.

# Eigen

include(${MEDITEC_LIBS_PATH}/Eigen/3.2.8/configure.cmake)

# OpenCV

include(${MEDITEC_LIBS_PATH}/opencv2.4/2.4.1/configure.cmake)

# FLANN

include(${MEDITEC_LIBS_PATH}/FLANN/1.7.1/configure.cmake)

# Project

include_directories(${CMAKE_PROJECT_DIR}include)

set(PUBLIC_HEADERS
    include/TRTK/Circle.hpp
    include/TRTK/ClientServer.hpp
    include/TRTK/Clock.hpp
    include/TRTK/Coordinate.hpp
    include/TRTK/CorrelationCircle.hpp
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
 include/TRTK/Line3D.hpp
    include/TRTK/Optimization.hpp
    include/TRTK/PivotCalibration.hpp
 include/TRTK/Plane.hpp
    include/TRTK/Polynomial.hpp
    include/TRTK/Range.hpp
    include/TRTK/Ransac.hpp
    include/TRTK/RansacGenericFittingModel.hpp
    include/TRTK/RansacPivotCalibrationModel.hpp
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
    src/CorrelationCircle.cpp
    src/ErrorObj.cpp
	src/Line3D.cpp
	src/Plane.cpp
    src/Signals.cpp
    src/Timestamp.cpp
    src/Tools.cpp
)

add_library(${TARGET_NAME} STATIC ${SRC} ${PUBLIC_HEADERS} ${PRIVATE_HEADERS})


#####################################################################
# Add dependencies to the linker
#####################################################################

target_link_libraries(
    ${TARGET_NAME}
    ${OPENCV_LIBLIST}
    ${FLANN_LIBLIST}
)


#vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
##vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
### NO CHANGES EVER NECESSARY BELOW THIS POINT!!
##vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

string(REGEX REPLACE "\\\\" "/" INSTALL_DIR ${MEDITEC_LIBS_PATH}/${PROJECT_NAME}/${FULL_VERSION})

install(
    TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION ${INSTALL_DIR}/bin
    ARCHIVE DESTINATION ${INSTALL_DIR}/bin
    LIBRARY DESTINATION ${INSTALL_DIR}/bin
)

install(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/configure.cmake ${CMAKE_CURRENT_BINARY_DIR}/cmake/install.bat
    DESTINATION ${INSTALL_DIR}
)

install(
    DIRECTORY ${CMAKE_PROJECT_DIR}include
    DESTINATION ${INSTALL_DIR}
)

if (WIN32)
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/bin/${TARGET_NAME}${CMAKE_DEBUG_POSTFIX}.pdb
        DESTINATION ${MEDITEC_LIBS_PATH}/bin
        OPTIONAL
    )
    install(
        FILES ${CMAKE_CURRENT_BINARY_DIR}/bin/${TARGET_NAME}${CMAKE_DEBUG_POSTFIX}.pdb
        DESTINATION ${INSTALL_DIR}/bin
        OPTIONAL
    )
endif()


#####################################################################
# Unit tests
#####################################################################

if(BUILD_TEST)
    # enable_testing()
    add_subdirectory(test)
endif()


#####################################################################
# Examples
#####################################################################

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
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
                DESTINATION ${MEDITEC_LIBS_PATH}/${PROJECT_NAME}/tags/${MAJOR_VERSION}.${MINOR_VERSION}
        )
    endif(DOXYGEN_FOUND)
endif()
