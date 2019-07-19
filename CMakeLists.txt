# Last changed on 2019-07-19

cmake_minimum_required(VERSION 3.10)

set(CMAKE_DEBUG_POSTFIX d)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)

cmake_policy(SET CMP0022 NEW)


#####################################################################
# Library name
#####################################################################

project(TRTK)


#####################################################################
# Version declaration
# Target file will be named with version suffix
#####################################################################

set(MAJOR_VERSION  1)
set(MINOR_VERSION  4)
set(PATCH_VERSION  0)

set(FULL_VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION})
set(TRTK_VERSION ${FULL_VERSION})

set(TARGET_NAME ${PROJECT_NAME})


#####################################################################
# Compilation Flags
#####################################################################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(UNIX AND CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    add_definitions(-fPIC)
endif()

if(MSVC)
    add_definitions(/MP) # Object Level Parallelism (parallel builds)
    add_definitions(/permissive-) # Standards-conforming compiler behavior
    add_definitions(/bigobj) # Avoid fatal error C1128 (number of sections exceeds object file format limit)
    add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE) # standard-compliant aligned storage in MSVC 15.8 and higher
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS)
endif()


#####################################################################
# Config files
#####################################################################

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
# Find dependencies
#####################################################################

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Flann QUIET)
if(FLANN_FOUND)
    include_directories(${FLANN_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${FLANN_LIBRARIES})
endif()

find_library(FFTW_LIBRARY NAMES fftw3 fftw fftw3-3)
set(FFTW_LIBRARIES "${FFTW_LIBRARY}")
if(FFTW_LIBRARY)
    set(FFTW_FOUND3 1)
    include_directories(${FFTW_LIBRARY}/include)
    link_directories(${FFTW_LIBRARY}/lib)
    set(LIBS ${LIBS} libfftw3-3.lib)
    add_definitions(-DFFTW3_FOUND)
endif()

find_path(CPPOPTLIB NAMES cppoptlib cppoptlib-1.0.0)
if(CPPOPTLIB)
    set(CPPOPTLIB_FOUND 1)
    include_directories(${CPPOPTLIB})
    add_definitions(-DCPPOPTLIB_FOUND)
endif()


#####################################################################
# Project files
#####################################################################

set(PUBLIC_HEADERS
    include/TRTK/CImg.h
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
    include/TRTK/Iterator.hpp
    include/TRTK/Optimization.hpp
    include/TRTK/PinholeCameraModel.hpp
    include/TRTK/PivotCalibration.hpp
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

if(FLANN_FOUND)
    list(APPEND PUBLIC_HEADERS
        include/TRTK/ICP.hpp
        include/TRTK/Interpolation2D.hpp
        include/TRTK/Interpolation3D.hpp
    )
endif()

if(FFTW_FOUND3)
    list(APPEND PUBLIC_HEADERS
        include/TRTK/CorrelationSphere.hpp
    )
endif()

set(PRIVATE_HEADERS
    ""
)

set(SRC
    src/ClientServer.cpp
    src/Clock.cpp
    src/Coordinate.cpp
    src/ErrorObj.cpp
    src/Signals.cpp
    src/Timestamp.cpp
    src/Tools.cpp
)

add_library(${TARGET_NAME} STATIC ${SRC} ${PUBLIC_HEADERS} ${PRIVATE_HEADERS} ${CV_HEADERS} ${CV_SRC})
target_include_directories(${TARGET_NAME} BEFORE PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" "$<INSTALL_INTERFACE:include>")
target_include_directories(${TARGET_NAME} PUBLIC "$<INSTALL_INTERFACE:${FLANN_INCLUDE_DIR};${EIGEN3_INCLUDE_DIR}>")


#####################################################################
# Add dependencies to the linker
#####################################################################

target_link_libraries(${TARGET_NAME} INTERFACE ${LIBS})


#####################################################################
# Examples
#####################################################################

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()


#####################################################################
# Unit tests
#####################################################################

if(BUILD_TEST)
    add_subdirectory(test)
endif()


#vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
##vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
### NO CHANGES EVER NECESSARY BELOW THIS POINT!!
##vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
set(INCLUDE_INSTALL_DESTINATION "include/${TARGET_NAME}")

include(GenerateExportHeader)
generate_export_header(${TARGET_NAME} EXPORT_FILE_NAME ${TARGET_NAME}_export.h)
set_property(TARGET ${TARGET_NAME} PROPERTY VERSION ${TRTK_VERSION})
set_property(TARGET ${TARGET_NAME} PROPERTY SOVERSION ${MAJOR_VERSION})
set_property(TARGET ${TARGET_NAME} PROPERTY
  INTERFACE_${TARGET_NAME}_MAJOR_VERSION ${MAJOR_VERSION})
set_property(TARGET ${TARGET_NAME} APPEND PROPERTY
  COMPATIBLE_INTERFACE_STRING ${TARGET_NAME}_MAJOR_VERSION
)


install(TARGETS ${TARGET_NAME} EXPORT ${TARGET_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(
  FILES
    ${PUBLIC_HEADERS}
    "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}_export.h"
  DESTINATION
    ${INCLUDE_INSTALL_DESTINATION}
  COMPONENT
    Devel
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}ConfigVersion.cmake
  VERSION ${TRTK_VERSION}
  COMPATIBILITY SameMajorVersion
)

export(EXPORT ${TARGET_NAME}Targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}/${TARGET_NAME}Targets.cmake"
  NAMESPACE TRTK::
)

set(INCLUDE_INSTALL_DIR include CACHE PATH "Install dir for include headers")
set(LIB_INSTALL_DIR lib CACHE PATH  "Install dir for libraries")
set(SYSCONFIG_INSTALL_DIR /usr/local/ CACHE  PATH  "Install dir for system configuration")
set(INSTALL_INCLUDE_DIR include/${PROJECT_NAME} CACHE PATH "Install dir for headers")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH "Install dir for CMake files")

include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/${TARGET_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}Config.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
  PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR
)


#####################################################################
# Installation
#####################################################################

install(EXPORT ${TARGET_NAME}Targets
  FILE
    ${TARGET_NAME}Targets.cmake
  NAMESPACE
    TRTK::
  DESTINATION
    ${INSTALL_CMAKE_DIR}
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}ConfigVersion.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_NAME}Config.cmake
  DESTINATION
    ${INSTALL_CMAKE_DIR}
  COMPONENT
    Devel
)


#####################################################################
# Doxygen API documentation
#####################################################################

if(BUILD_DOC)
    find_package(Doxygen REQUIRED)
    if(DOXYGEN_FOUND)
        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/doc/Doxyfile @ONLY)
        add_custom_target(doc ALL
                        ${DOXYGEN_EXECUTABLE}
                        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
                        COMMENT "Generating API documentation with Doxygen" VERBATIM
        )
        install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/doc
                DESTINATION $ENV{MEDITEC_LIBS}/${PROJECT_NAME}/tags/${MAJOR_VERSION}.${MINOR_VERSION}
        )
    endif()
endif()

###################### Add uninstall target ############################
if("${CMAKE_PROJECT_NAME}" STREQUAL "${PROJECT_NAME}")
    add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake)
endif()

