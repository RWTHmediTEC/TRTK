# Last changed on 2013-06-06

TEMPLATE = app

CONFIG -= qt core
CONFIG += 3dnow console debug exceptions mmx rtti stl sse sse2 thread warn_on

TARGET = unit_test

DESTDIR = .

OBJECTS_DIR = ./build

INCLUDEPATH += ../include
INCLUDEPATH += "$(MEDITEC_LIBS)/Eigen/tags/3.0.5/include"

win32:INCLUDEPATH += "$(MEDITEC_LIBS)/FLANN/tags/1.7.1/include"
win32:INCLUDEPATH += "$(MEDITEC_LIBS)/OpenCV2.4/tags/2.4.1/include"
win32:INCLUDEPATH += "$(MEDITEC_LIBS)/OpenCV2.4/tags/2.4.1/include/opencv"

unix:INCLUDEPATH += "/usr/include"
unix:INCLUDEPATH += "/usr/include/opencv"

LIBS += -L"$(MEDITEC_LIBS)/bin/debug"

win32:LIBS += -L"$(MEDITEC_LIBS)/FLANN/tags/1.7.1/bin/debug"
win32:LIBS += -L"$(MEDITEC_LIBS)/OpenCV2.4/tags/2.4.1/bin/debug"
win32:LIBS += TRTKd.lib flann.lib opencv_core241d.lib opencv_highgui241d.lib opencv_imgproc241d.lib

unix:QMAKE_LIBDIR += "/usr/lib"
unix:LIBS += -lTRTK -lflann -lopencv_core -lopencv_highgui -lopencv_imgproc

HEADERS = unit_test.hpp

SOURCES = unit_test.cpp \
          unit_test_Circle.cpp \
          unit_test_Clock.cpp \
          unit_test_Coordinate.cpp \
          unit_test_CorrelationCircle.cpp \
          unit_test_Diffusion.cpp \
          unit_test_ErrorObj.cpp \
          unit_test_EstimateAffineTransformation2D.cpp \
          unit_test_EstimateAffineTransformation3D.cpp \
          unit_test_EstimateAffineTransformationFromPlaneTo3D.cpp \
          unit_test_EstimateProjectiveTransformation2D.cpp \
          unit_test_EstimateProjectiveTransformation3D.cpp \
          unit_test_EstimateRigidTransformation2D.cpp \
          unit_test_EstimateRigidTransformation3D.cpp \
          unit_test_EstimateSimilarityTransformation2D.cpp \
          unit_test_EstimateSimilarityTransformation3D.cpp \
          unit_test_FitCircle.cpp \
          unit_test_FitCircle3D.cpp \
          unit_test_FitCircleInOrigin.cpp \
          unit_test_FitLine.cpp \
          unit_test_FitLine3D.cpp \
          unit_test_FitPlane.cpp \
          unit_test_FitSphere.cpp \
          unit_test_GenericPolynomial.cpp \
          unit_test_Icp.cpp \
          unit_test_Iterator.cpp \
          unit_test_Optimization.cpp \
          unit_test_PivotCalibration.cpp \
          unit_test_Range.cpp \
          unit_test_RegionGrowing2D.cpp \
          unit_test_RegionGrowing3D.cpp \
          unit_test_Signals.cpp \
          unit_test_SurfaceExtraction3D.cpp \
          unit_test_Timestamp.cpp \
          unit_test_Tools.cpp \
          unit_test_Transform2D.cpp \
          unit_test_Transform3D.cpp \
          unit_test_TrivariateQuadraticPolynomial.cpp
