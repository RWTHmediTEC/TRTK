VERSION  = 0.6
# COMPILER = clang++
COMPILER = g++
CFLAGS   = -I ../../include -I ../../include/TRTK -I $(MEDITEC_LIBS)/Eigen/3.0.5/include/ -I $(MEDITEC_LIBS)/FLANN/tags/1.7.1/include -I ../
LIBS     = -lflann

OBJECTS  = Circle.o \
           Clock.o \
           Coordinate.o \
           ErrorObj.o \
           Signals.o \
           Timestamp.o \
           Tools.o \
           unit_test.o \
           unit_test_Circle.o \
           unit_test_Clock.o \
           unit_test_Coordinate.o \
           unit_test_Diffusion.o \
           unit_test_ErrorObj.o \
           unit_test_EstimateAffineTransformation2D.o \
           unit_test_EstimateAffineTransformation3D.o \
           unit_test_EstimateAffineTransformationFromPlaneTo3D.o \
           unit_test_EstimateRigidTransformation2D.o \
           unit_test_EstimateRigidTransformation3D.o \
           unit_test_EstimateSimilarityTransformation2D.o \
           unit_test_EstimateSimilarityTransformation3D.o \
           unit_test_EstimateProjectiveTransformation2D.o \
           unit_test_EstimateProjectiveTransformation3D.o \
           unit_test_FitCircle.o \
           unit_test_FitCircle3D.o \
           unit_test_FitCircleInOrigin.o \
           unit_test_FitLine.o \
           unit_test_FitLine3D.o \
           unit_test_FitPlane.o \
           unit_test_FitSphere.o \
           unit_test_GenericPolynomial.o \
           unit_test_Icp.o \
           unit_test_Iterator.o \
           unit_test_Optimization.o \
           unit_test_PivotCalibration.o \
           unit_test_Range.o \
           unit_test_RegionGrowing2D.o \
           unit_test_RegionGrowing3D.o \
           unit_test_Signals.o \
           unit_test_SurfaceExtraction3D.o \
           unit_test_Timestamp.o \
           unit_test_Tools.o \
           unit_test_Transform2D.o \
           unit_test_Transform3D.o \
           unit_test_TrivariateQuadraticPolynomial.o

all: $(OBJECTS)
	@echo Linking...
	@$(COMPILER) $(CFLAGS) -o unit_tests $(OBJECTS) $(LIBS) && echo Done. || echo Linking failed!

clean:
	@echo Remove all object files...
	@rm -f *.o

Circle.o: ../../src/Circle.cpp
	$(COMPILER) $(CFLAGS) -c $<

Clock.o: ../../src/Clock.cpp
	$(COMPILER) $(CFLAGS) -c $<

Coordinate.o: ../../src/Coordinate.cpp
	$(COMPILER) $(CFLAGS) -c $<

ErrorObj.o: ../../src/ErrorObj.cpp
	$(COMPILER) $(CFLAGS) -c $<

Signals.o: ../../src/Signals.cpp
	$(COMPILER) $(CFLAGS) -c $<

Timestamp.o: ../../src/Timestamp.cpp
	$(COMPILER) $(CFLAGS) -c $<

Tools.o: ../../src/Tools.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test.o: unit_test.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Circle.o: ../unit_test_Circle.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Clock.o: ../unit_test_Clock.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Coordinate.o: ../unit_test_Coordinate.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_CorrelationCircle.o: ../unit_test_CorrelationCircle.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Diffusion.o: ../unit_test_Diffusion.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_ErrorObj.o: ../unit_test_ErrorObj.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateAffineTransformation2D.o: ../unit_test_EstimateAffineTransformation2D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateAffineTransformation3D.o: ../unit_test_EstimateAffineTransformation3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateAffineTransformationFromPlaneTo3D.o: ../unit_test_EstimateAffineTransformationFromPlaneTo3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateRigidTransformation2D.o: ../unit_test_EstimateRigidTransformation2D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateRigidTransformation3D.o: ../unit_test_EstimateRigidTransformation3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateSimilarityTransformation2D.o: ../unit_test_EstimateSimilarityTransformation2D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateSimilarityTransformation3D.o: ../unit_test_EstimateSimilarityTransformation3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateProjectiveTransformation2D.o: ../unit_test_EstimateProjectiveTransformation2D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_EstimateProjectiveTransformation3D.o: ../unit_test_EstimateProjectiveTransformation3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_FitCircle.o: ../unit_test_FitCircle.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_FitCircle3D.o: ../unit_test_FitCircle3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_FitCircleInOrigin.o: ../unit_test_FitCircleInOrigin.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_FitLine.o: ../unit_test_FitLine.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_FitLine3D.o: ../unit_test_FitLine3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_FitPlane.o: ../unit_test_FitPlane.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_FitSphere.o: ../unit_test_FitSphere.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_GenericPolynomial.o: ../unit_test_GenericPolynomial.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Icp.o: ../unit_test_Icp.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Iterator.o: ../unit_test_Iterator.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Optimization.o: ../unit_test_Optimization.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_PivotCalibration.o: ../unit_test_PivotCalibration.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Range.o: ../unit_test_Range.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_RegionGrowing2D.o: ../unit_test_RegionGrowing2D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_RegionGrowing3D.o: ../unit_test_RegionGrowing3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Signals.o: ../unit_test_Signals.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_SurfaceExtraction3D.o: ../unit_test_SurfaceExtraction3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Timestamp.o: ../unit_test_Timestamp.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Tools.o: ../unit_test_Tools.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Transform2D.o: ../unit_test_Transform2D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_Transform3D.o: ../unit_test_Transform3D.cpp
	$(COMPILER) $(CFLAGS) -c $<

unit_test_TrivariateQuadraticPolynomial.o: ../unit_test_TrivariateQuadraticPolynomial.cpp
	$(COMPILER) $(CFLAGS) -c $<
