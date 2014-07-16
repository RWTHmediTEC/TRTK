// Last changed on 2012-08-03.


#include <TRTK/ErrorObj.hpp>

#include "unit_test.hpp"


void unit_test_BivariateQuadraticPolynomial();
void unit_test_BivariateCubicPolynomial();
void unit_test_BivariateQuarticPolynomial();
void unit_test_BivariateQuinticPolynomial();
void unit_test_BivariateSexticPolynomial();
void unit_test_Circle();
void unit_test_Clock();
void unit_test_Coordinate();
void unit_test_CorrelationCircle();
void unit_test_Diffusion();
void unit_test_ErrorObj();
void unit_test_EstimateAffineTransformation2D();
void unit_test_EstimateAffineTransformation3D();
void unit_test_EstimateAffineTransformationFromPlaneTo3D();
void unit_test_EstimateRigidTransformation2D();
void unit_test_EstimateRigidTransformation3D();
void unit_test_EstimateSimilarityTransformation2D();
void unit_test_EstimateSimilarityTransformation3D();
void unit_test_EstimateProjectiveTransformation2D();
void unit_test_EstimateProjectiveTransformation3D();
void unit_test_FitCircle();
void unit_test_FitCircle3D();
void unit_test_FitCircleInOrigin();
void unit_test_FitLine();
void unit_test_FitLine3D();
void unit_test_FitPlane();
void unit_test_FitSphere();
void unit_test_Icp();
void unit_test_Iterator();
void unit_test_Optimization();
void unit_test_GenericPolynomial();
void unit_test_PivotCalibration();
void unit_test_Range();
void unit_test_RegionGrowing2D();
void unit_test_RegionGrowing3D();
void unit_test_SurfaceExtraction3D();
void unit_test_Signals();
void unit_test_Timestamp();
void unit_test_Tools();
void unit_test_Transform2D();
void unit_test_Transform3D();
void unit_test_TrivariateQuadraticPolynomial();
void unit_test_TrivariateCubicPolynomial();
void unit_test_TrivariateQuarticPolynomial();
void unit_test_TrivariateQuinticPolynomial();
void unit_test_TrivariateSexticPolynomial();


int main()
{
    try
    {
        // Data Structures

        unit_test_Circle();


        // Error Handling and Tools

        unit_test_ErrorObj();
        unit_test_Tools();


        // Transformations

        unit_test_Coordinate();
        unit_test_Transform2D();
        unit_test_Transform3D();


        // Transformation Estimation

        unit_test_EstimateAffineTransformation2D();
        unit_test_EstimateAffineTransformation3D();
        unit_test_EstimateAffineTransformationFromPlaneTo3D();
        unit_test_EstimateRigidTransformation2D();
        unit_test_EstimateRigidTransformation3D();
        unit_test_EstimateSimilarityTransformation2D();
        unit_test_EstimateSimilarityTransformation3D();
        // unit_test_EstimateProjectiveTransformation2D();
        // unit_test_EstimateProjectiveTransformation3D();


        // Fitting

        unit_test_FitCircle();
        unit_test_FitCircle3D();
        unit_test_FitCircleInOrigin();
        unit_test_FitLine();
        unit_test_FitLine3D();
        unit_test_FitPlane();
        unit_test_FitSphere();

        // Polynomials

        unit_test_GenericPolynomial();
        // unit_test_BivariateQuadraticPolynomial();
        // unit_test_BivariateCubicPolynomial();
        // unit_test_BivariateQuarticPolynomial();
        // unit_test_BivariateQuinticPolynomial();
        // unit_test_BivariateSexticPolynomial();
        unit_test_TrivariateQuadraticPolynomial();
        // unit_test_TrivariateCubicPolynomial();
        // unit_test_TrivariateQuarticPolynomial();
        // unit_test_TrivariateQuinticPolynomial();
        // unit_test_TrivariateSexticPolynomial();


        // ICP

        unit_test_Icp();


        // Optimization

        unit_test_Optimization();


        // Segmentation

        unit_test_RegionGrowing2D();
        unit_test_RegionGrowing3D();


        // Correlation

        // unit_test_CorrelationCircle();


        // Calibration

        unit_test_PivotCalibration();


        // Data Enhancement

        unit_test_Diffusion();


        // Miscellaneous

        unit_test_Clock();
        unit_test_Iterator();
        unit_test_Range();
        // unit_test_Signals();
        unit_test_Timestamp();


        cout << endl << endl
             << "*****************************************************************************" << endl
             << "Error Summery" << endl
             << "*****************************************************************************" << endl
             << endl
             << "No errors occurred." << endl;
    }
    catch (TRTK::ErrorObj & error)
    {
        cout << error.what(TRTK::ErrorObj::VERBOSE) << endl;
    }
    catch (std::exception & error)
    {
        cout << "Error: " << error.what() << endl;
    }

    return 0;
}
