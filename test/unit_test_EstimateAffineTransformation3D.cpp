// Last changed on 2011-06-06.


// NOTE: This unit test assumes Coordinate<T> and Transform3D<T> to work properly.


#include <vector>

#include <TRTK/Coordinate.hpp>
#include <TRTK/Transform3D.hpp>
#include <TRTK/EstimateAffineTransformation3D.hpp>

#include "unit_test.hpp"


void unit_test_EstimateAffineTransformation3D()
{
    using std::cout;
    using std::endl;
    using std::vector;

    using namespace TRTK;


    typedef EstimateAffineTransformation3D<double> EstimateAffineTransformation;
    typedef EstimateAffineTransformation::value_type T;
    typedef EstimateAffineTransformation::Matrix3T Matrix3T;
    typedef EstimateAffineTransformation::Matrix4T Matrix4T;
    typedef EstimateAffineTransformation::Vector3T Vector3T;
    typedef EstimateAffineTransformation::Vector4T Vector4T;


    // Construct an affine transformation.

    Transform3D<T> transform;
    transform.a12() =  1;
    transform.a14() =  1;
    transform.a31() =  2;
    transform.a34() =  3;


    // Construct sets with source points.

    vector<Coordinate<T> >                                 source_points1;
    vector<Vector3T>                                       source_points2;
    vector<Vector4T, Eigen::aligned_allocator<Vector4T> >  source_points3;

    Coordinate<T> source_point1( 1,  2,  0);
    Coordinate<T> source_point2( 3, -2,  0);
    Coordinate<T> source_point3(-1,  1,  1);
    Coordinate<T> source_point4( 2,  0, -1);

    source_points1.push_back(source_point1);
    source_points1.push_back(source_point2);
    source_points1.push_back(source_point3);
    source_points1.push_back(source_point4);

    source_points2.push_back(source_point1.toArray());
    source_points2.push_back(source_point2.toArray());
    source_points2.push_back(source_point3.toArray());
    source_points2.push_back(source_point4.toArray());

    source_points3.push_back((source_point1, 1).toArray());
    source_points3.push_back((source_point2, 1).toArray());
    source_points3.push_back((source_point3, 1).toArray());
    source_points3.push_back((source_point4, 1).toArray());


    // Construct sets with corresponding target points.

    vector<Coordinate<T> >                                 target_points1;
    vector<Vector3T>                                       target_points2;
    vector<Vector4T, Eigen::aligned_allocator<Vector4T> >  target_points3;

    Coordinate<T> target_point1 = transform * source_point1;
    Coordinate<T> target_point2 = transform * source_point2;
    Coordinate<T> target_point3 = transform * source_point3;
    Coordinate<T> target_point4 = transform * source_point4;

    target_points1.push_back(target_point1);
    target_points1.push_back(target_point2);
    target_points1.push_back(target_point3);
    target_points1.push_back(target_point4);

    target_points2.push_back(target_point1.toArray());
    target_points2.push_back(target_point2.toArray());
    target_points2.push_back(target_point3.toArray());
    target_points2.push_back(target_point4.toArray());

    target_points3.push_back((target_point1, 1).toArray());
    target_points3.push_back((target_point2, 1).toArray());
    target_points3.push_back((target_point3, 1).toArray());
    target_points3.push_back((target_point4, 1).toArray());


    HEADING(EstimateAffineTransformation3D<T>)


    SUBHEADING(Constructors)


        // Empty constructor.

        START_TEST
            EstimateAffineTransformation estimateAffineTransformation;
            assert(estimateAffineTransformation.getTransformationMatrix().isApprox(Matrix4T::Identity()));
            assert(estimateAffineTransformation.getLinearTransformationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateAffineTransformation.getTranslationVector().isApprox(Vector3T::Zero()));
        STOP_TEST


        // Constructor with vector<Coordinate>.

        START_TEST
            EstimateAffineTransformation estimateAffineTransformation1(source_points1, target_points1);
            assert(estimateAffineTransformation1.getTransformationMatrix().isApprox(Matrix4T::Identity()));
            assert(estimateAffineTransformation1.getLinearTransformationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateAffineTransformation1.getTranslationVector().isApprox(Vector3T::Zero()));
        STOP_TEST


        // Constructor with vector<Vector3T>.

        START_TEST
            EstimateAffineTransformation estimateAffineTransformation2(source_points2, target_points2);
            assert(estimateAffineTransformation2.getTransformationMatrix().isApprox(Matrix4T::Identity()));
            assert(estimateAffineTransformation2.getLinearTransformationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateAffineTransformation2.getTranslationVector().isApprox(Vector3T::Zero()));
        STOP_TEST


        // Constructor with vector<Vector4T>.

        START_TEST
            EstimateAffineTransformation estimateAffineTransformation3(source_points3, target_points3);
            assert(estimateAffineTransformation3.getTransformationMatrix().isApprox(Matrix4T::Identity()));
            assert(estimateAffineTransformation3.getLinearTransformationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateAffineTransformation3.getTranslationVector().isApprox(Vector3T::Zero()));
        STOP_TEST


    SUBHEADING(setSourcePoints())


        // Coordinate<T>

        START_TEST
            estimateAffineTransformation.setSourcePoints(source_points1);
        STOP_TEST


        START_TEST
            source_points1[2] = (Coordinate<T>(1, 2, 3, 4), 5);

            try
            {
                estimateAffineTransformation.setSourcePoints(source_points1);
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateAffineTransformation::WRONG_POINT_SIZE);
            }

            source_points1[2] = source_point3;
        STOP_TEST


        // Vector3T

        START_TEST
            estimateAffineTransformation.setSourcePoints(source_points2);
        STOP_TEST


        // Vector4T

        START_TEST
            estimateAffineTransformation.setSourcePoints(source_points3);
        STOP_TEST


    SUBHEADING(setTargetPoints())


        // Coordinate<T>

        START_TEST
            estimateAffineTransformation.setTargetPoints(target_points1);
        STOP_TEST


        START_TEST
            target_points1[2] = (Coordinate<T>(1, 2, 3, 4), 5);

            try
            {
                estimateAffineTransformation.setTargetPoints(target_points1);
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateAffineTransformation::WRONG_POINT_SIZE);
            }

            target_points1[2] = target_point3;
        STOP_TEST


        // Vector3T

        START_TEST
            estimateAffineTransformation.setTargetPoints(target_points2);
        STOP_TEST


        // Vector4T

        START_TEST
            estimateAffineTransformation.setTargetPoints(target_points3);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            source_points1.push_back(Coordinate<T>(1, 2, 3));
            estimateAffineTransformation.setSourcePoints(source_points1);

            try
            {
                estimateAffineTransformation.compute();
                const bool UNEQUAL_NUMBER_OF_POINTS_NOT_DETECTED = false;
                assert(UNEQUAL_NUMBER_OF_POINTS_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateAffineTransformation::UNEQUAL_NUMBER_OF_POINTS);
            }

            // restore the old state
            source_points1.pop_back();
            estimateAffineTransformation.setSourcePoints(source_points1);
        STOP_TEST


        START_TEST
            source_points1.clear();
            source_points1.push_back(source_point1);

            target_points1.clear();
            target_points1.push_back(target_point1);

            estimateAffineTransformation.setSourcePoints(source_points1);
            estimateAffineTransformation.setTargetPoints(target_points1);

            try
            {
                estimateAffineTransformation.compute();
                const bool NOT_ENOUGH_POINTS_NOT_DETECTED = false;
                assert(NOT_ENOUGH_POINTS_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateAffineTransformation::NOT_ENOUGH_POINTS);
            }

            // restore the old state

            source_points1.clear();
            source_points1.push_back(source_point1);
            source_points1.push_back(source_point2);
            source_points1.push_back(source_point3);
            source_points1.push_back(source_point4);

            target_points1.clear();
            target_points1.push_back(target_point1);
            target_points1.push_back(target_point2);
            target_points1.push_back(target_point3);
            target_points1.push_back(target_point4);

            estimateAffineTransformation.setSourcePoints(source_points1);
            estimateAffineTransformation.setTargetPoints(target_points1);
        STOP_TEST


        START_TEST
            estimateAffineTransformation.compute();
        STOP_TEST


    SUBHEADING(getTransformationMatrix())


        START_TEST
            assert(estimateAffineTransformation.getTransformationMatrix().isApprox(transform.getTransformationMatrix()));
        STOP_TEST


    SUBHEADING(getLinearTransformationMatrix())


        START_TEST
            assert(estimateAffineTransformation.getLinearTransformationMatrix().isApprox(transform.getTransformationMatrix().block(0, 0, 3, 3)));
        STOP_TEST


    SUBHEADING(getTranslationVector())


        START_TEST
            assert(estimateAffineTransformation.getTranslationVector().isApprox(transform.getTransformationMatrix().block(0, 3, 3, 1)));
        STOP_TEST
}
