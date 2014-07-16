// Last changed on 2011-08-05.


// NOTE: This unit test assumes Coordinate<T> and Transform2D<T> to work properly.


#include <limits>
#include <vector>

#include <TRTK/Coordinate.hpp>
#include <TRTK/Transform2D.hpp>
#include <TRTK/EstimateProjectiveTransformation2D.hpp>

#include "unit_test.hpp"


void unit_test_EstimateProjectiveTransformation2D()
{
    using std::vector;

    using namespace TRTK;


    typedef EstimateProjectiveTransformation2D<double> EstimateProjectiveTransformation;
    typedef EstimateProjectiveTransformation::value_type T;
    typedef EstimateProjectiveTransformation::Matrix2T Matrix2T;
    typedef EstimateProjectiveTransformation::Matrix3T Matrix3T;
    typedef EstimateProjectiveTransformation::Vector2T Vector2T;
    typedef EstimateProjectiveTransformation::Vector3T Vector3T;


    // Construct an affine transformation.

    Transform2D<T> transform;
    transform.a12() = 1;
    transform.a13() = 3;
    transform.a21() = 2;
    transform.a31() = 3;
    transform.a33() = 1;


    // Construct two sets with source and target points, respectively.

    vector<Coordinate<T> >                                 source_points1;
    vector<Vector2T, Eigen::aligned_allocator<Vector2T> >  source_points2;
    vector<Vector3T>                                       source_points3;

    vector<Coordinate<T> >                                 target_points1;
    vector<Vector2T, Eigen::aligned_allocator<Vector2T> >  target_points2;
    vector<Vector3T>                                       target_points3;

    for (unsigned i = 0; i < 50; ++i)
    {
        double x = std::rand() % 100;
        double y = std::rand() % 100;

        Coordinate<double> source_point(x, y);
        Coordinate<double> target_point = transform * source_point;

        source_points1.push_back(source_point);
        source_points2.push_back(source_point.toArray());
        source_points3.push_back((source_point, 1).toArray());

        target_points1.push_back(target_point);
        target_points2.push_back(target_point.toArray());
        target_points3.push_back((target_point, 1).toArray());
    }


    HEADING(EstimateProjectiveTransformation2D<T>)


    SUBHEADING(Constructors)


        // Empty constructor.

        START_TEST
            EstimateProjectiveTransformation estimateProjectiveTransformation;
            assert(estimateProjectiveTransformation.getTransformationMatrix().isApprox(Matrix3T::Identity()));
        STOP_TEST


        // Constructor with vector<Coordinate>.

        START_TEST
            EstimateProjectiveTransformation estimateProjectiveTransformation1(source_points1, target_points1);
            assert(estimateProjectiveTransformation1.getTransformationMatrix().isApprox(Matrix3T::Identity()));
        STOP_TEST


        // Constructor with vector<Vector2T>.

        START_TEST
            EstimateProjectiveTransformation estimateProjectiveTransformation2(source_points2, target_points2);
            assert(estimateProjectiveTransformation2.getTransformationMatrix().isApprox(Matrix3T::Identity()));
        STOP_TEST


        // Constructor with vector<Vector3T>.

        START_TEST
            EstimateProjectiveTransformation estimateProjectiveTransformation3(source_points3, target_points3);
            assert(estimateProjectiveTransformation3.getTransformationMatrix().isApprox(Matrix3T::Identity()));
        STOP_TEST


    SUBHEADING(setSourcePoints())


        // Coordinate<T>

        START_TEST
            estimateProjectiveTransformation.setSourcePoints(source_points1);
        STOP_TEST


        START_TEST
            Coordinate<T> backup_point = source_points1[2];
            source_points1[2] = Coordinate<T>(1, 2, 3, 4);

            try
            {
                estimateProjectiveTransformation.setSourcePoints(source_points1);
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateProjectiveTransformation::WRONG_POINT_SIZE);
            }

            // restore the old state
            source_points1[2] = backup_point;
        STOP_TEST


        // Vector2T

        START_TEST
            estimateProjectiveTransformation.setSourcePoints(source_points2);
        STOP_TEST


        // Vector3T

        START_TEST
            estimateProjectiveTransformation.setSourcePoints(source_points3);
        STOP_TEST


    SUBHEADING(setTargetPoints())


        // Coordinate<T>

        START_TEST
            estimateProjectiveTransformation.setTargetPoints(target_points1);
        STOP_TEST


        START_TEST
            backup_point = target_points1[2];
            target_points1[2] = Coordinate<T>(1, 2, 3, 4);

            try
            {
                estimateProjectiveTransformation.setTargetPoints(target_points1);
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateProjectiveTransformation::WRONG_POINT_SIZE);
            }

            // restore the old state
            target_points1[2] = backup_point;
        STOP_TEST


        // Vector2T

        START_TEST
            estimateProjectiveTransformation.setTargetPoints(target_points2);
        STOP_TEST


        // Vector3T

        START_TEST
            estimateProjectiveTransformation.setTargetPoints(target_points3);
        STOP_TEST


    SUBHEADING(setMaxIterations())


        START_TEST
            estimateProjectiveTransformation.setMaxIterations(100);
        STOP_TEST


    SUBHEADING(setMaxRMS())


        START_TEST
            estimateProjectiveTransformation.setMaxRMS(1e-10);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            source_points1.push_back(Coordinate<T>(1, 2));
            estimateProjectiveTransformation.setSourcePoints(source_points1);

            try
            {
                estimateProjectiveTransformation.compute();
                const bool UNEQUAL_NUMBER_OF_POINTS_NOT_DETECTED = false;
                assert(UNEQUAL_NUMBER_OF_POINTS_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateProjectiveTransformation::UNEQUAL_NUMBER_OF_POINTS);
            }

            // restore the old state
            source_points1.pop_back();
            estimateProjectiveTransformation.setSourcePoints(source_points1);
        STOP_TEST


        START_TEST
            vector<Coordinate<T> > source_points_tmp;
            source_points_tmp.push_back(source_points1[0]);

            vector<Coordinate<T> > target_points_tmp;
            target_points_tmp.push_back(target_points1[0]);

            estimateProjectiveTransformation.setSourcePoints(source_points_tmp);
            estimateProjectiveTransformation.setTargetPoints(target_points_tmp);

            try
            {
                estimateProjectiveTransformation.compute();
                const bool NOT_ENOUGH_POINTS_NOT_DETECTED = false;
                assert(NOT_ENOUGH_POINTS_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateProjectiveTransformation::NOT_ENOUGH_POINTS);
            }

            // restore the old state

            estimateProjectiveTransformation.setSourcePoints(source_points1);
            estimateProjectiveTransformation.setTargetPoints(target_points1);
        STOP_TEST


        START_TEST
            estimateProjectiveTransformation.compute();
        STOP_TEST


    SUBHEADING(getTransformationMatrix())


        START_TEST
            assert(estimateProjectiveTransformation.getTransformationMatrix().isApprox(transform.getTransformationMatrix()));
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            EstimateProjectiveTransformation estimateProjectiveTransformation4;
            assert(estimateProjectiveTransformation4.getRMS() < std::numeric_limits<T>::epsilon());
        STOP_TEST


        START_TEST
            assert(estimateProjectiveTransformation.getRMS() < 1e-10);
        STOP_TEST
}
