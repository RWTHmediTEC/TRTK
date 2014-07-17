// Last changed on 2011-06-06.


// NOTE: This unit test assumes Coordinate<T> and Transform2D<T> to work properly.


#include <vector>

#include <TRTK/Coordinate.hpp>
#include <TRTK/EstimateSimilarityTransformation2D.hpp>
#include <TRTK/Transform2D.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_EstimateSimilarityTransformation2D()
{
    using std::cout;
    using std::endl;
    using std::vector;

    using TRTK::Tools::isEqual;

    using namespace TRTK;


    typedef EstimateSimilarityTransformation2D<double> EstimateSimilarityTransformation;
    typedef EstimateSimilarityTransformation::value_type T;
    typedef EstimateSimilarityTransformation::Matrix2T Matrix2T;
    typedef EstimateSimilarityTransformation::Matrix3T Matrix3T;
    typedef EstimateSimilarityTransformation::Vector2T Vector2T;
    typedef EstimateSimilarityTransformation::Vector3T Vector3T;


    // Construct a transformation which rotates 90 degrees counter-clockwise
    // around point (1, 3) and scales with a factor of 2.

    Transform2D<T> transform;
    const T pi = Transform2D<T>::pi;
    transform.translate(-1, -3).rotate(pi/2).scale(2, 2).translate(1, 3);


    // Construct sets with source points.

    vector<Coordinate<T> >                                 source_points1;
    vector<Vector2T, Eigen::aligned_allocator<Vector2T> >  source_points2;
    vector<Vector3T>                                       source_points3;

    Coordinate<T> source_point1( 1,  2);
    Coordinate<T> source_point2( 3, -2);
    Coordinate<T> source_point3(-1,  1);
    Coordinate<T> source_point4( 2,  0);

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
    vector<Vector2T, Eigen::aligned_allocator<Vector2T> >  target_points2;
    vector<Vector3T>                                       target_points3;

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


    HEADING(EstimateSimilarityTransformation2D<T>)


    SUBHEADING(Constructors)


        // Empty constructor.

        START_TEST
            EstimateSimilarityTransformation estimateSimilarityTransformation;
            assert(estimateSimilarityTransformation.getTransformationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateSimilarityTransformation.getRotationMatrix().isApprox(Matrix2T::Identity()));
            assert(estimateSimilarityTransformation.getTranslationVector().isApprox(Vector2T::Zero()));
        STOP_TEST


        // Constructor with vector<Coordinate>.

        START_TEST
            EstimateSimilarityTransformation estimateSimilarityTransformation1(source_points1, target_points1);
            assert(estimateSimilarityTransformation1.getTransformationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateSimilarityTransformation1.getRotationMatrix().isApprox(Matrix2T::Identity()));
            assert(estimateSimilarityTransformation1.getTranslationVector().isApprox(Vector2T::Zero()));
        STOP_TEST


        // Constructor with vector<Vector2T>.

        START_TEST
            EstimateSimilarityTransformation estimateSimilarityTransformation2(source_points2, target_points2);
            assert(estimateSimilarityTransformation2.getTransformationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateSimilarityTransformation2.getRotationMatrix().isApprox(Matrix2T::Identity()));
            assert(estimateSimilarityTransformation2.getTranslationVector().isApprox(Vector2T::Zero()));
        STOP_TEST


        // Constructor with vector<Vector3T>.

        START_TEST
            EstimateSimilarityTransformation estimateSimilarityTransformation3(source_points3, target_points3);
            assert(estimateSimilarityTransformation3.getTransformationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateSimilarityTransformation3.getRotationMatrix().isApprox(Matrix2T::Identity()));
            assert(estimateSimilarityTransformation3.getTranslationVector().isApprox(Vector2T::Zero()));
        STOP_TEST


    SUBHEADING(setSourcePoints())


        // Coordinate<T>

        START_TEST
            estimateSimilarityTransformation.setSourcePoints(source_points1);
        STOP_TEST


        START_TEST
            source_points1[2] = Coordinate<T>(1, 2, 3, 4);

            try
            {
                estimateSimilarityTransformation.setSourcePoints(source_points1);
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateSimilarityTransformation::WRONG_POINT_SIZE);
            }

            source_points1[2] = source_point3;
        STOP_TEST


        // Vector2T

        START_TEST
            estimateSimilarityTransformation.setSourcePoints(source_points2);
        STOP_TEST


        // Vector3T

        START_TEST
            estimateSimilarityTransformation.setSourcePoints(source_points3);
        STOP_TEST


    SUBHEADING(setTargetPoints())


        // Coordinate<T>

        START_TEST
            estimateSimilarityTransformation.setTargetPoints(target_points1);
        STOP_TEST


        START_TEST
            target_points1[2] = Coordinate<T>(1, 2, 3, 4);

            try
            {
                estimateSimilarityTransformation.setTargetPoints(target_points1);
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateSimilarityTransformation::WRONG_POINT_SIZE);
            }

            target_points1[2] = target_point3;
        STOP_TEST


        // Vector2T

        START_TEST
            estimateSimilarityTransformation.setTargetPoints(target_points2);
        STOP_TEST


        // Vector3T

        START_TEST
            estimateSimilarityTransformation.setTargetPoints(target_points3);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            source_points1.push_back(Coordinate<T>(1, 2));
            estimateSimilarityTransformation.setSourcePoints(source_points1);

            try
            {
                estimateSimilarityTransformation.compute();
                const bool UNEQUAL_NUMBER_OF_POINTS_NOT_DETECTED = false;
                assert(UNEQUAL_NUMBER_OF_POINTS_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateSimilarityTransformation::UNEQUAL_NUMBER_OF_POINTS);
            }

            // restore the old state
            source_points1.pop_back();
            estimateSimilarityTransformation.setSourcePoints(source_points1);
        STOP_TEST


        START_TEST
            source_points1.clear();
            source_points1.push_back(source_point1);

            target_points1.clear();
            target_points1.push_back(target_point1);

            estimateSimilarityTransformation.setSourcePoints(source_points1);
            estimateSimilarityTransformation.setTargetPoints(target_points1);

            try
            {
                estimateSimilarityTransformation.compute();
                const bool NOT_ENOUGH_POINTS_NOT_DETECTED = false;
                assert(NOT_ENOUGH_POINTS_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateSimilarityTransformation::NOT_ENOUGH_POINTS);
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

            estimateSimilarityTransformation.setSourcePoints(source_points1);
            estimateSimilarityTransformation.setTargetPoints(target_points1);
        STOP_TEST


        START_TEST
            estimateSimilarityTransformation.compute();
        STOP_TEST


    SUBHEADING(getTransformationMatrix())


        START_TEST
            assert(estimateSimilarityTransformation.getTransformationMatrix().isApprox(transform.getTransformationMatrix()));
        STOP_TEST


    SUBHEADING(getRotationMatrix())


        START_TEST
            assert(estimateSimilarityTransformation.getRotationMatrix().isApprox(transform.getTransformationMatrix().block(0, 0, 2, 2)));
        STOP_TEST


    SUBHEADING(getTranslationVector())


        START_TEST
            assert(estimateSimilarityTransformation.getTranslationVector().isApprox(transform.getTransformationMatrix().block(0, 2, 2, 1)));
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            assert(estimateSimilarityTransformation.getRMS() < 1e-10);
        STOP_TEST


    SUBHEADING(getRotationAngle())


        START_TEST
            assert(isEqual(estimateSimilarityTransformation.getRotationAngle(), pi / 2.0));
        STOP_TEST


    SUBHEADING(getScalingFactor())


        START_TEST
            assert(isEqual(estimateSimilarityTransformation.getScalingFactor(), 2.0));
        STOP_TEST
}
