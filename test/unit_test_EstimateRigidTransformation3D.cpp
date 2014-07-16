// Last changed on 2012-10-12.


// NOTE: This unit test assumes Coordinate<T> and Transform3D<T> to work properly.


#include <vector>

#include <TRTK/Coordinate.hpp>
#include <TRTK/Transform3D.hpp>
#include <TRTK/EstimateRigidTransformation3D.hpp>

#include "unit_test.hpp"


void unit_test_EstimateRigidTransformation3D()
{
    using std::cout;
    using std::endl;
    using std::vector;

    using namespace TRTK;


    typedef EstimateRigidTransformation3D<double> EstimateRigidTransformation;
    typedef EstimateRigidTransformation::value_type T;
    typedef EstimateRigidTransformation::Matrix3T Matrix3T;
    typedef EstimateRigidTransformation::Matrix4T Matrix4T;
    typedef EstimateRigidTransformation::Vector3T Vector3T;
    typedef EstimateRigidTransformation::Vector4T Vector4T;


    // Construct a transformation which rotates 90 degrees counter-clockwise
    // in the x-y plane with a center of rotation of (1, 3, 0).

    Transform3D<T> transform;
    const T pi = Transform3D<T>::pi;
    transform.translate(-1, -3, 0).rotateZ(pi/2).translate(1, 3, 0);


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


    HEADING(EstimateRigidTransformation3D<T>)


    SUBHEADING(Constructors)


        // Empty constructor.

        START_TEST
            EstimateRigidTransformation estimateRigidTransformation;
            assert(estimateRigidTransformation.getTransformationMatrix().isApprox(Matrix4T::Identity()));
            assert(estimateRigidTransformation.getRotationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateRigidTransformation.getTranslationVector().isApprox(Vector3T::Zero()));
        STOP_TEST


        // Constructor with vector<Coordinate>.

        START_TEST
            EstimateRigidTransformation estimateRigidTransformation1(source_points1, target_points1);
            assert(estimateRigidTransformation1.getTransformationMatrix().isApprox(Matrix4T::Identity()));
            assert(estimateRigidTransformation1.getRotationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateRigidTransformation1.getTranslationVector().isApprox(Vector3T::Zero()));
        STOP_TEST


        // Constructor with vector<Vector3T>.

        START_TEST
            EstimateRigidTransformation estimateRigidTransformation2(source_points2, target_points2);
            assert(estimateRigidTransformation2.getTransformationMatrix().isApprox(Matrix4T::Identity()));
            assert(estimateRigidTransformation2.getRotationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateRigidTransformation2.getTranslationVector().isApprox(Vector3T::Zero()));
        STOP_TEST


        // Constructor with vector<Vector4T>.

        START_TEST
            EstimateRigidTransformation estimateRigidTransformation3(source_points3, target_points3);
            assert(estimateRigidTransformation3.getTransformationMatrix().isApprox(Matrix4T::Identity()));
            assert(estimateRigidTransformation3.getRotationMatrix().isApprox(Matrix3T::Identity()));
            assert(estimateRigidTransformation3.getTranslationVector().isApprox(Vector3T::Zero()));
        STOP_TEST


    SUBHEADING(setSourcePoints())


        // Coordinate<T>

        START_TEST
            estimateRigidTransformation.setSourcePoints(source_points1);
        STOP_TEST


        START_TEST
            source_points1[2] = (Coordinate<T>(1, 2, 3, 4), 5);

            try
            {
                estimateRigidTransformation.setSourcePoints(source_points1);
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateRigidTransformation::WRONG_POINT_SIZE);
            }

            source_points1[2] = source_point3;
        STOP_TEST


        // Vector3T

        START_TEST
            estimateRigidTransformation.setSourcePoints(source_points2);
        STOP_TEST


        // Vector4T

        START_TEST
            estimateRigidTransformation.setSourcePoints(source_points3);
        STOP_TEST


    SUBHEADING(setTargetPoints())


        // Coordinate<T>

        START_TEST
            estimateRigidTransformation.setTargetPoints(target_points1);
        STOP_TEST


        START_TEST
            target_points1[2] = (Coordinate<T>(1, 2, 3, 4), 5);

            try
            {
                estimateRigidTransformation.setTargetPoints(target_points1);
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateRigidTransformation::WRONG_POINT_SIZE);
            }

            target_points1[2] = target_point3;
        STOP_TEST


        // Vector3T

        START_TEST
            estimateRigidTransformation.setTargetPoints(target_points2);
        STOP_TEST


        // Vector4T

        START_TEST
            estimateRigidTransformation.setTargetPoints(target_points3);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            source_points1.push_back(Coordinate<T>(1, 2, 3));
            estimateRigidTransformation.setSourcePoints(source_points1);

            try
            {
                estimateRigidTransformation.compute();
                const bool UNEQUAL_NUMBER_OF_POINTS_NOT_DETECTED = false;
                assert(UNEQUAL_NUMBER_OF_POINTS_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateRigidTransformation::UNEQUAL_NUMBER_OF_POINTS);
            }

            // restore the old state
            source_points1.pop_back();
            estimateRigidTransformation.setSourcePoints(source_points1);
        STOP_TEST


        START_TEST
            source_points1.clear();
            source_points1.push_back(source_point1);

            target_points1.clear();
            target_points1.push_back(target_point1);

            estimateRigidTransformation.setSourcePoints(source_points1);
            estimateRigidTransformation.setTargetPoints(target_points1);

            try
            {
                estimateRigidTransformation.compute();
                const bool NOT_ENOUGH_POINTS_NOT_DETECTED = false;
                assert(NOT_ENOUGH_POINTS_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() == EstimateRigidTransformation::NOT_ENOUGH_POINTS);
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

            estimateRigidTransformation.setSourcePoints(source_points1);
            estimateRigidTransformation.setTargetPoints(target_points1);
        STOP_TEST


        START_TEST
            estimateRigidTransformation.compute();
        STOP_TEST


    SUBHEADING(getTransformationMatrix())


        START_TEST
            assert(estimateRigidTransformation.getTransformationMatrix().isApprox(transform.getTransformationMatrix()));
        STOP_TEST


    SUBHEADING(getRotationMatrix())


        START_TEST
            assert(estimateRigidTransformation.getRotationMatrix().isApprox(transform.getTransformationMatrix().block(0, 0, 3, 3)));
        STOP_TEST


    SUBHEADING(getTranslationVector())


        START_TEST
            assert(estimateRigidTransformation.getTranslationVector().isApprox(transform.getTransformationMatrix().block(0, 3, 3, 1)));
        STOP_TEST


    SUBHEADING(Handedness)


        START_TEST
            transform.reset().rotateZ(1.0).rotateY(2.0).translate(1, 2, 3); // arbitrary rotation in 3D

            // generate some test points

            vector<Coordinate<double> > source_points;

            source_points.push_back(Coordinate<double>( 1,  1, 0));
            source_points.push_back(Coordinate<double>(-1, -1, 0));
            source_points.push_back(Coordinate<double>( 2,  0, 0));

            vector<Coordinate<double> > target_points;

            target_points.push_back(transform * Coordinate<double>( 1,  1, 0));
            target_points.push_back(transform * Coordinate<double>(-1, -1, 0));
            target_points.push_back(transform * Coordinate<double>( 2,  0, 0) + Coordinate<double>(0, 0, 1e-15)); // leads to a left-handed coordinate system

            // estimate the transformation

            EstimateRigidTransformation estimator(source_points, target_points);

            estimator.compute();

            assert(estimator.getTransformationMatrix().determinant() > 0); // right-handed coordinate system?

            // second test for right-handness (generate new points along the z-axis and ...)

            Coordinate<double> s1 = Coordinate<double>( 2,  5, 1);
            Coordinate<double> s2 = Coordinate<double>(-3,  2, 1);

            Coordinate<double> t1 = transform * Coordinate<double>( 2,  5, 1);
            Coordinate<double> t2 = transform * Coordinate<double>(-3,  2, 1);

            Transform3D<double> transformator(estimator.getTransformationMatrix());

            double residual = (transformator * s1 - t1).squaredNorm() + (transformator * s2 - t2).squaredNorm();

            assert(residual < 1e-15);
        STOP_TEST
}
