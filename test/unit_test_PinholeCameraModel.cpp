// Last changed on 2019-05-28.

#include <cmath>
#include <iostream>
#include <tuple>

#include <TRTK/ErrorObj.hpp>
#include <TRTK/PinholeCameraModel.hpp>
#include <TRTK/Tools.hpp>
#include <TRTK/Transform3D.hpp>

#include "unit_test.hpp"


namespace
{
    using namespace Eigen;
    using namespace std;
    using namespace TRTK;
    using namespace TRTK::Tools;

    double pi = 3.1415926535;

    using CameraModel = PinholeCameraModel<double>;
    using Point3D = CameraModel::Vector3T;
    using Point2D = CameraModel::Vector2T;

    struct GenerateTestData
    {
        GenerateTestData(double sigma = 0)
        {
            // Instantiate a pinhole camera model.

            CameraModel camera_model;
            camera_model.setFocalLengths(10, 10);
            T_proj = camera_model.getIntrinsicParameters();
            T_pose = camera_model.getExtrinsicParameters();

            // Generate some (noisy) test data.

            points_world.emplace_back(0, 0, 100); // mapped to (0, 0) in the image plane
            points_world.emplace_back(10, 10, 100); // mapped to (1, 1) in the image plane
            points_world.emplace_back(20, -10, 80); // mapped to (2.5, -1.25) in the image plane
            points_world.emplace_back(15, -10, 90);
            points_world.emplace_back(-20, 10, 100);
            points_world.emplace_back(-20, -20, 80);

            for (auto const & p_W : points_world)
            {
                Point2D p_D = camera_model.transform(p_W);
                points_display.push_back(p_D);
            }

            // Add noise to the points recorded in the world COS.

            for (auto & p_W : points_world)
            {
                p_W += Point3D(randn(0.0, sigma), randn(0.0, sigma), randn(0.0, sigma));
            }
        }

        vector<Point3D> points_world;
        vector<Point2D> points_display; // image plane
        MatrixXd T_proj;
        MatrixXd T_pose;
    };


    struct GenerateTestData2
    {
        GenerateTestData2(int number_of_points = 6, double sigma_world = 0, double sigma_display = 0)
        {
            // Instantiate a pinhole camera model.

            CameraModel camera_model;

            // Set the intrinsic camera parameters.

            camera_model.setFocalLengths(80, 79);
            camera_model.setImageCenter(250, 251);
            camera_model.setSkew(0.1);
            T_proj = camera_model.getIntrinsicParameters();

            // Set the extrinsic camera parameters.

            Transform3D<double> T;
            T.rotateAxis(40, Point3D(1, 2, 3), Transform3D<double>::DEGREES);
            T.translate(130, 135, 90);
            camera_model.setExtrinsicParameters(T.getTransformationMatrix());
            T_pose = camera_model.getExtrinsicParameters();
            MatrixXd T_camera_to_world = T_pose.inverse();

            // Generate some test data.

            for (int i = 0; i < number_of_points; ++i) // points in the world COS
            {
                auto x = randn(0.0, 30.0);    // in the camera COS
                auto y = randn(0.0, 30.0);    // in the camera COS
                auto z = randn(600.0, 10.0);  // in the camera COS
                Point3D p = (T_camera_to_world * Point3D(x, y, z).homogeneous()).hnormalized();
                points_world.emplace_back(p);
            }

            for (auto const & p_W : points_world) // points in the display COS; add noise if specified
            {
                Point2D p_D = camera_model.transform(p_W) + Point2D(randn(0.0, sigma_display), randn(0.0, sigma_display));
                points_display.push_back(p_D);
            }

            // Add noise to the points recorded in the world COS.

            for (auto & p_W : points_world)
            {
                p_W += Point3D(randn(0.0, sigma_world), randn(0.0, sigma_world), randn(0.0, sigma_world));
            }
        }

        vector<Point3D> points_world;
        vector<Point2D> points_display; // image plane
        MatrixXd T_proj;
        MatrixXd T_pose;
    };


    struct GenerateTestData3
    {
        GenerateTestData3(int number_of_points = 25, double sigma_world = 0, double sigma_display = 0)
        {
            // Instantiate a pinhole camera model.

            CameraModel camera_model;

            // Set the intrinsic camera parameters.

            camera_model.setFocalLengths(80, 80);
            camera_model.setImageCenter(250, 250);
            camera_model.setSkew(0);
            T_proj = camera_model.getIntrinsicParameters();

            // Set the extrinsic camera parameters.

            Transform3D<double> T;
            T.rotateAxis(40, Point3D(1, 2, 3), Transform3D<double>::DEGREES);
            T.translate(130, 135, 90);
            camera_model.setExtrinsicParameters(T.getTransformationMatrix());
            T_pose = camera_model.getExtrinsicParameters();
            MatrixXd T_camera_to_world = T_pose.inverse();

            // Generate some test data.

            for (int i = 0; i < number_of_points; ++i) // points in the world COS
            {
                auto x = randn(0.0, 30.0);    // in the camera COS
                auto y = randn(0.0, 30.0);    // in the camera COS
                auto z = randn(600.0, 10.0);  // in the camera COS
                Point3D p = (T_camera_to_world * Point3D(x, y, z).homogeneous()).hnormalized();
                points_world.emplace_back(p);
            }

            for (auto const & p_W : points_world) // points in the display COS; add noise if specified
            {
                Point2D p_D = camera_model.transform(p_W) + Point2D(randn(0.0, sigma_display), randn(0.0, sigma_display));
                points_display.push_back(p_D);
            }

            // Add noise to the points recorded in the world COS.

            for (auto & p_W : points_world)
            {
                p_W += Point3D(randn(0.0, sigma_world), randn(0.0, sigma_world), randn(0.0, sigma_world));
            }
        }

        vector<Point3D> points_world;
        vector<Point2D> points_display; // image plane
        MatrixXd T_proj;
        MatrixXd T_pose;
    };

} // end of anonymous namespace


void unit_test_PinholeCameraModel()
{
    using namespace Eigen;
    using namespace std;
    using namespace TRTK;
    using namespace TRTK::Tools;


    /////////////////////////////////////////////////////////////////////////


    HEADING(computeRQDecomposition<T>)


    SUBHEADING(RQ decomposition)


        START_TEST
        {
            Transform3D<double> T;
            using Vector3T = Transform3D<double>::Vector3T;
            T.rotateAxis(40, Vector3T(1, 1, 1), Transform3D<double>::DEGREES);

            Matrix3d R = (Matrix3d() << 1, 2, 3, 0, 4, 5, 0, 0, 6).finished(); // upper triangular matrix
            Matrix3d Q = T.getTransformationMatrix().block<3, 3>(0, 0); // rotation matrix --> orthogonal
            Matrix3d A = R * Q;

            Matrix3d R_estimated, Q_estimated;
            tie(R_estimated, Q_estimated) = computeRQDecomposition<double>(A);

            double estimation_error_R = (R - R_estimated).norm();
            double estimation_error_Q = (Q - Q_estimated).norm();
            double reconstruction_error_A = (A - R_estimated * Q_estimated).norm();
            double Q_is_orthogonal = (Matrix3d::Identity() - Q_estimated * Q_estimated.transpose()).norm();

            assert(estimation_error_R < 1e-7);
            assert(estimation_error_Q < 1e-7);
            assert(reconstruction_error_A < 1e-7);
            assert(Q_is_orthogonal < 1e-7);
        }
        STOP_TEST


    SUBHEADING(Uniqueness)


        START_TEST
        {
            // diagonal elements of R must always be greater than zero

            Transform3D<double> T;
            using Vector3T = Transform3D<double>::Vector3T;
            T.rotateAxis(40, Vector3T(1, 1, 1), Transform3D<double>::DEGREES);

            Matrix3d R = -1 * (Matrix3d() << 1, 2, 3, 0, 4, 5, 0, 0, 6).finished(); // upper triangular matrix
            Matrix3d Q = -1 * T.getTransformationMatrix().block<3, 3>(0, 0); // rotation matrix --> orthogonal
            Matrix3d A = R * Q;

            Matrix3d R_estimated, Q_estimated;
            tie(R_estimated, Q_estimated) = computeRQDecomposition<double>(A);

            double estimation_error_R = (-1 * R - R_estimated).norm();
            double estimation_error_Q = (-1 * Q - Q_estimated).norm();
            double reconstruction_error_A = (A - R_estimated * Q_estimated).norm();
            double Q_is_orthogonal = (Matrix3d::Identity() - Q_estimated * Q_estimated.transpose()).norm();

            assert(estimation_error_R < 1e-7);
            assert(estimation_error_Q < 1e-7);
            assert(reconstruction_error_A < 1e-7);
            assert(Q_is_orthogonal < 1e-7);
        }
        STOP_TEST


    SUBHEADING(Rank deficient matrix)


        START_TEST
        {
            // todo: diagonal elements of R must always be greater than zero

            Transform3D<double> T;
            using Vector3T = Transform3D<double>::Vector3T;
            T.rotateAxis(40, Vector3T(1, 1, 1), Transform3D<double>::DEGREES);

            Matrix3d R = (Matrix3d() << 1, 2, 3, 0, 2, 3, 0, 2, 3).finished(); // upper triangular matrix
            Matrix3d Q = T.getTransformationMatrix().block<3, 3>(0, 0); // rotation matrix --> orthogonal
            Matrix3d A = R * Q;

            Matrix3d R_estimated, Q_estimated;
            tie(R_estimated, Q_estimated) = computeRQDecomposition<double>(A);

            double reconstruction_error_A = (A - R_estimated * Q_estimated).norm();
            double Q_is_orthogonal = (Matrix3d::Identity() - Q_estimated * Q_estimated.transpose()).norm();

            assert(reconstruction_error_A < 1e-7);
            assert(Q_is_orthogonal < 1e-7);
        }
        STOP_TEST


    /////////////////////////////////////////////////////////////////////////


    HEADING(PinholeCameraModel<T>)


    SUBHEADING(Constructors & Destructor)


        START_TEST
        {
            PinholeCameraModel<double> pinhole_camera_model;
            assert((pinhole_camera_model.getCameraOrientation() - Matrix3d::Identity()).norm() < 1e-7);
            assert((pinhole_camera_model.getCameraParameters() - MatrixXd::Identity(3, 4)).norm() < 1e-7);
            assert(pinhole_camera_model.getCameraPosition().norm() < 1e-7);
            assert((pinhole_camera_model.getExtrinsicParameters() - Matrix4d::Identity()).norm() < 1e-7);
            auto focal_lengths = pinhole_camera_model.getFocalLengths();
            assert(abs(get<0>(focal_lengths) - 1) < 1e-7);
            assert(abs(get<1>(focal_lengths) - 1) < 1e-7);
            auto image_center = pinhole_camera_model.getImageCenter();
            assert(abs(get<0>(image_center)) < 1e-7);
            assert(abs(get<1>(image_center)) < 1e-7);
            assert(abs(pinhole_camera_model.getSkew()) < 1e-7);
        }
        STOP_TEST


    SUBHEADING(Getters and Setters)


        START_TEST
        {
            PinholeCameraModel<double> pinhole_camera_model;

            // Extrinsic parameters.

            Transform3D<double> T;
            using Vector3T = Transform3D<double>::Vector3T;
            T.rotateAxis(40, Vector3T(1, 1, 1), Transform3D<double>::DEGREES).translate(1.1, 2.2, 3.3);
            MatrixXd pose_matrix = T.getTransformationMatrix();
            MatrixXd rotation_matrix = T.getTransformationMatrix().block<3, 3>(0, 0);
            MatrixXd translation_vector = T.getTransformationMatrix().block<3, 1>(0, 3);
            pinhole_camera_model.setExtrinsicParameters(pose_matrix);
            assert((pinhole_camera_model.getExtrinsicParameters() - pose_matrix).norm() < 1e-7);
            assert((pinhole_camera_model.getCameraOrientation() - rotation_matrix).norm() < 1e-7);
            assert((pinhole_camera_model.getCameraPosition() - translation_vector).norm() < 1e-7);

            // Intrinsic parameters.

            MatrixXd projection_matrix = (Matrix<double, 3, 4>() << 80, 0.1, 250, 0,
                                                                    0,  85,  220, 0,
                                                                    0,  0,   1,   0).finished();
            pinhole_camera_model.setIntrinsicParameters(projection_matrix);
            assert((pinhole_camera_model.getIntrinsicParameters() - projection_matrix).norm() < 1e-7);
            auto focal_lengths = pinhole_camera_model.getFocalLengths();
            assert(abs(get<0>(focal_lengths) - 80) < 1e-7);
            assert(abs(get<1>(focal_lengths) - 85) < 1e-7);
            auto image_center = pinhole_camera_model.getImageCenter();
            assert(abs(get<0>(image_center) - 250) < 1e-7);
            assert(abs(get<1>(image_center) - 220) < 1e-7);
            assert(abs(pinhole_camera_model.getSkew() - 0.1) < 1e-7);

            // Other

            assert((pinhole_camera_model.getCameraParameters() - projection_matrix * pose_matrix).norm() < 1e-7);
        }
        STOP_TEST


        START_TEST
        {
            PinholeCameraModel<double> pinhole_camera_model;

            // Extrinsic parameters.

            Transform3D<double> T;
            using Vector3T = Transform3D<double>::Vector3T;
            T.rotateAxis(-22, Vector3T(2, 0, 4), Transform3D<double>::DEGREES).translate(11, 22, 33);
            MatrixXd pose_matrix = T.getTransformationMatrix();
            MatrixXd rotation_matrix = T.getTransformationMatrix().block<3, 3>(0, 0);
            MatrixXd translation_vector = T.getTransformationMatrix().block<3, 1>(0, 3);
            pinhole_camera_model.setCameraOrientation(rotation_matrix);
            pinhole_camera_model.setCameraPosition(translation_vector);
            assert((pinhole_camera_model.getCameraOrientation() - rotation_matrix).norm() < 1e-7);
            assert((pinhole_camera_model.getCameraPosition() - translation_vector).norm() < 1e-7);
            assert((pinhole_camera_model.getExtrinsicParameters() - pose_matrix).norm() < 1e-7);

            // Intrinsic parameters.

            MatrixXd projection_matrix = (Matrix<double, 3, 4>() << 55, -0.2, 500, 0,
                                                                    0,  56,   499, 0,
                                                                    0,  0,    1,   0).finished();
            pinhole_camera_model.setFocalLengths(55, 56);
            pinhole_camera_model.setImageCenter(500, 499);
            pinhole_camera_model.setSkew(-0.2);
            assert((pinhole_camera_model.getIntrinsicParameters() - projection_matrix).norm() < 1e-7);
            auto focal_lengths = pinhole_camera_model.getFocalLengths();
            assert(abs(get<0>(focal_lengths) - 55) < 1e-7);
            assert(abs(get<1>(focal_lengths) - 56) < 1e-7);
            auto image_center = pinhole_camera_model.getImageCenter();
            assert(abs(get<0>(image_center) - 500) < 1e-7);
            assert(abs(get<1>(image_center) - 499) < 1e-7);
            assert(abs(pinhole_camera_model.getSkew() - -0.2) < 1e-7);

            // Other

            assert((pinhole_camera_model.getCameraParameters() - projection_matrix * pose_matrix).norm() < 1e-7);
        }
        STOP_TEST


    SUBHEADING(Projection)


        START_TEST // transform
        {
            CameraModel camera_model;
            camera_model.setFocalLengths(10, 10);
            Point2D mapped_point;

            mapped_point = camera_model.transform(Point3D(0, 0, 100)); // mapped to (0, 0) in the image plane
            assert((mapped_point - Point2D(0, 0)).norm() < 1e-7);

            mapped_point = camera_model.transform(Point3D(10, 10, 100)); // mapped to (1, 1) in the image plane
            assert((mapped_point - Point2D(1, 1)).norm() < 1e-7);

            mapped_point = camera_model.transform(Point3D(20, -10, 80)); // mapped to (2.5, -1.25) in the image plane
            assert((mapped_point - Point2D(2.5, -1.25)).norm() < 1e-7);
        }
        STOP_TEST


        START_TEST // operator*()
        {
            CameraModel camera_model;
            camera_model.setFocalLengths(10, 10);
            Point2D mapped_point;

            mapped_point = camera_model * Point3D(0, 0, 100); // mapped to (0, 0) in the image plane
            assert((mapped_point - Point2D(0, 0)).norm() < 1e-7);

            mapped_point = camera_model * Point3D(10, 10, 100); // mapped to (1, 1) in the image plane
            assert((mapped_point - Point2D(1, 1)).norm() < 1e-7);

            mapped_point = camera_model * Point3D(20, -10, 80); // mapped to (2.5, -1.25) in the image plane
            assert((mapped_point - Point2D(2.5, -1.25)).norm() < 1e-7);
        }
        STOP_TEST


    SUBHEADING(Parameter Estimation)


        START_TEST
        {
            try
            {
                PinholeCameraModel<double> pinhole_camera_model;
                vector<Point3D> points_world;
                vector<Point2D> points_display;
                auto rmse = pinhole_camera_model.estimate(make_range(points_world), make_range(points_display));
                const bool WRONG_POINT_SIZE_NOT_DETECTED = false;
                assert(WRONG_POINT_SIZE_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() ==  PinholeCameraModel<double>::NOT_ENOUGH_INPUT_DATA);
            }
        }
        STOP_TEST


        START_TEST
        {
            try
            {
                PinholeCameraModel<double> pinhole_camera_model;
                vector<Point3D> points_world;
                vector<Point2D> points_display;
                points_display.emplace_back(Point2D(0, 0));
                auto rmse = pinhole_camera_model.estimate(make_range(points_world), make_range(points_display));
                const bool SET_SIZE_MISMATCH_NOT_DETECTED = false;
                assert(SET_SIZE_MISMATCH_NOT_DETECTED);
            }
            catch (ErrorObj & error)
            {
                assert(error.getErrorCode() ==  PinholeCameraModel<double>::UNEQUAL_CARDINALITY_OF_INPUT_SETS);
            }
        }
        STOP_TEST


        START_TEST
        {
            PinholeCameraModel<double> pinhole_camera_model;
            GenerateTestData test_data;
            double rmse = pinhole_camera_model.estimate(make_range(test_data.points_world), make_range(test_data.points_display));
            MatrixXd T_proj = pinhole_camera_model.getIntrinsicParameters();
            MatrixXd T_pose = pinhole_camera_model.getExtrinsicParameters();
            assert((T_proj - test_data.T_proj).norm() < 1e-7);
            assert((T_pose - test_data.T_pose).norm() < 1e-7);
        }
        STOP_TEST


        START_TEST
        {
            PinholeCameraModel<double> pinhole_camera_model;
            GenerateTestData2 test_data(16);
            double rmse = pinhole_camera_model.estimate(make_range(test_data.points_world), make_range(test_data.points_display));
            MatrixXd T_proj = pinhole_camera_model.getIntrinsicParameters();
            MatrixXd T_pose = pinhole_camera_model.getExtrinsicParameters();
            assert((T_proj - test_data.T_proj).norm() < 1e-7);
            assert((T_pose - test_data.T_pose).norm() < 1e-5);
        }
        STOP_TEST


        START_TEST
        {
            // estimate() does not cope too well with noisy data---at least when decomposing it
            PinholeCameraModel<double> pinhole_camera_model;
            GenerateTestData2 test_data(25, 0.001);
            double rmse = pinhole_camera_model.estimate(make_range(test_data.points_world), make_range(test_data.points_display));
            MatrixXd T_proj = pinhole_camera_model.getIntrinsicParameters();
            MatrixXd T_pose = pinhole_camera_model.getExtrinsicParameters();
            double error_T_proj = (T_proj - test_data.T_proj).norm(); // 0.123733
            double error_T_pose = (T_pose - test_data.T_pose).norm(); // 0.521966
            cout << endl;
            assert(rmse < 0.001);
            assert(error_T_proj < 0.2);
            assert(error_T_pose < 0.7);
        }
        STOP_TEST


    #ifdef CPPOPTLIB_FOUND

    SUBHEADING(estimateWithConstraints())


        START_TEST
        {
            PinholeCameraModel<double> pinhole_camera_model;
            GenerateTestData3 test_data(30, 0.5); // no skewed image plane
            double rmse = pinhole_camera_model.estimateWithConstraints(make_range(test_data.points_world), make_range(test_data.points_display), PinholeCameraModel<double>::NO_SKEW);
            MatrixXd T_proj = pinhole_camera_model.getIntrinsicParameters();
            MatrixXd T_pose = pinhole_camera_model.getExtrinsicParameters();
            double error_T_proj = (T_proj - test_data.T_proj).norm() / test_data.T_proj.norm(); // 0.025331
            double error_T_pose = (T_pose - test_data.T_pose).norm() / test_data.T_pose.norm(); // 0.249261
            assert(rmse < 0.2);
            assert(error_T_proj < 0.25); // mostly due to a inexact estimate of the focal lengths (previously 0.1)
            assert(error_T_pose < 2.5); // position may deviate in the z-direction (previously 0.3)
        }
        STOP_TEST


        START_TEST
        {
            PinholeCameraModel<double> pinhole_camera_model;
            GenerateTestData3 test_data(30, 0.5); // no skewed image plane
            double rmse = pinhole_camera_model.estimateWithConstraints(make_range(test_data.points_world), make_range(test_data.points_display), PinholeCameraModel<double>::SAME_FOCAL_LENGTHS);
            MatrixXd T_proj = pinhole_camera_model.getIntrinsicParameters();
            MatrixXd T_pose = pinhole_camera_model.getExtrinsicParameters();
            double error_T_proj = (T_proj - test_data.T_proj).norm() / test_data.T_proj.norm(); // 0.0128983
            double error_T_pose = (T_pose - test_data.T_pose).norm() / test_data.T_pose.norm(); // 0.0485156
            assert(rmse < 0.2);
            assert(error_T_proj < 0.1); // mostly due to a inexact estimate of the focal lengths
            assert(error_T_pose < 0.7); // position may deviate in the z-direction (previously 0.3)
        }
        STOP_TEST


        START_TEST
        {
            PinholeCameraModel<double> pinhole_camera_model;
            GenerateTestData3 test_data(30, 0.5); // no skewed image plane
            double rmse = pinhole_camera_model.estimateWithConstraints(make_range(test_data.points_world), make_range(test_data.points_display), PinholeCameraModel<double>::SAME_FOCAL_LENGTHS_AND_NO_SKEW);
            MatrixXd T_proj = pinhole_camera_model.getIntrinsicParameters();
            MatrixXd T_pose = pinhole_camera_model.getExtrinsicParameters();
            double error_T_proj = (T_proj - test_data.T_proj).norm() / test_data.T_proj.norm(); // 0.057552
            double error_T_pose = (T_pose - test_data.T_pose).norm() / test_data.T_pose.norm(); // 0.394669
            assert(rmse < 0.2);
            assert(error_T_proj < 0.1); // mostly due to a inexact estimate of the focal lengths
            assert(error_T_pose < 0.5); // position may deviate in the z-direction
        }
        STOP_TEST

    #endif // CPPOPTLIB_FOUND

}
