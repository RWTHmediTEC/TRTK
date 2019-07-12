// Last changed on 2019-06-12.

#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

#include<Eigen/Core>
#include<Eigen/StdVector>

#include <TRTK/Clock.hpp>
#include <TRTK/Coordinate.hpp>
#include <TRTK/PinholeCameraModel.hpp>
#include <TRTK/Tools.hpp>
#include <TRTK/Transform3D.hpp>


using namespace Eigen;
using namespace std;
using namespace TRTK;
using namespace TRTK::Tools;

using CameraModel = PinholeCameraModel<double>;
using value_type = CameraModel::value_type ;
using Point3D = CameraModel::Vector3T;
using Point2D = CameraModel::Vector2T;


template <typename... Args>
string strprintf (const char * fmt, Args... args)
{
    int resulting_size = snprintf(nullptr, 0, fmt, args...);
    vector<char> buffer(resulting_size + 1);
    snprintf(&buffer[0], buffer.size(), fmt, args...);
    return string(&buffer[0]);
}


struct GenerateTestData
{
    GenerateTestData(int number_of_points = 6, double sigma_world = 0, double sigma_display = 0)
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

        // Generate some test data.

        for (int i = 0; i < number_of_points; ++i) // generate points in the world COS
        {
            auto x = randn(0.0, 30.0);    // in the camera COS
            auto y = randn(0.0, 30.0);    // in the camera COS
            auto z = randn(600.0, 10.0);  // in the camera COS
            Point3D p = T.inverse() * Point3D(x, y, z); // in the world COS
            points_world.emplace_back(p);
        }

        for (auto const & p_W : points_world) // generate points in the camera COS; add noise if specified
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


template<class Matrix1, class Matrix2>
double computeRelativeError(const Matrix1 & M1, const Matrix2 & M2)
{
    assert(M1.rows() == M2.rows() && M1.cols() == M2.cols());

    double eps = numeric_limits<double>::epsilon();
    Matrix1 errors = (M1.array() - M2.array()) / (M2.array().abs() + eps);
    return errors.cwiseAbs2().sum() / M1.size();
}


double computeAngularDeviation(const Matrix3d & R1, const Matrix3d & R2)
{
    auto [axis1, rotation_angle1] = axisAngleFromRotationMatrix(R1);
    auto [__, axis1_theta, axis1_phi] = cartesian2Spherical(axis1.x(), axis1.y(), axis1.z());
    auto [axis2, rotation_angle2] = axisAngleFromRotationMatrix(R2);
    auto [___, axis2_theta, axis2_phi] = cartesian2Spherical(axis2.x(), axis2.y(), axis2.z());
    return abs(axis1_theta - axis2_theta) + abs(axis1_phi - axis2_phi) + abs(rotation_angle1 - rotation_angle2);
}


int main()
{
    try
    {
        ofstream file("data.csv");

        file << strprintf("%15s, ", "Number points")
             << strprintf("%15s, ", "Number trials")
             << strprintf("%15s, ", "Noise level")
             << strprintf("%15s, ", "RMSE")
             << strprintf("%15s, ", "RMSE (SD)")
             << strprintf("%30s, ", "Projection matrix error")
             << strprintf("%30s, ", "Projection matrix error (SD)")
             << strprintf("%15s, ", "Pose error")
             << strprintf("%15s, ", "Pose error (SD)")
             << strprintf("%30s, ", "Orientation error")
             << strprintf("%30s, ", "Orientation error (SD)")
             << strprintf("%50s, ", "Orientation error (sum of abs. angular diff.)")
             << strprintf("%55s, ", "Orientation error (sum of abs. angular diff.) (SD)")
             << strprintf("%30s, ", "Location error")
             << strprintf("%30s, ", "Location error (SD)")
             << strprintf("%30s, ", "Running time")
             << strprintf("%30s\n", "Running time (SD)");

        cout << "Pinhole camera model analysis" << endl;
        cout << "-----------------------------" << endl << endl;

        Clock clock;
        const int number_trials = 25;
        cout << setprecision(4);
        cout << fixed;

        for (auto & number_points : array<int, 9>{6, 10, 20, 30, 40, 50, 75, 100, 200})
        {
            cout << endl << "Number of points: " << number_points << endl << endl;

            for (auto & noise_level : array<double, 9>{0, 0.001, 0.01, 0.1, 0.5, 1, 2, 3, 5})
            {
                cout << "Noise level: " << noise_level << endl;

                srand(0);
                vector<double> rmses;
                vector<double> deviations_in_intrinsic_parameters;
                vector<double> deviations_in_extrinsic_parameters;
                vector<double> deviations_in_orientation;
                vector<double> deviations_in_orientation2;
                vector<double> deviations_in_location;
                vector<double> running_times;

                rmses.reserve(number_trials);
                deviations_in_intrinsic_parameters.reserve(number_trials);
                deviations_in_extrinsic_parameters.reserve(number_trials);
                deviations_in_orientation.reserve(number_trials);
                deviations_in_orientation2.reserve(number_trials);
                deviations_in_location.reserve(number_trials);
                running_times.reserve(number_trials);


                for (int i = 0; i < number_trials; ++i)
                {
                    auto test_data = GenerateTestData(number_points, noise_level);
                    PinholeCameraModel<double> pinhole_camera_model;
                    clock.reset();
                    double rmse = pinhole_camera_model.estimate(make_range(test_data.points_world), make_range(test_data.points_display));
                    // double rmse = pinhole_camera_model.estimateWithConstraints(make_range(test_data.points_world),
                    //                                                            make_range(test_data.points_display),
                    //                                                            PinholeCameraModel<double>::SAME_FOCAL_LENGTHS_AND_NO_SKEW);
                    running_times.push_back(clock.elapsed_time());
                    rmses.push_back(rmse);

                    // auto deviation_in_intrinsic_parameters = computeRelativeError(pinhole_camera_model.getIntrinsicParameters(), test_data.T_proj);
                    // deviations_in_intrinsic_parameters.push_back(deviation_in_intrinsic_parameters);
                    // auto deviation_in_extrinsic_parameters = computeRelativeError(pinhole_camera_model.getExtrinsicParameters(), test_data.T_pose);
                    // deviations_in_extrinsic_parameters.push_back(deviation_in_extrinsic_parameters);
                    // auto deviation_in_orientation = computeRelativeError(pinhole_camera_model.getCameraOrientation(), test_data.T_pose.block<3,3>(0,0));
                    // deviations_in_orientation.push_back(deviation_in_orientation);
                    // auto deviation_in_orientation2 = computeAngularDeviation(pinhole_camera_model.getCameraOrientation(), test_data.T_pose.block<3,3>(0,0));
                    // deviations_in_orientation2.push_back(deviation_in_orientation2);
                    // auto deviation_in_location = computeRelativeError(pinhole_camera_model.getCameraPosition(), test_data.T_pose.block<3,1>(0,3));
                    // deviations_in_location.push_back(deviation_in_location);

                    auto deviation_in_intrinsic_parameters = (pinhole_camera_model.getIntrinsicParameters() - test_data.T_proj).norm();
                    deviations_in_intrinsic_parameters.push_back(deviation_in_intrinsic_parameters);
                    auto deviation_in_extrinsic_parameters = (pinhole_camera_model.getExtrinsicParameters() - test_data.T_pose).norm();
                    deviations_in_extrinsic_parameters.push_back(deviation_in_extrinsic_parameters);
                    auto deviation_in_orientation = (pinhole_camera_model.getCameraOrientation() - test_data.T_pose.block<3,3>(0,0)).norm();
                    deviations_in_orientation.push_back(deviation_in_orientation);
                    auto deviation_in_orientation2 = computeAngularDeviation(pinhole_camera_model.getCameraOrientation(), test_data.T_pose.block<3,3>(0,0));
                    deviations_in_orientation2.push_back(deviation_in_orientation2);
                    auto deviation_in_location = (pinhole_camera_model.getCameraPosition() - test_data.T_pose.block<3,1>(0,3)).norm();
                    deviations_in_location.push_back(deviation_in_location);
                }

                double rmse = mean(rmses, 0.0);
                double rmse_sd = standardDeviation(rmses, 0.0);

                double projection_matrix_error = mean(deviations_in_intrinsic_parameters, 0.0);
                double projection_matrix_error_sd = standardDeviation(deviations_in_intrinsic_parameters, 0.0);

                double pose_error = mean(deviations_in_extrinsic_parameters, 0.0);
                double pose_error_sd = standardDeviation(deviations_in_extrinsic_parameters, 0.0);

                double orientation_error = mean(deviations_in_orientation, 0.0);
                double orientation_error_sd = standardDeviation(deviations_in_orientation, 0.0);

                double orientation_error2 = mean(deviations_in_orientation2, 0.0);
                double orientation_error2_sd = standardDeviation(deviations_in_orientation2, 0.0);

                double location_error = mean(deviations_in_location, 0.0);
                double location_error_sd = standardDeviation(deviations_in_location, 0.0);

                double running_time = mean(running_times, 0.0);
                double running_time_sd = standardDeviation(running_times, 0.0);

                cout << "RMSE: " << rmse << "  (" << rmse_sd << ")" << endl;
                cout << "Projection matrix error: " << projection_matrix_error << "  (" << projection_matrix_error_sd << ")" << endl;
                cout << "Pose error: " << pose_error << "  (" << pose_error_sd << ")" << endl;
                cout << "Orientation error: " << orientation_error << "  (" << orientation_error_sd << ")" << endl;
                cout << "Orientation error (sum of abs. angular diff.): " << orientation_error2 << "  (" << orientation_error2_sd << ")" << endl;
                cout << "Location error: " << location_error << "  (" << location_error_sd << ")" << endl << endl;
                cout << "Running time: " << running_time << "  (" << running_time_sd << ")" << endl << endl;

                file << strprintf("%15i, ", number_points)
                     << strprintf("%15i, ", number_trials)
                     << strprintf("%15f, ", noise_level)
                     << strprintf("%15f, ", rmse)
                     << strprintf("%15f, ", rmse_sd)
                     << strprintf("%30f, ", projection_matrix_error)
                     << strprintf("%30f, ", projection_matrix_error_sd)
                     << strprintf("%15f, ", pose_error)
                     << strprintf("%15f, ", pose_error_sd)
                     << strprintf("%30f, ", orientation_error)
                     << strprintf("%30f, ", orientation_error_sd)
                     << strprintf("%50f, ", orientation_error2)
                     << strprintf("%55f, ", orientation_error2_sd)
                     << strprintf("%30f, ", location_error)
                     << strprintf("%30f, ", location_error_sd)
                     << strprintf("%30f, ", running_time)
                     << strprintf("%30f\n", running_time_sd);
            }
        }
    }
    catch (exception & e)
    {
        cout << endl << "Error: " << e.what() << endl;
    }

    return 0;
}

