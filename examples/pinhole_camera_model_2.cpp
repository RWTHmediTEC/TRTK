// Last changed on 2018-05-28.

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <tuple>
#include <utility>
#include <vector>

#include<Eigen/Core>
#include<Eigen/StdVector>

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


namespace
{

    struct GenerateTestData
    {
        GenerateTestData(int number_of_points = 6, double sigma_world = 0, double sigma_display = 0)
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

} // end of anonymous namespace


int main()
{
    try
    {
        cout << "Pinhole camera model analysis" << endl;
        cout << "-----------------------------" << endl << endl;

        const int number_trials = 25;
        cout << setprecision(4);
        cout << fixed;

        for (auto & number_points : array<int, 6>{6, 12, 24, 50, 100, 200})
        {
            cout << endl << "Number of points: " << number_points << endl << endl;

            for (auto & noise_level : array<double, 7>{0, 0.001, 0.01, 0.1, 1, 2, 5})
            {
                cout << "Noise level: " << noise_level << endl;

                srand(0);
                vector<double> rmses;

                for (int i = 0; i < number_trials; ++i)
                {
                    auto test_data = GenerateTestData(number_points, noise_level);
                    PinholeCameraModel<double> pinhole_camera_model;
                    double rmse = pinhole_camera_model.estimate(make_range(test_data.points_world), make_range(test_data.points_display));
                    rmses.push_back(rmse);
                }

                double rmse_mean = mean(rmses, 0.0);
                double rmse_sd = standardDeviation(rmses, 0.0);

                cout << "RMSE: " << rmse_mean << "  (" << rmse_sd << ")" << endl << endl;
            }
        }
    }
    catch (exception & e)
    {
        cout << endl << "Error: " << e.what() << endl;
    }

    return 0;
}

