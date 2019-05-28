// Last changed on 2018-05-17.

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

double pi = 3.1415926535;

using CameraModel = PinholeCameraModel<double>;
using value_type = CameraModel::value_type ;
using Point3D = CameraModel::Vector3T;
using Point2D = CameraModel::Vector2T;


// Note, PinholeCameraModel<T>::transform() and PinholeCameraModel<T>::operator() were
// manually checked and seem to works correctly (either way, these are currently only
// one-liners). Thus, we will make use of them and assume their correctness.


namespace
{

    struct GenerateTestData
    {
        GenerateTestData(double sigma = 0)
        {
            // Instantiate a pinhole camera model.

            CameraModel camera_model;
            camera_model.setFocalLengths(10, 10);

            cout << "Intrinsic camera parameters: " << endl << camera_model.getIntrinsicParameters() << endl;
            cout << "Extrinsic camera parameters: " << endl << camera_model.getExtrinsicParameters() << endl;

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
    };


    struct GenerateTestData2
    {
        GenerateTestData2(double sigma = 0)
        {
            // Instantiate a pinhole camera model.

            CameraModel camera_model;

            // Set the intrinsic camera parameters.

            camera_model.setFocalLengths(80, 80);
            camera_model.setImageCenter(0, 0);
            camera_model.setSkew(0);

            // Set the extrinsic camera parameters.

            Transform3D<double> T;
            T.rotateAxis(0, Point3D(1, 1, 1), Transform3D<double>::DEGREES);
            T.translate(130, 135, 90);
            // camera_model.setExtrinsicParameters(T.getTransformationMatrix());

            cout << "Intrinsic camera parameters: " << endl << camera_model.getIntrinsicParameters() << endl;
            cout << "Extrinsic camera parameters: " << endl << camera_model.getExtrinsicParameters() << endl;

            // Generate some test data.

            const int number_of_points = 12;

            for (int i = 0; i < number_of_points; ++i) // points in the world COS
            {
                auto x = randn(0.0, 2.0);
                auto y = randn(50.0, 2.0);
                auto z = randn(600.0, 10.0);
                points_world.emplace_back(Point3D(x, y, z));
            }

            for (auto const & p_W : points_world) // points in the display COS
            {
                Point2D p_D = camera_model.transform(p_W) + Point2D(randn(0.0, sigma), randn(0.0, sigma));
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
    };

} // end of anonymous namespace


int main()
{
    cout << setprecision(4);
    cout << fixed;

    // Matrix3d A = (Matrix3d() << 1, 2, 3, 0, 4, 5, 0, 0, 6).finished();
    // Matrix3d Q, R;
    // tie(Q, R) = computeRQDecomposition<double>(A);
    // cout << "A:" << endl << A << endl << endl;
    // cout << "Q:" << endl << Q << endl << endl;
    // cout << "R:" << endl << R << endl << endl;
    // cout << "R * Q:" << endl << R * Q << endl << endl;


    cout << "Pinhole camera model example" << endl;
    cout << "----------------------------" << endl << endl;

    {
        srand(0);
        GenerateTestData2 test_data(0.1);

        CameraModel model;
        auto rmse = model.estimate(make_range(test_data.points_world), make_range(test_data.points_display));

        cout << "No noise" << endl;
        cout << "RMSE: " << rmse << endl;
        cout << "Extrinsic camera parameters: " << endl << model.getExtrinsicParameters() << endl;
        cout << "Intrinsic camera parameters: " << endl << model.getIntrinsicParameters() << endl;
    }

    cout << endl;

    return 0;
}

