// Last changed on 2019-07-12.

#include <iostream>
#include <iomanip>
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

using Point3D = PinholeCameraModel<double>::Vector3T;
using Point2D = PinholeCameraModel<double>::Vector2T;


struct GenerateTestData
{
    GenerateTestData()
    {
        // Set the intrinsic camera parameters.

        camera_model.setFocalLengths(80, 80);
        camera_model.setImageCenter(250, 250);
        camera_model.setSkew(0);

        // Set the extrinsic camera parameters.

        Transform3D<double> T;
        T.rotateAxis(40, Point3D(1, 2, 3), Transform3D<double>::DEGREES);
        T.translate(130, 135, 90);
        camera_model.setExtrinsicParameters(T.getTransformationMatrix());

        // Generate some test data.

        const int number_of_points = 50;

        for (int i = 0; i < number_of_points; ++i) // generate points in the world COS
        {
            auto x = randn(0.0, 30.0);    // in the camera COS
            auto y = randn(0.0, 30.0);    // in the camera COS
            auto z = randn(600.0, 10.0);  // in the camera COS
            Point3D p = T.inverse() * Point3D(x, y, z);  // in the world COS
            points_world.emplace_back(p);
        }

        for (auto const & p_W : points_world) // generate points in the camera COS
        {
            Point2D p_D = camera_model.transform(p_W);
            points_display.push_back(p_D);
        }

        // Add noise to the points recorded in the world COS.

        for (auto & p_W : points_world)
        {
            double sigma_world = 0.5;
            p_W += Point3D(randn(0.0, sigma_world), randn(0.0, sigma_world), randn(0.0, sigma_world));
        }
    }

    PinholeCameraModel<double> camera_model;
    vector<Point3D> points_world;
    vector<Point2D> points_display; // image plane
};


int main()
{

    srand(0);
    GenerateTestData test_data;

    cout << setprecision(4);
    cout << fixed;

    cout << "Original values:" << endl << endl;
    cout << "Intrinsic camera parameters: " << endl << test_data.camera_model.getIntrinsicParameters() << endl << endl << endl;
    cout << "Extrinsic camera parameters: " << endl << test_data.camera_model.getExtrinsicParameters() << endl << endl;
    cout << "Camera parameters: " << endl << test_data.camera_model.getCameraParameters() << endl << endl;

    PinholeCameraModel<double> model;
    auto rmse = model.estimateWithConstraints(make_range(test_data.points_world),
                                              make_range(test_data.points_display),
                                              PinholeCameraModel<double>::SAME_FOCAL_LENGTHS_AND_NO_SKEW);

    cout << "Estimated values:" << endl << endl;
    cout << "RMSE: " << rmse << endl << endl;
    cout << "Intrinsic camera parameters: " << endl << model.getIntrinsicParameters() << endl << endl;
    cout << "Extrinsic camera parameters: " << endl << model.getExtrinsicParameters() << endl << endl;
    cout << "Camera parameters: " << endl << model.getCameraParameters() << endl << endl;

    return 0;
}
