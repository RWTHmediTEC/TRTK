// Last changed on 2018-05-28.

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
        // Instantiate a pinhole camera model.

        PinholeCameraModel<double> camera_model;

        // Set the intrinsic camera parameters.

        camera_model.setFocalLengths(80, 79);
        camera_model.setImageCenter(250, 251);
        camera_model.setSkew(0.1);

        // Set the extrinsic camera parameters.

        Transform3D<double> T;
        T.rotateAxis(40, Point3D(1, 2, 3), Transform3D<double>::DEGREES);
        T.translate(130, 135, 90);
        camera_model.setExtrinsicParameters(T.getTransformationMatrix());

        // Generate some test data.

        const int number_of_points = 6;

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
    }

    vector<Point3D> points_world;
    vector<Point2D> points_display; // image plane
};


int main()
{

    srand(0);
    GenerateTestData test_data;

    PinholeCameraModel<double> model;
    auto rmse = model.estimate(make_range(test_data.points_world), make_range(test_data.points_display));

    cout << setprecision(4);
    cout << fixed;
    cout << "RMSE: " << rmse << endl << endl;
    cout << "Extrinsic camera parameters: " << endl << model.getExtrinsicParameters() << endl << endl;
    cout << "Intrinsic camera parameters: " << endl << model.getIntrinsicParameters() << endl;

    return 0;
}
