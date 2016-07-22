// Last changed on 2016-07-13.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <utility>
#include <vector>

#include<Eigen/StdVector>

#include <TRTK/Coordinate.hpp>
#include <TRTK/Tools.hpp>
#include <TRTK/Transform3D.hpp>
#include <TRTK/PivotCalibration.hpp>


using namespace std;
using namespace TRTK;
using namespace TRTK::Tools;

double pi = 3.1415926535;

typedef PivotCalibration<double> Calibration;
typedef Calibration::value_type value_type;
typedef Calibration::Matrix3T Matrix;
typedef Calibration::Vector3T Vector;


namespace
{

    struct GenerateTestData
    {
        GenerateTestData(double sigma = 0)
        {
            // Generate some (noisy) test data.

            /* We assume, there is a tool which is aligned with the x-axis. Its tool
             * tip is located at the (global) point p (pivot point), and the center of
             * the tool coordinate system is located at the point t. The local tool tip
             * position is defined to be (a, 0, 0).
             *
             * The relation between the local coordinate system and the global coordinate
             * system is given by p_global = R * p_local + t. The initial rotation R of
             * the tool coordinate system is a rotation of pi/2 in the x-y plane.
             *
             * Now the tool is rotated, where the tool tip remains at the same position.
             */

            Matrix R;
            R << cos(pi/2), -sin(pi/2), 0,
                 sin(pi/2),  cos(pi/2), 0,
                         0,          0, 1;

            double a = 70;
            p_local = Vector(a, 0, 0);
            p = Vector(1, 1, 1);
            Vector t = p - R * p_local;

            for (double theta = -0.8 * pi / 2; theta < 0.8 * pi / 2; theta += pi / 20)
            {
                for (double phi = -0.8 * pi / 2; phi < 0.8 * pi / 2; phi += pi / 20)
                {
                    // Rotate around pivot point P.

                    const double x = p.x();
                    const double y = p.y();
                    const double z = p.z();

                    Transform3D<double> transform;
                    transform.translate(-x, -y, -z).rotateZ(phi).rotateY(theta).translate(x, y, z);

                    Vector location = transform * t;
                    Matrix rotation = transform.getTransformationMatrix().block(0, 0, 3, 3) * R;

                    // Add some noise

                    using TRTK::Tools::randn;
                    location += sigma * Vector(randn(), randn(), randn());

                    // Store the results

                    locations.push_back(location);
                    rotations.push_back(rotation);
                }
            }
        }

        vector<Matrix> rotations;
        vector<Vector> locations;
        Vector p;
        Vector p_local;
    };

} // end of anonymous namespace


int main()
{
    cout << setprecision(4);
    cout << fixed;

    cout << "Pivot calibration example" << endl;
    cout << "-------------------------" << endl << endl;

    GenerateTestData test_data;
    cout << "Ground truth" << endl;
    cout << "Pivot point: " << test_data.p.transpose() << endl;
    cout << "Local pivot point: " << test_data.p_local.transpose() << endl << endl << endl;

    {
        srand(0);
        GenerateTestData test_data;

        PivotCalibrationTwoStep<double> calibration;
        calibration.setLocations(make_range(test_data.locations));
        calibration.setRotations(make_range(test_data.rotations));
        double rmse = calibration.compute();

        cout << "PivotCalibrationTwoStep (no noise)" << endl;
        cout << "RMSE: " << rmse << endl;
        cout << "Global pivot point: " << calibration.getPivotPoint().transpose() << "\t";
        cout << "Error: " << (calibration.getPivotPoint() - test_data.p).norm() << endl;
        cout << "Local pivot point: " << calibration.getLocalPivotPoint().transpose() << "\t";
        cout << "Error: " << (calibration.getLocalPivotPoint() - test_data.p_local).norm() << endl << endl;
    }

    {
        srand(0);
        GenerateTestData test_data(0.1);

        PivotCalibrationTwoStep<double> calibration;
        calibration.setLocations(make_range(test_data.locations));
        calibration.setRotations(make_range(test_data.rotations));
        double rmse = calibration.compute();

        cout << "PivotCalibrationTwoStep (with noise)" << endl;
        cout << "RMSE: " << rmse << endl;
        cout << "Global pivot point: " << calibration.getPivotPoint().transpose() << "\t";
        cout << "Error: " << (calibration.getPivotPoint() - test_data.p).norm() << endl;
        cout << "Local pivot point: " << calibration.getLocalPivotPoint().transpose() << "\t";
        cout << "Error: " << (calibration.getLocalPivotPoint() - test_data.p_local).norm() << endl << endl;
    }

    cout << endl;

    return 0;
}

