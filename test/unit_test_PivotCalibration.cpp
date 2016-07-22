// Last changed on 2016-07-03.


#include <cmath>
#include <cstdlib>
#include <utility>
#include <vector>

#include<Eigen/StdVector>

#include <TRTK/Coordinate.hpp>
#include <TRTK/Tools.hpp>
#include <TRTK/Transform3D.hpp>
#include <TRTK/PivotCalibration.hpp>
#include <TRTK/RansacPivotCalibrationModel.hpp>

#include "unit_test.hpp"


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
         * tip is located at the point P (pivot point), and the center of the tool
         * coordinate system is located at the point C. The difference vector showing
         * from P to C is (a, 0, 0). The initial rotation R of the tool coordinate
         * system is a rotation of pi/2 in the x-y plane.
         *
         * Now the tool is rotated, where the tool tip remains at the same position.
         */

        const double a = 10.0;

        Vector P = Vector(1, 1, 1);
        Vector C = P + Vector(a, 0, 0);

        Matrix R;
        R << std::cos(pi/2), -std::sin(pi/2), 0,
             std::sin(pi/2),  std::cos(pi/2), 0,
                            0,               0, 1;

        for (double theta = -0.8 * pi / 2; theta < 0.8 * pi / 2; theta += pi / 20)
        {
            for (double phi = -0.8 * pi / 2; phi < 0.8 * pi / 2; phi += pi / 20)
            {
                // Rotate around pivot point P.

                const double x = P.x();
                const double y = P.y();
                const double z = P.z();

                Transform3D<double> transform;
                transform.translate(-x, -y, -z).rotateZ(phi).rotateY(theta).translate(x, y, z);

                Vector location = transform * C;
                Matrix rotation = transform.getTransformationMatrix().block(0, 0, 3, 3)  * R;

                // Add some noise

                using TRTK::Tools::randn;
                location += sigma * Vector(randn(), randn(), randn());

                // Store the results

                locations.push_back(location);
                rotations.push_back(rotation);
            }
        }
    }

    std::vector<Matrix> rotations;
    std::vector<Vector> locations;
};


} // end of anonymous namespace


void unit_test_PivotCalibration()
{
    HEADING(PivotCalibration<T>)


    SUBHEADING(PivotCalibrationTwoStep)


        START_TEST
        {
            GenerateTestData test_data;
            PivotCalibrationTwoStep<double> calibration;
            calibration.setLocations(make_range(test_data.locations));
            calibration.setRotations(make_range(test_data.rotations));
            double rmse = calibration.compute();
            assert(calibration.getLocalPivotPoint().isApprox(Vector(0, 10, 0), 1e-9));
        }
        STOP_TEST


        START_TEST
        {
            srand(0);
            GenerateTestData test_data(0.1);
            PivotCalibrationTwoStep<double> calibration;
            calibration.setLocations(make_range(test_data.locations));
            calibration.setRotations(make_range(test_data.rotations));
            double rmse = calibration.compute();
            assert(calibration.getLocalPivotPoint().isApprox(Vector(0, 10, 0), 1e-1));
        }
        STOP_TEST


        SUBHEADING(RansacPivotCalibrationModel)


        START_TEST
        {
            srand(0);
            GenerateTestData test_data(0.1);
            std::vector<std::pair<Vector, Matrix> > data = zip(test_data.locations, test_data.rotations);

            PivotCalibrationTwoStep<double> calibration;
            RansacPivotCalibrationModel<double> model(calibration);
            Ransac<double, PivotCalibration<double>::DataType> ransac;

            ransac.setModel(model);
            ransac.setData(data);
            ransac.setErrorTolerance(0.15);
            ransac.setMinimumSetSize(40);
            // ransac.setAlgorithm(Ransac<double, PivotCalibration<double>::DataType>::RANSAC_SMALLEST_RMSE);

            unsigned number_of_samples_used = ransac.compute();
            assert(calibration.getLocalPivotPoint().isApprox(Vector(0, 10, 0), 1e-1));
        }
        STOP_TEST
}
