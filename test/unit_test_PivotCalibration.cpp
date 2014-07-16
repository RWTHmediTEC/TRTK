// Last changed on 2012-02-05.


#include <cmath>

#include<Eigen/StdVector>

#include <TRTK/Coordinate.hpp>
#include <TRTK/Transform3D.hpp>
#include <TRTK/PivotCalibration.hpp>

#include "unit_test.hpp"


void unit_test_PivotCalibration()
{
    // Generate some test data.

    /* We assume, there is a tool which is aligned with the x-axis. Its tool
     * tip is located at the point P (pivot point), and the center of the tool
     * coordinate system is located at the point C. The difference vector showing
     * from P to C is (a, 0, 0). The initial rotation R of the tool coordinate
     * system is a rotation of pi/2 in the x-y plane.
     *
     * Now the tool is rotated, where the tool tip remains at the same position.
     */

    using namespace TRTK;

    double pi = 3.1415926535;

    typedef PivotCalibration<double> Calibration;
    typedef Calibration::value_type value_type;
    typedef Calibration::Matrix3T Matrix;
    typedef Calibration::Vector3T Vector;
    typedef Calibration::Vector4T Vector4T;

    const double a = 10.0;

    Vector P = Vector(1, 1, 1);
    Vector C = P + Vector(a, 0, 0);

    Matrix R;
    R << std::cos(pi/2), -std::sin(pi/2), 0,
         std::sin(pi/2),  std::cos(pi/2), 0,
                      0,               0, 1;

    std::vector<Matrix> rotations;
    std::vector<Vector> locations;
    std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > locations4T;
    std::vector<Coordinate<double> > locations_coordinate1;
    std::vector<Coordinate<double> > locations_coordinate2; // homogeneous coordinates
    std::vector<Transform3D<double>, Eigen::aligned_allocator<Transform3D<double> > > transformations;

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

            rotations.push_back(rotation);
            locations.push_back(location);

            locations4T.push_back(Vector4T(location.x(), location.y(), location.z(), 1));

            Coordinate<double> coordinate(location);
            locations_coordinate1.push_back(coordinate);
            locations_coordinate2.push_back((coordinate, 1)); // homogeneous coordinate

            Transform3D<double> transformation;
            transformation.getTransformationMatrix().block(0, 0, 3, 3) = rotation;
            transformation.getTransformationMatrix().block(0, 3, 3, 1) = location;

            transformations.push_back(transformation);
        }
    }


    HEADING(PivotCalibration<T>)


    SUBHEADING(Constructors)


        START_TEST
            Calibration pivotCalibration1;
            Calibration pivotCalibration2;
            Calibration pivotCalibration3;
            Calibration pivotCalibration4;
            Calibration pivotCalibration5;
        STOP_TEST


        START_TEST
            Calibration pivotCalibration6(rotations, locations);
        STOP_TEST


        START_TEST
            Calibration pivotCalibration7(transformations);
        STOP_TEST


    SUBHEADING(setLocations())


        // std::vector<Vector3T> & locations

        START_TEST
            pivotCalibration1.setLocations(locations);
        STOP_TEST


        // std::vector<Vector4T> & locations

        START_TEST
            pivotCalibration2.setLocations(locations4T);
        STOP_TEST


        // std::vector<Coordinate<double> > & locations

        START_TEST
            pivotCalibration3.setLocations(locations_coordinate1);
        STOP_TEST


        START_TEST
            pivotCalibration4.setLocations(locations_coordinate2);
        STOP_TEST


    SUBHEADING(setRotations())


        START_TEST
            pivotCalibration1.setRotations(rotations);
            pivotCalibration2.setRotations(rotations);
            pivotCalibration3.setRotations(rotations);
            pivotCalibration4.setRotations(rotations);
        STOP_TEST


    SUBHEADING(setTransformations())


        START_TEST
            pivotCalibration5.setTransformations(transformations);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            pivotCalibration1.compute();
        STOP_TEST


        START_TEST
            pivotCalibration2.compute();
        STOP_TEST


        START_TEST
            pivotCalibration3.compute();
        STOP_TEST


        START_TEST
            pivotCalibration4.compute();
        STOP_TEST


        START_TEST
            pivotCalibration5.compute();
        STOP_TEST


        START_TEST
            pivotCalibration6.compute();
        STOP_TEST


        START_TEST
            pivotCalibration7.compute();
        STOP_TEST


    SUBHEADING(getTranslation())


        START_TEST
            assert(pivotCalibration1.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST


        START_TEST
            assert(pivotCalibration2.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST


        START_TEST
            assert(pivotCalibration3.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST


        START_TEST
            assert(pivotCalibration4.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST


        START_TEST
            assert(pivotCalibration5.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST


        START_TEST
            assert(pivotCalibration6.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST


        START_TEST
            assert(pivotCalibration7.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST


    SUBHEADING(setAlgorithm())


        START_TEST
            pivotCalibration1.setAlgorithm(Calibration::TWO_STEP_PROCEDURE);
            pivotCalibration1.compute();
            assert(pivotCalibration1.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST


        START_TEST
            pivotCalibration1.setAlgorithm(Calibration::COMBINATORICAL_APPROACH);
            pivotCalibration1.compute();
            assert(pivotCalibration1.getTranslation().isApprox(Vector(0, 10, 0), 1e-9));
        STOP_TEST
}
