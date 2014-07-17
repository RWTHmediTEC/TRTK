// Last changed on 2012-01-25.

#include <cmath>

#include <TRTK/FitCircle3D.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_FitCircle3D()
{
    using namespace TRTK;
    using namespace TRTK::Tools;

    double pi = 3.1415926535;


    // Construct a circle in 3D space with center point (7, -15, 3) and a radius of 7.

    double radius = 7;
    Coordinate<double> centerPoint = Coordinate<double>(7, -15, 3);
    Coordinate<double> normal = Coordinate<double>(0, 0, 1);

    // Orientate the circle in space.

    Transform3D<double> R;
    R.rotateX(pi/3).rotateZ(pi/4).rotateY(pi/7);
    normal = R * normal;

    Transform3D<double> T;
    T.translate(centerPoint.x(), centerPoint.y(), centerPoint.z());


    // Create some points lying on the circle and add some noise.

    std::vector<Coordinate<double> > coordinate_points;
    std::vector<Eigen::Vector3d> vector3d_points;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > vector4d_points;

    for (double phi = 0.0; phi < 2.0 * pi; phi += pi/1000)
    {
        // convert spherical coordinates in cartesian coordinates

        double x = radius * std::cos(phi) + randn<double>(0, 0.1);
        double y = radius * std::sin(phi) + randn<double>(0, 0.1);
        double z = randn<double>(0, 0.1);

        Coordinate<double> coordinate_point = T * R * Coordinate<double>(x, y, z);

        Eigen::Vector3d vector3d_point = Eigen::Vector3d(coordinate_point.x(),
                                                         coordinate_point.y(),
                                                         coordinate_point.z());

        Eigen::Vector4d vector4d_point = Eigen::Vector4d(coordinate_point.x(),
                                                         coordinate_point.y(),
                                                         coordinate_point.z(),
                                                                            1);

        coordinate_points.push_back(coordinate_point);
        vector3d_points.push_back(vector3d_point);
        vector4d_points.push_back(vector4d_point);

    }


    HEADING(FitCircle3D<T>)


    SUBHEADING(Constructors)

        START_TEST
            FitCircle3D<double> fitCircle3D;

            assert(isZero(fitCircle3D.getRadius()));

            assert(isZero(fitCircle3D.getCenterPoint().x()) &&
                   isZero(fitCircle3D.getCenterPoint().y()) &&
                   isZero(fitCircle3D.getCenterPoint().z()));

            assert(isZero(fitCircle3D.getNormal().x()) &&
                   isZero(fitCircle3D.getNormal().y()) &&
                   isZero(fitCircle3D.getNormal().z()));
        STOP_TEST


        START_TEST
            FitCircle3D<double> coordinate_fitCircle3D(coordinate_points); // std::vector<Coordinate<double> >
        STOP_TEST


        START_TEST
            FitCircle3D<double> vector3d_fitCircle3D(vector3d_points); // std::vector<Eigen::Vector3d>
        STOP_TEST


        START_TEST
            FitCircle3D<double> vector4d_fitCircle3D(vector4d_points); // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        STOP_TEST


    SUBHEADING(setPoints())

        START_TEST
            fitCircle3D.setPoints(coordinate_points);
        STOP_TEST


        START_TEST
            fitCircle3D.setPoints(vector3d_points);
        STOP_TEST


        START_TEST
            fitCircle3D.setPoints(vector4d_points);
        STOP_TEST


    SUBHEADING(getCenterPoint())

        double residual = 0.0;

        START_TEST
            fitCircle3D.compute();
            residual = (fitCircle3D.getCenterPoint() - centerPoint).norm();
            assert( residual < 0.01);
        STOP_TEST


        START_TEST
            coordinate_fitCircle3D.compute();
            residual = (coordinate_fitCircle3D.getCenterPoint() - centerPoint).norm();
            assert( residual < 0.01);
        STOP_TEST


        START_TEST
            vector3d_fitCircle3D.compute();
            residual = (vector3d_fitCircle3D.getCenterPoint() - centerPoint).norm();
            assert( residual < 0.01);
        STOP_TEST


        START_TEST
            vector4d_fitCircle3D.compute();
            residual = (vector4d_fitCircle3D.getCenterPoint() - centerPoint).norm();
            assert( residual < 0.01);
        STOP_TEST


    SUBHEADING(getNormal())

        using std::abs;

        START_TEST
            residual = abs(abs(fitCircle3D.getNormal().dot(normal)) - 1);
            assert( residual < 10e-7);
        STOP_TEST


        START_TEST
            residual = abs(abs(coordinate_fitCircle3D.getNormal().dot(normal)) - 1);
            assert( residual < 10e-7);
        STOP_TEST


        START_TEST
            residual = abs(abs(vector3d_fitCircle3D.getNormal().dot(normal)) - 1);
            assert( residual < 10e-7);
        STOP_TEST


        START_TEST
            residual = abs(abs(vector4d_fitCircle3D.getNormal().dot(normal)) - 1);
            assert( residual < 10e-7);
        STOP_TEST


    SUBHEADING(getRadius())


        START_TEST
            residual = abs(fitCircle3D.getRadius() - radius);
            assert( residual < 10e-3);
        STOP_TEST


        START_TEST
            residual = abs(coordinate_fitCircle3D.getRadius() - radius);
            assert( residual < 10e-3);
        STOP_TEST


        START_TEST
            residual = abs(vector3d_fitCircle3D.getRadius() - radius);
            assert( residual < 10e-3);
        STOP_TEST


        START_TEST
            residual = abs(vector4d_fitCircle3D.getRadius() - radius);
            assert( residual < 10e-3);
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            assert(fitCircle3D.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(coordinate_fitCircle3D.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(vector3d_fitCircle3D.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(vector4d_fitCircle3D.getRMS() < 0.2);
        STOP_TEST

}
