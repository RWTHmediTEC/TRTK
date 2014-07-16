// Last changed on 2012-08-03.

#include <cmath>

#include <TRTK/FitSphere.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_FitSphere()
{
    using namespace TRTK;
    using namespace TRTK::Tools;

    double pi = 3.1415926535;

    // Construct some points lying on a sphere with center point (1, 8, 5)
    // and a radius of 7.

    double radius = 7;
    Coordinate<double> centerPoint = Coordinate<double>(1, 8, 5);

    std::vector<Coordinate<double> > surface_points1;
    std::vector<Eigen::Vector3d> surface_points2;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > surface_points3;

    for (double theta = 0.0; theta < 2.0 * pi; theta += pi/10)
    {
        for (double phi = 0.0; phi < 2.0 * pi; phi += pi/10)
        {
            // convert spherical coordinates in cartesian coordinates

            double x = radius * std::sin(theta) * std::cos(phi);
            double y = radius * std::sin(theta) * std::sin(phi);
            double z = radius * std::cos(theta);

            // add noise

            x += randn<double>(0, 0.1);
            y += randn<double>(0, 0.1);
            z += randn<double>(0, 0.1);

            // create point

            Coordinate<double> point1 = centerPoint + Coordinate<double>(x, y, z);

            Eigen::Vector3d point2 = centerPoint.toArray().matrix() + Eigen::Vector3d(x, y, z);
            Eigen::Vector4d point3 = Eigen::Vector4d(centerPoint[0], centerPoint[1], centerPoint[2], 1) + Eigen::Vector4d(x, y, z, 1);

            surface_points1.push_back(point1);
            surface_points2.push_back(point2);
            surface_points3.push_back(point3);
        }
    }


    HEADING(FitSphere<T>)


    SUBHEADING(Constructors)

        START_TEST
            FitSphere<double> fitSphere;
            assert(isZero(fitSphere.getRadius()));
            assert(isZero(fitSphere.getCenterPoint().x()) &&
                   isZero(fitSphere.getCenterPoint().y()) &&
                   isZero(fitSphere.getCenterPoint().z()));
        STOP_TEST


        START_TEST
            FitSphere<double> fitSphere1(surface_points1); // std::vector<Coordinate<double> >
        STOP_TEST


        START_TEST
            FitSphere<double> fitSphere2(surface_points2); // std::vector<Eigen::Vector3d>
        STOP_TEST


        START_TEST
            FitSphere<double> fitSphere3(surface_points3); // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        STOP_TEST


    SUBHEADING(setPoints())

        START_TEST
            fitSphere.setPoints(surface_points1);
        STOP_TEST


        START_TEST
            fitSphere.setPoints(surface_points2);
        STOP_TEST


        START_TEST
            fitSphere.setPoints(surface_points3);
        STOP_TEST


    SUBHEADING(getCenterPoint()...)

        using std::abs;
        double residual = 0.0;

        START_TEST
            fitSphere.compute();
            residual = (fitSphere.getCenterPoint() - centerPoint).norm();
            assert( residual < 0.1);
        STOP_TEST


        START_TEST
            fitSphere1.compute();
            residual = (fitSphere1.getCenterPoint() - centerPoint).norm();
            assert( residual < 0.1);
        STOP_TEST


        START_TEST
            fitSphere2.compute();
            residual = (fitSphere2.getCenterPoint() - centerPoint).norm();
            assert( residual < 0.1);
        STOP_TEST


        START_TEST
            fitSphere3.compute();
            residual = (fitSphere3.getCenterPoint() - centerPoint).norm();
            assert( residual < 0.1);
        STOP_TEST


    SUBHEADING(getRadius())


        START_TEST
            residual = (fitSphere.getRadius() - radius);
            assert( residual < 0.02);
        STOP_TEST


        START_TEST
            residual = (fitSphere.getRadius() - radius);
            assert( residual < 0.02);
        STOP_TEST


        START_TEST
            residual = (fitSphere.getRadius() - radius);
            assert( residual < 0.02);
        STOP_TEST


        START_TEST
            residual = (fitSphere.getRadius() - radius);
            assert( residual < 0.02);
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            assert(fitSphere.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(fitSphere1.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(fitSphere2.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(fitSphere3.getRMS() < 0.2);
        STOP_TEST
}
