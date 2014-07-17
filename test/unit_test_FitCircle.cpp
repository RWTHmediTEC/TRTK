// Last changed on 2012-01-25.

#include <cmath>

#include <TRTK/FitCircle.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_FitCircle()
{
    using namespace TRTK;
    using namespace TRTK::Tools;

    double pi = 3.1415926535;

    // Construct some points lying on a circle with center point (5, 3)
    // and a radius of 7.

    double radius = 7;
    Coordinate<double> centerPoint = Coordinate<double>(5, 3);

    std::vector<Coordinate<double> > coordinate_points;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vector2d_points;
    std::vector<Eigen::Vector3d> vector3d_points;

    for (double phi = 0.0; phi < 2.0 * pi; phi += pi/10)
    {
        // convert spherical coordinates in cartesian coordinates

        double x = radius * std::cos(phi) + randn<double>(0,0.1);
        double y = radius * std::sin(phi) + randn<double>(0,0.1);

        Coordinate<double> coordinate_point = centerPoint + Coordinate<double>(x, y);
        Eigen::Vector2d vector2d_point = centerPoint.toArray().matrix() + Eigen::Vector2d(x, y);
        Eigen::Vector3d vector3d_point = Eigen::Vector3d(centerPoint[0], centerPoint[1], 1) + Eigen::Vector3d(x, y, 1);

        coordinate_points.push_back(coordinate_point);
        vector2d_points.push_back(vector2d_point);
        vector3d_points.push_back(vector3d_point);
    }


    HEADING(FitCircle<T>)


    SUBHEADING(Constructors)

        START_TEST
            FitCircle<double> fitCircle;
            assert(isZero(fitCircle.getRadius()));
            assert(isZero(fitCircle.getCenterPoint().x()) &&
                   isZero(fitCircle.getCenterPoint().y()));
        STOP_TEST


        START_TEST
            FitCircle<double> coordinate_fitCircle(coordinate_points); // std::vector<Coordinate<double> >
        STOP_TEST


        START_TEST
            FitCircle<double> vector2d_fitCircle(vector2d_points); // std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        STOP_TEST


        START_TEST
            FitCircle<double> vector3d_fitCircle(vector3d_points); // std::vector<Eigen::Vector3d>
        STOP_TEST


    SUBHEADING(setPoints())

        START_TEST
            fitCircle.setPoints(coordinate_points);
        STOP_TEST


        START_TEST
            fitCircle.setPoints(vector2d_points);
        STOP_TEST


        START_TEST
            fitCircle.setPoints(vector3d_points);
        STOP_TEST


    SUBHEADING(getCenterPoint())

        START_TEST
            fitCircle.compute();
            assert((fitCircle.getCenterPoint() - centerPoint).norm() < 0.1);
        STOP_TEST


        START_TEST
            coordinate_fitCircle.compute();
            assert((coordinate_fitCircle.getCenterPoint() - centerPoint).norm() < 0.1);
        STOP_TEST


        START_TEST
            vector2d_fitCircle.compute();
            assert((vector2d_fitCircle.getCenterPoint() - centerPoint).norm() < 0.1);
        STOP_TEST


        START_TEST
            vector3d_fitCircle.compute();
            assert((vector3d_fitCircle.getCenterPoint() - centerPoint).norm() < 0.1);
        STOP_TEST


    SUBHEADING(getRadius())


        START_TEST
            assert((fitCircle.getRadius() - radius) < 0.1);
        STOP_TEST


        START_TEST
            assert((coordinate_fitCircle.getRadius() - radius) < 0.1);
        STOP_TEST


        START_TEST
            assert((vector2d_fitCircle.getRadius() - radius) < 0.1);
        STOP_TEST


        START_TEST
            assert((vector3d_fitCircle.getRadius() - radius) < 0.1);
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            assert(fitCircle.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(coordinate_fitCircle.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(vector2d_fitCircle.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(vector3d_fitCircle.getRMS() < 0.2);
        STOP_TEST
}
