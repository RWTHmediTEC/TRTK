// Last changed on 2012-01-25.

#include <cmath>

#include <TRTK/FitCircleInOrigin.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_FitCircleInOrigin()
{
    using namespace TRTK;
    using namespace TRTK::Tools;

    double pi = 3.1415926535;

    // Construct some points lying on a circle with center point in the origin
    // and radius of 7.

    double radius = 7;

    std::vector<Coordinate<double> > coordinate_points;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vector2d_points;
    std::vector<Eigen::Vector3d> vector3d_points;

    for (double phi = 0.0; phi < 2.0 * pi; phi += pi/1000)
    {
        // convert spherical coordinates in cartesian coordinates
        // and add some noise.

        double x = radius * std::cos(phi) + randn<double>(0, 0.1);
        double y = radius * std::sin(phi) + randn<double>(0, 0.1);

        Coordinate<double> coordinate_point = Coordinate<double>(x, y);
        Eigen::Vector2d vector2d_point = Eigen::Vector2d(x, y);
        Eigen::Vector3d vector3d_point = Eigen::Vector3d(x, y, 1);

        coordinate_points.push_back(coordinate_point);
        vector2d_points.push_back(vector2d_point);
        vector3d_points.push_back(vector3d_point);
    }


    HEADING(FitCircleInOrigin<T>)


    SUBHEADING(Constructors)

        START_TEST
            FitCircleInOrigin<double> fitCircleInOrigin;
            assert(isZero(fitCircleInOrigin.getRadius()));
            assert(isZero(fitCircleInOrigin.getCenterPoint().x()) &&
                   isZero(fitCircleInOrigin.getCenterPoint().y()));
        STOP_TEST


        START_TEST
            FitCircleInOrigin<double> coordinate_fitCircleInOrigin(coordinate_points); // std::vector<Coordinate<double> >
        STOP_TEST


        START_TEST
            FitCircleInOrigin<double> vector2d_fitCircleInOrigin(vector2d_points); // std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        STOP_TEST


        START_TEST
            FitCircleInOrigin<double> vector3d_fitCircleInOrigin(vector3d_points); // std::vector<Eigen::Vector3d>
        STOP_TEST


    SUBHEADING(setPoints())

        START_TEST
            fitCircleInOrigin.setPoints(coordinate_points);
        STOP_TEST


        START_TEST
            fitCircleInOrigin.setPoints(vector2d_points);
        STOP_TEST


        START_TEST
            fitCircleInOrigin.setPoints(vector3d_points);
        STOP_TEST


    SUBHEADING(getCenterPoint())

        START_TEST
            fitCircleInOrigin.compute();
            assert(isZero(fitCircleInOrigin.getCenterPoint().norm()));
        STOP_TEST


        START_TEST
            coordinate_fitCircleInOrigin.compute();
            assert(isZero(coordinate_fitCircleInOrigin.getCenterPoint().norm()));
        STOP_TEST


        START_TEST
            vector2d_fitCircleInOrigin.compute();
            assert(isZero(vector2d_fitCircleInOrigin.getCenterPoint().norm()));
        STOP_TEST


        START_TEST
            vector3d_fitCircleInOrigin.compute();
            assert(isZero(vector3d_fitCircleInOrigin.getCenterPoint().norm()));
        STOP_TEST


    SUBHEADING(getRadius())

        using std::abs;
        double residual = 0.0;


        START_TEST
            residual = abs(fitCircleInOrigin.getRadius() - radius);
            assert( residual < 0.01);
        STOP_TEST


        START_TEST
            residual = abs(coordinate_fitCircleInOrigin.getRadius() - radius);
            assert( residual < 0.01);
        STOP_TEST


        START_TEST
            residual = abs(vector2d_fitCircleInOrigin.getRadius() - radius);
            assert( residual < 0.01);
        STOP_TEST


        START_TEST
            residual = abs(vector3d_fitCircleInOrigin.getRadius() - radius);
            assert( residual < 0.01);
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            assert( fitCircleInOrigin.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert( coordinate_fitCircleInOrigin.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert( vector2d_fitCircleInOrigin.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert( vector3d_fitCircleInOrigin.getRMS() < 0.2);
        STOP_TEST
}
