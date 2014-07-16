// Last changed on 2012-07-23.

#include <cmath>

#include <TRTK/FitLine.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_FitLine()
{
    using namespace TRTK;
    using namespace TRTK::Tools;

    // Construct a line with slope 0.3 and center point (-10, 3).

    std::vector<Coordinate<double> > coordinate_points;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vector2d_points;
    std::vector<Eigen::Vector3d> vector3d_points;

    // Properties:

    Coordinate<double> center_point(-10, 3);

    double length = 10;
    double slope = 0.3;
    double y_intercept = center_point.y() - center_point.x() * slope; // = 6
    double x_intercept = - y_intercept / slope; // = -20
    double distance_from_origin = abs(y_intercept) / sqrt( 1 + slope * slope); // = 5.5

    // Construct some points lying on the line

    for (int i = 0; i <= 1000; ++i)
    {
        double x = center_point.x() + rand(-length / 2.0, length / 2.0);
        double y = center_point.y() + slope * (x - center_point.x());

        // add noise
        x += randn(0.0, 0.1); // 0, 0.1
        y += randn(0.0, 0.1); // 0, 0.1

        Coordinate<double> coordinate_point(x, y);
        Eigen::Vector2d vector2d_point(x, y);
        Eigen::Vector3d vector3d_point(x, y, 1);

        coordinate_points.push_back(coordinate_point);
        vector2d_points.push_back(vector2d_point);
        vector3d_points.push_back(vector3d_point);
    }


    HEADING(FitLine<T>)


    SUBHEADING(Constructors)

        START_TEST
            FitLine<double> fitLine;
            assert(isZero(fitLine.getSlope()));
            assert(isZero(fitLine.getYIntercept()));
            assert(isZero(fitLine.getXIntercept()));
        STOP_TEST


        START_TEST
            FitLine<double> coordinate_fitLine(coordinate_points); // std::vector<Coordinate<double> >
        STOP_TEST


        START_TEST
            FitLine<double> vector2d_fitLine(vector2d_points); // std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >
        STOP_TEST


        START_TEST
            FitLine<double> vector3d_fitLine(vector3d_points); // std::vector<Eigen::Vector3d>
        STOP_TEST


    SUBHEADING(setPoints())

        START_TEST
            fitLine.setPoints(coordinate_points);
        STOP_TEST


        START_TEST
            fitLine.setPoints(vector2d_points);
        STOP_TEST


        START_TEST
            fitLine.setPoints(vector3d_points);
        STOP_TEST


    SUBHEADING(getSlope()...)

        using std::abs;
        double residual = 0.0;

        START_TEST
            fitLine.compute();
            residual = abs(fitLine.getSlope() - slope);
            assert( residual < 0.003);
        STOP_TEST


        START_TEST
            coordinate_fitLine.compute();
            residual = abs(coordinate_fitLine.getSlope() - slope);
            assert(residual < 0.003);
        STOP_TEST


        START_TEST
            vector2d_fitLine.compute();
            residual = abs(vector2d_fitLine.getSlope() - slope);
            assert(residual < 0.003);
        STOP_TEST


        START_TEST
            vector3d_fitLine.compute();
            residual = abs(vector3d_fitLine.getSlope() - slope);
            assert(residual < 0.003);
        STOP_TEST


    SUBHEADING(getYIntercept())


        START_TEST
            residual = abs(fitLine.getYIntercept() - y_intercept);
            assert( residual < 0.03);
        STOP_TEST


        START_TEST
            residual = abs(coordinate_fitLine.getYIntercept() - y_intercept);
            assert( residual < 0.03);
        STOP_TEST


        START_TEST
            residual = abs(vector2d_fitLine.getYIntercept() - y_intercept);
            assert( residual < 0.03);
        STOP_TEST


        START_TEST
            residual = abs(vector3d_fitLine.getYIntercept() - y_intercept);
            assert( residual < 0.03);
        STOP_TEST


    SUBHEADING(getXIntercept())


        START_TEST
            residual = abs(fitLine.getXIntercept() - x_intercept);
            assert( residual < 0.1);
        STOP_TEST


        START_TEST
            residual = abs(coordinate_fitLine.getXIntercept() - x_intercept);
            assert(residual < 0.1);
        STOP_TEST


        START_TEST
            residual = abs(vector2d_fitLine.getXIntercept() - x_intercept);
            assert(residual < 0.1);
        STOP_TEST


        START_TEST
            residual = abs(vector3d_fitLine.getXIntercept() - x_intercept);
            assert(residual < 0.1);
        STOP_TEST


    SUBHEADING(getDistanceFromOrigin())


        START_TEST
            residual = abs(fitLine.getDistanceFromOrigin() - distance_from_origin);
            assert(residual < 0.1);
        STOP_TEST


        START_TEST
            residual = abs(coordinate_fitLine.getDistanceFromOrigin() - distance_from_origin);
            assert(residual < 0.1);
        STOP_TEST


        START_TEST
            residual = abs(vector2d_fitLine.getDistanceFromOrigin() - distance_from_origin);
            assert(residual < 0.1);
        STOP_TEST

        START_TEST
            residual = abs(vector3d_fitLine.getDistanceFromOrigin() - distance_from_origin);
            assert(residual < 0.1);
        STOP_TEST


    SUBHEADING(getPointOnLineSegment())


        START_TEST
            residual = (fitLine.getPointOnLineSegment() - center_point).norm();
            assert(residual < 0.25);
        STOP_TEST


        START_TEST
            residual = (coordinate_fitLine.getPointOnLineSegment() - center_point).norm();
            assert(residual < 0.25);
        STOP_TEST


        START_TEST
            residual = (vector2d_fitLine.getPointOnLineSegment() - center_point).norm();
            assert(residual < 0.25);
        STOP_TEST


        START_TEST
            residual = (vector3d_fitLine.getPointOnLineSegment() - center_point).norm();
            assert(residual < 0.25);
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            assert(fitLine.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(coordinate_fitLine.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(vector2d_fitLine.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(vector3d_fitLine.getRMS() < 0.2);
        STOP_TEST


    SUBHEADING(Line parallel to x-axis)


        std::vector<Coordinate<double> > points(10);

        for (unsigned i = 0; i < points.size(); ++i)
        {
            double x = rand(-10.0, 10.0);
            double y = 5.0;

            points[i] = Coordinate<double>(x, y);
        }

        fitLine.setPoints(points);
        fitLine.compute();


        START_TEST
            residual = abs(fitLine.getDirectionVector().dot(Coordinate<double>(0, 1)));
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getNormalVector().dot(Coordinate<double>(1, 0)));
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs((fitLine.getPointOnLineSegment() - Coordinate<double>(0, 5)).dot(fitLine.getNormalVector()));
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getRMS() - 0.0);
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getSlope() - 0.0);
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getYIntercept() - 5.0);
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getDistanceFromOrigin() - 5.0);
            assert(residual < 0.00001);
        STOP_TEST


    SUBHEADING(Line parallel to y-axis)


        for (unsigned i = 0; i < points.size(); ++i)
        {
            double x = 5.0;
            double y = rand(-10.0, 10.0);

            points[i] = Coordinate<double>(x, y);
        }

        fitLine.setPoints(points);
        fitLine.compute();


        START_TEST
            residual = abs(fitLine.getDirectionVector().dot(Coordinate<double>(1, 0)));
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getNormalVector().dot(Coordinate<double>(0, 1)));
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs((fitLine.getPointOnLineSegment() - Coordinate<double>(5, 0)).dot(fitLine.getNormalVector()));
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getRMS() - 0.0);
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            assert(fitLine.getSlope() == std::numeric_limits<double>::infinity());
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getXIntercept() - 5.0);
            assert(residual < 0.00001);
        STOP_TEST


        START_TEST
            residual = abs(fitLine.getDistanceFromOrigin() - 5.0);
            assert(residual < 0.00001);
        STOP_TEST
}
