// Last changed on 2012-01-25.

#include <cmath>
#include <TRTK/FitPlane.hpp>
#include <TRTK/Tools.hpp>
#include <TRTK/Coordinate.hpp>

#include "unit_test.hpp"


void unit_test_FitPlane()
{

    using std::abs;
    using TRTK::FitPlane;
    using TRTK::Tools::isEqual;
    using TRTK::Tools::isZero;
    using TRTK::Tools::rand;
    using TRTK::Tools::randn;

    typedef TRTK::Coordinate<double> Coordinate;

    std::vector<Coordinate> points_coordinate;
    std::vector<Eigen::Vector3d> points_vector3d;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_vector4d;

    // Define a plane.

    Coordinate point_in_plane(13, 4, 9);
    Coordinate v1(3, 2, 4);
    Coordinate v2(1, 3, 5);
    Coordinate normal = v1.cross(v2).normalize();

    // Construct some points lying on the plane.

    for (int i = 0; i < 100; ++i)
    {
        Coordinate point = point_in_plane + rand(-10, 10) * v1 + rand(-10, 10) * v2;
        point += 0.1 * randn() * normal;

        points_coordinate.push_back(point);
        points_vector3d.push_back(point.toArray().matrix());
        points_vector4d.push_back((point, 1).toArray().matrix());
    }

    // Determine the shortest distance to the origin

    double distance_to_origin = abs(point_in_plane.dot(normal));


    HEADING(FitPlane<T>)


    SUBHEADING(Constructors)

        START_TEST
            FitPlane<double> fitPlane;
            assert(isZero(fitPlane.getNormal().norm()));
            assert(isZero(fitPlane.getPointInPlane().norm()));
        STOP_TEST


        START_TEST
            FitPlane<double> fitPlane_coordinate(points_coordinate); // std::vector<Coordinate>
        STOP_TEST


        START_TEST
            FitPlane<double> fitPlane_vector3d(points_vector3d); // std::vector<Eigen::Vector3d>
        STOP_TEST


        START_TEST
            FitPlane<double> fitPlane_vector4d(points_vector4d); // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        STOP_TEST


    SUBHEADING(setPoints())

        START_TEST
            fitPlane.setPoints(points_coordinate);
        STOP_TEST


        START_TEST
            fitPlane.setPoints(points_vector3d);
        STOP_TEST


        START_TEST
            fitPlane.setPoints(points_vector4d);
        STOP_TEST


    SUBHEADING(getNormal())


        double residual = 0.0;


        START_TEST
            fitPlane_coordinate.compute();
            residual = fitPlane_coordinate.getNormal().cross(normal).norm();
            assert(residual < 0.1);
        STOP_TEST


        START_TEST
            fitPlane_vector3d.compute();
            residual = fitPlane_vector3d.getNormal().cross(normal).norm();
            assert(residual < 0.1);
        STOP_TEST


        START_TEST
            fitPlane_vector4d.compute();
            residual = fitPlane_vector4d.getNormal().cross(normal).norm();
            assert(residual < 0.1);
        STOP_TEST


    SUBHEADING(getDistanceFromOrigin())


        START_TEST
            residual = abs(fitPlane_coordinate.getDistanceFromOrigin() - distance_to_origin);
            assert(residual < 0.1);
        STOP_TEST


        START_TEST
            residual = abs(fitPlane_vector3d.getDistanceFromOrigin() - distance_to_origin);
            assert(residual < 0.1);
        STOP_TEST


        START_TEST
            residual = abs(fitPlane_vector4d.getDistanceFromOrigin() - distance_to_origin);
            assert(residual < 0.1);
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            assert(fitPlane_coordinate.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(fitPlane_vector3d.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(fitPlane_vector4d.getRMS() < 0.2);
        STOP_TEST


    SUBHEADING(getPointInPlane())


        // Define a x-y-plane.

        point_in_plane = Coordinate(0, 0, 3);
        v1 = Coordinate(1, 0, 0);
        v2 = Coordinate(0, 1, 0);

        // Construct some points lying on the plane.

        points_coordinate.clear();

        for (int i = 0; i < 100; ++i)
        {
            points_coordinate.push_back(point_in_plane +
                                        rand(-10, 10) * v1 +
                                        rand(-10, 10) * v2);
        }

        fitPlane_coordinate.setPoints(points_coordinate);
        fitPlane_coordinate.compute();

        START_TEST
            assert(-10.0 <= fitPlane_coordinate.getPointInPlane().x() &&
                    10.0 >  fitPlane_coordinate.getPointInPlane().x() &&
                   -10.0 <= fitPlane_coordinate.getPointInPlane().y() &&
                    10.0 >  fitPlane_coordinate.getPointInPlane().y() );
        STOP_TEST
}
