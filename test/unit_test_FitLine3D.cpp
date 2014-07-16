// Last changed on 2014-07-05.

#include <cmath>

#include <TRTK/FitLine3D.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_FitLine3D()
{
    using namespace TRTK;
    using namespace TRTK::Tools;
    using std::abs;
    using std::pow;
    using std::sqrt;

    typedef Coordinate<double> Coordinate;

    // Construct some points lying on a line.

    std::vector<Coordinate> coordinate_points;
    std::vector<Eigen::Vector3d> vector3d_points;
    std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > vector4d_points;

    double distance_from_origin = 4;

    Coordinate normal_vector = (Coordinate(2, -1, 3)).normalize();
    Coordinate direction_vector = normal_vector.orthogonal();
    Coordinate point_on_line = distance_from_origin * normal_vector;

    for (int i = 0; i < 100; ++i)
    {
        Coordinate point = point_on_line + rand(-10, 10) * direction_vector;

        // Add some noise.

        point.x() += 0.1 * randn();
        point.y() += 0.1 * randn();
        point.z() += 0.1 * randn();

        Eigen::Vector3d vector3d_point(point.toArray());
        Eigen::Vector4d vector4d_point((point, 1).toArray());

        coordinate_points.push_back(point);
        vector3d_points.push_back(vector3d_point);
        vector4d_points.push_back(vector4d_point);
    }


    HEADING(FitLine3D<T>)


    SUBHEADING(Constructors)

        START_TEST
            FitLine3D<double> fitLine3D;

            assert(isZero(fitLine3D.getDirectionVector().x()) &&
                   isZero(fitLine3D.getDirectionVector().y()) &&
                   isZero(fitLine3D.getDirectionVector().z()));

            assert(isZero(fitLine3D.getPointOnLineSegment().x()) &&
                   isZero(fitLine3D.getPointOnLineSegment().y()) &&
                   isZero(fitLine3D.getPointOnLineSegment().z()));
        STOP_TEST


        START_TEST
            FitLine3D<double> coordinate_fitLine3D(coordinate_points); // std::vector<Coordinate<double> >
        STOP_TEST


        START_TEST
            FitLine3D<double> vector3d_fitLine3D(vector3d_points); // std::vector<Eigen::Vector3d>
        STOP_TEST


        START_TEST
            FitLine3D<double> vector4d_fitLine3D(vector4d_points); // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >
        STOP_TEST


    SUBHEADING(setPoints())

        START_TEST
            fitLine3D.setPoints(coordinate_points);
        STOP_TEST


        START_TEST
            fitLine3D.setPoints(vector3d_points);
        STOP_TEST


        START_TEST
            fitLine3D.setPoints(vector4d_points);
        STOP_TEST


    SUBHEADING(getDirectionVector())

        double residual = 0.0;

        START_TEST
            coordinate_fitLine3D.compute();
            residual = (coordinate_fitLine3D.getDirectionVector().cross(direction_vector)).norm();
            assert(residual < 10e-3);
        STOP_TEST


        START_TEST
            vector3d_fitLine3D.compute();
            residual = (vector3d_fitLine3D.getDirectionVector().cross(direction_vector)).norm();
            assert(residual < 10e-3);
        STOP_TEST


        START_TEST
            vector4d_fitLine3D.compute();
            residual = (vector4d_fitLine3D.getDirectionVector().cross(direction_vector)).norm();
            assert(residual < 10e-3);
        STOP_TEST


    SUBHEADING(getNormalVector())


        START_TEST
            residual = (coordinate_fitLine3D.getNormalVector().cross(normal_vector)).norm();
            assert(residual < 10e-3);
        STOP_TEST


        START_TEST
            residual = abs(coordinate_fitLine3D.getNormalVector().norm() - 1);
            assert(residual < 10e-3);
        STOP_TEST


    SUBHEADING(getDistanceFromOrigin())


        START_TEST
            residual = abs(coordinate_fitLine3D.getDistanceFromOrigin() - distance_from_origin);
            assert(residual < 0.02);
        STOP_TEST


        START_TEST
            residual = abs(vector3d_fitLine3D.getDistanceFromOrigin() - distance_from_origin);
            assert(residual < 0.02);
        STOP_TEST


        START_TEST
            residual = abs(vector4d_fitLine3D.getDistanceFromOrigin() - distance_from_origin);
            assert(residual < 0.02);
        STOP_TEST


    SUBHEADING(getRMS())


        START_TEST
            assert(coordinate_fitLine3D.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(vector3d_fitLine3D.getRMS() < 0.2);
        STOP_TEST


        START_TEST
            assert(vector4d_fitLine3D.getRMS() < 0.2);
        STOP_TEST


    SUBHEADING(getPointOnLineSegment())


        coordinate_points.clear();
        coordinate_points.push_back(Coordinate(3, 0, 0));
        coordinate_points.push_back(Coordinate(4, 0, 0));

        coordinate_fitLine3D.setPoints(coordinate_points);
        coordinate_fitLine3D.compute();

        START_TEST
            point_on_line = coordinate_fitLine3D.getPointOnLineSegment();
            assert(3 <= point_on_line.x() && point_on_line.x() <= 4);
        STOP_TEST
}
