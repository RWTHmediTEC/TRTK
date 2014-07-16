/*
    Estimates the parameters of a circle in 3D space from a set of points lying on
    the circle.

    Copyright (C) 2010 - 2014 Christoph Haenisch, Fabian Killus

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.0 (2012-03-20)
*/

/** \file FitCircle3D.hpp
  * \brief This file contains the \ref TRTK::FitCircle3D "FitCircle3D" class.
  */


#ifndef FIT_CIRCLE_3D_HPP_6647389212
#define FIT_CIRCLE_3D_HPP_6647389212


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/SVD>

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "Fit3D.hpp"
#include "FitPlane.hpp"
#include "FitCircle.hpp"


namespace TRTK
{


/** \tparam T scalar type of the coordinates
  *
  * \brief Estimates the parameters of a circle in 3D space from a set of
  *        points lying on the circle.
  *
  * This class allows estimating the center point, the radius and the normal
  * vector of a circle in 3D space from known points on the circle.
  *
  * Here is an more elaborate example to see how to use the class:
  *
  * \code
  * #include <iostream>
  *
  * #include <TRTK/FitCircle3D.hpp>
  * #include <TRTK/Tools.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
  *
  * int main()
  * {
  *     typedef Coordinate<double> Coordinate;
  *
  *     double pi = 3.1415926535;
  *
  *     double radius = 10;
  *     Coordinate center_point = Coordinate(2, -1, 0);
  *
  *     // Create some noisy points lying on a circle in the x-y-plane.
  *
  *     vector<Coordinate> points;
  *
  *     for (double phi = 0.0; phi < 2.0 * pi; phi += 0.1)
  *     {
  *         double x = radius * cos(phi) + 0.1 * randn();
  *         double y = radius * sin(phi) + 0.1 * randn();
  *
  *         points.push_back(Coordinate(x, y, 0));
  *     }
  *
  *     // Rotate the circle and move it to the above center point.
  *
  *     Transform3D<double> transform;
  *
  *     transform.rotateY(pi/2).rotateX(pi/2).translate(center_point);
  *
  *     for (unsigned i = 0; i < points.size(); ++i)
  *     {
  *         points[i] = transform * points[i];
  *     }
  *
  *     // Estimate the circle's parameters.
  *
  *     FitCircle3D<double> fitCircle3D(points);
  *     fitCircle3D.compute();
  *
  *     cout << "Center point: " << fitCircle3D.getCenterPoint() << endl;
  *     cout << "Normal:       " << fitCircle3D.getNormal() << endl;
  *     cout << "Radius:       " << fitCircle3D.getRadius() << endl;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Center point: (2, -0.94518, -0.0189328)
  * Normal:       (-1, 4.50982e-011, 5.1715e-014)
  * Radius:       9.99913
  * \endcode
  *
  * \author Christoph Haenisch, Fabian Killus
  * \version 0.2.0
  * \date last changed on 2012-03-20
  */

template <class T>
class FitCircle3D : public Fit3D<T>
{
private:
    typedef Fit3D<T> super;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::Vector2T Vector2T;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::Matrix3T Matrix3T;

    FitCircle3D();
    FitCircle3D(const std::vector<Coordinate<T> > & points);
    FitCircle3D(const std::vector<Vector3T> & points);
    FitCircle3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points);

    virtual ~FitCircle3D();

    void compute();

    const Coordinate<T> & getCenterPoint() const;
    T getDistanceTo(const Coordinate<T> & point) const;
    const Coordinate<T> & getNormal() const;
    unsigned getNumberPointsRequired() const;
    T getRadius() const;
    T getRMS() const;

    using super::NOT_ENOUGH_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_POINT_SIZE;

private:
    T m_radius;
    Coordinate<T> m_center_point;
    Coordinate<T> m_normal;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs an empty FitCircle3D object.
  */

template <class T>
FitCircle3D<T>::FitCircle3D() :
    m_radius(0),
    m_center_point(0, 0, 0),
    m_normal(0, 0, 0)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle. The points can
  *                    be 3D or 4D homogeneous coordinates.
  *
  * \brief Constructs a FitCircle3D object.
  */

template <class T>
FitCircle3D<T>::FitCircle3D(const std::vector<Coordinate<T> > & points) :
    m_radius(0),
    m_center_point(0, 0, 0),
    m_normal(0, 0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle.
  *
  * \brief Constructs a FitCircle3D object.
  */

template <class T>
FitCircle3D<T>::FitCircle3D(const std::vector<Vector3T> & points) :
    m_radius(0),
    m_center_point(0, 0, 0),
    m_normal(0, 0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle. The points are
  *                    assumed to  be homogeneous coordinates, whose
  *                    fourth coordinate entry is equal to one.
  *
  * \brief Constructs an FitCircle3D object.
  */

template <class T>
FitCircle3D<T>::FitCircle3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points) :
    m_radius(0),
    m_center_point(0, 0, 0),
    m_normal(0, 0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Destructs the FitCircle3D object.
  */

template <class T>
FitCircle3D<T>::~FitCircle3D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Runs the fitting algorithm.
  *
  * There must be at least 3 known points on the circle to perform the fitting.
  *
  * \throw ErrorObj If there are not enough points to fit the circle, an error
  *                 object is thrown and its error code is set to
  *                 \c NOT_ENOUGH_POINTS.
  *
  * \see setPoints(), getCenterPoint() and getRadius()
  */

template <class T>
void FitCircle3D<T>::compute()
{
    /*

    Explanation of the algorithm:

    1) First, the plane in which the circle lies is estimated from the given set of
       points. This yields the normal vector of the plane.

    2) Then the above plane is rotated such that its normal corresponds to the z-axis.

    3) As a result, all points lie in an x-y-plane where z = const. (Now, the last
       component can be just dropped.)

    4) The circle within the x-y-plane is estimated and its center point is transformed
       (inverse rotation) to the original position. For this purpose we need the
       z-coordinate which is the interception of the plane with the z-axis.

    */

    using namespace std;

    const int number_of_points =  super::m_points.cols();

    if (number_of_points < 3)
    {
        ErrorObj error;
        error.setClassName("FitCircle3D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to fit the circle.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    // Estimate the plane the circle lies in.

    vector<Coordinate<T> > points(number_of_points);

    for (int i = 0; i < number_of_points; ++i)
    {
        Vector3T point = super::m_points.col(i);
        points[i] = Coordinate<T>(point);
    }

    FitPlane<T> fitPlane(points);
    fitPlane.compute();

    m_normal = fitPlane.getNormal();

    // Rotate the original point set such that the normal of the
    // plane corresponds to the z-axis. Then estimate the 2D
    // circle from the projection of the points to the x-y-plane
    // (just drop the last component).

    Matrix3T R; // rotation matrix

    Coordinate<T> row3 = m_normal;
    Coordinate<T> row2 = row3.orthonormal();
    Coordinate<T> row1 = row2.cross(row3);

    R << row1.x(), row1.y(), row1.z(),
         row2.x(), row2.y(), row2.z(),
         row3.x(), row3.y(), row3.z();

    for (int i = 0; i < number_of_points; ++i)
    {
        Vector3T point = super::m_points.col(i);
        Vector2T projected_point = (R * point).head(2);
        points[i] = Coordinate<T>(projected_point); // overwrite above point set
    }

    // interception of the rotated plane with the z-axis

    T interception_z_axis = (R * fitPlane.getPointInPlane().toArray().matrix()).z();

    // Estimate the parameters of the 2D circle.

    FitCircle<T> fitCircle(points);
    fitCircle.compute();

    m_radius = fitCircle.getRadius();

    // Invers-transform the center point of the 2D circle to obtain
    // the center point of the circle in the original 3D point set.

    Coordinate<T> center_point = (fitCircle.getCenterPoint(), interception_z_axis); // yields a 3D coordinate

    m_center_point = Coordinate<T>(R.inverse() * center_point.toArray().matrix());

}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the center point of the circle.
  */

template <class T>
const Coordinate<T> & FitCircle3D<T>::getCenterPoint() const
{
    return m_center_point;
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] point   A 2D coordinate.
  *
  * \brief Returns the shortest distance from the circle to the given point.
  */

template <class T>
T FitCircle3D<T>::getDistanceTo(const Coordinate<T> & point) const
{
    assert(point.size() == 3);

    using std::sqrt;

    // Vector pointing from the center point to the current point.

    Coordinate<T> difference_vector = point - m_center_point;

    // Shortest distance from the current point to the plane the
    // circle lies in (value can be negative, but it is squared
    // anyway).

    T distance_to_plane = difference_vector.dot(m_normal);

    // Distance from the (onto the plane) projected point to the circle.
    // (The original point, the projected point, and the center point
    // form a right angle triangle which allows using the Pythagorean
    // theorem).

    T op_distance_to_circle = sqrt(difference_vector.norm() * difference_vector.norm() -
                                    distance_to_plane * distance_to_plane) - m_radius;

    // Now, the Pythagorean theorem can be used once again to compute
    // the sought distance.

    T distance = sqrt(distance_to_plane * distance_to_plane +
                      op_distance_to_circle * op_distance_to_circle);

    return distance;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the normal of the plane the circle lies in.
  */

template <class T>
const Coordinate<T> & FitCircle3D<T>::getNormal() const
{
    return m_normal;
}


template <class T>
inline unsigned FitCircle3D<T>::getNumberPointsRequired() const
{
    return 3;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the radius of the circle.
  */

template <class T>
T FitCircle3D<T>::getRadius() const
{
    return m_radius;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the root mean square error.
  */

template <class T>
T FitCircle3D<T>::getRMS() const
{
    // The error between a point from the given point set and the
    // estimated model is defined to be the shortest distance
    // between that point and the circle.

    using std::sqrt;

    const int number_of_points = super::m_points.cols();

    T sum_of_squared_errors = 0.0;

    for (int i = 0; i < number_of_points; ++i)
    {
        // Vector pointing from the center point to the current point.

        Coordinate<T> difference_vector = Coordinate<T>(super::m_points.col(i)) - m_center_point;

        // Shortest distance from the current point to the plane the
        // circle lies in (value can be negative, but it is squared
        // anyway).

        T distance_to_plane = difference_vector.dot(m_normal);

        // Distance from the (onto the plane) projected point to the circle.
        // (The original point, the projected point, and the center point
        // form a right angle triangle which allows using the Pythagorean
        // theorem).

        T op_distance_to_circle = sqrt(difference_vector.norm() * difference_vector.norm() -
                                       distance_to_plane * distance_to_plane) - m_radius;

        // Now, the Pythagorean theorem can be used once again to compute
        // the sought distance.

        // T distance = sqrt(distance_to_plane * distance_to_plane +
        //                   op_distance_to_circle * op_distance_to_circle);

        sum_of_squared_errors += distance_to_plane * distance_to_plane +
                                 op_distance_to_circle * op_distance_to_circle;
    }

    return sqrt(sum_of_squared_errors / number_of_points);
}


} // namespace TRTK


#endif // FIT_CIRCLE_3D_HPP_6647389212
