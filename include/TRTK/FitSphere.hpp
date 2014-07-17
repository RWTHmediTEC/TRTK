/*
    Estimates the parameters of a 3D sphere from a set of points lying on
    the surface.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.3.0 (2012-01-25)
*/

/** \file FitSphere.hpp
  * \brief This file contains the \ref TRTK::FitSphere "FitSphere" class.
  */


#ifndef FITSPHERE_HPP_6184653510
#define FITSPHERE_HPP_6184653510


#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/SVD>
#include <vector>

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "Fit3D.hpp"


namespace TRTK
{


/** \tparam T scalar type of the coordinates
  *
  * \brief Estimates the parameters of a 3D sphere from a set of points lying on
  *        the surface.
  *
  * This class allows estimating the center point and the radius of a 3D sphere
  * whose surface points are known. The points may be erroneous since a least
  * square fitting is done.
  * If \f$ (x_i, y_i, z_i)^T \f$ are the data points, \f$ (x_0, y_0, z_0)^T \f$
  * the center point, and \f$ r \f$ the radius, then FitSphere tries to minimize
  * the sum of the squared residuals as shown below:
  *
  * \f[
  * (x_0, y_0, z_0, r) := \arg\min_{(x_0, y_0, z_0, r)}
  * \sum_{i=1}^N \left[(x_i - x_0)^2 + (y_i - y_0)^2 + (z_i - z_0)^2 - r^2 \right]^2
  * \f]
  *
  * Here is an more elaborate example to see how to use the class:
  *
  * \code
  * #include <cstdlib>
  * #include <iostream>
  *
  * #include <TRTK/FitSphere.hpp>
  * #include <TRTK/Tools.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
  *
  * int main()
  * {
  *     double pi = 3.1415926535;
  *
  *     vector<Coordinate<double> > surfacePoints;
  *
  *     // Construct some points lying on a sphere.
  *
  *     double radius = 7;
  *     Coordinate<double> centerPoint = Coordinate<double>(1, 8, 5);
  *
  *     for (double theta = 0.0; theta < 2.0 * pi; theta += pi/10)
  *     {
  *         for (double phi = 0.0; phi < 2.0 * pi; phi += pi/10)
  *         {
  *             // Convert the speherical coordinates into cartesian coordinates
  *             // and add some noise.
  *
  *             double x = radius * sin(theta) * cos(phi) + 0.1 * randn();
  *             double y = radius * sin(theta) * sin(phi) + 0.1 * randn();
  *             double z = radius * cos(theta) + 0.1 * randn();
  *
  *             Coordinate<double> point = centerPoint + Coordinate<double>(x, y, z);
  *
  *             surfacePoints.push_back(point);
  *         }
  *     }
  *
  *     // Estimate the parameters of the sphere.
  *
  *     FitSphere<double> fitSphere(surfacePoints);
  *
  *     fitSphere.compute();
  *
  *     cout << "Center point: " << fitSphere.getCenterPoint() << endl;
  *     cout << "Radius: " << fitSphere.getRadius() << endl;
  *
  *     return 0;
  * }
  *
  * \endcode
  *
  * Output:
  *
  * \code
  * Center point: (0.998629, 8.00457, 5.0021)
  * Radius: 7.00053
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.3.0
  * \date last changed on 2012-01-25
  */

template <class T>
class FitSphere : public Fit3D<T>
{
private:
    typedef Fit3D<T> super;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::MatrixXT MatrixXT;

    FitSphere();
    FitSphere(const std::vector<Coordinate<T> > & points);
    FitSphere(const std::vector<Vector3T> & points);
    FitSphere(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points);

    virtual ~FitSphere();

    void compute();

    const Coordinate<T> & getCenterPoint() const;
    T getDistanceTo(const Coordinate<T> & point) const;
    unsigned getNumberPointsRequired() const;
    T getRadius() const;
    T getRMS() const;

    enum Error {
        NOT_ENOUGH_POINTS,
        UNKNOWN_ERROR,
        WRONG_POINT_SIZE
    };

private:
    Coordinate<T> m_center_point;
    T m_radius;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs an empty FitSphere object.
  */

template <class T>
FitSphere<T>::FitSphere() :
    m_center_point(0, 0, 0),
    m_radius(0)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the surface of a sphere. The points can
  *                    be 3D or 4D homogeneous coordinates.
  *
  * \brief Constructs a FitSphere object.
  */

template <class T>
FitSphere<T>::FitSphere(const std::vector<Coordinate<T> > & points) :
    m_center_point(0, 0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the surface of a sphere.
  *
  * \brief Constructs a FitSphere object.
  */

template <class T>
FitSphere<T>::FitSphere(const std::vector<Vector3T> & points) :
    m_center_point(0, 0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the surface of a sphere. The points are
  *                    assumed to  be homogeneous coordinates, whose forth
  *                    coordinate entry is equal to one.
  *
  * \brief Constructs an FitSphere object.
  */

template <class T>
FitSphere<T>::FitSphere(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points) :
    m_center_point(0, 0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Destructs the FitSphere object.
  */

template <class T>
FitSphere<T>::~FitSphere()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Runs the fitting algorithm.
  *
  * There must be at least 4 surface points to perform the fitting.
  *
  * \throw ErrorObj If there are not enough points to fit the sphere, an error
  *                 object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  *
  * \see setPoints(), getCenterPoint() and getRadius()
  */

template <class T>
void FitSphere<T>::compute()
{
    /*

    Explanation of the algorithm:

    A point (x, y, z) on the surface of a sphere with center point (x0, y0, zo) and
    radius R fullfills the following equation:

    (x - x0)^2 + (y - y0)^2 + (z - z0)^2 = R^2

    This can be rewritten as

    2*x*x0 + 2*y*y0 + 2*z*z0 - x0^2 - y0^2 -z0^2 + R^2 = x^2 + y^2 + z^2

    or written as dot product

    [2x  2y  2z  1] * [x0  y0  z0  (R^2 - x0^2 - y0^2 - z0^2)]^T = x^2 + y^2 + z^2.

    Now we have an equation of the form

    A(i) * p = b(i).

    where

    A(i) = [2x  2y  2z  1]
    p    = [x0  y0  z0  (R^2 - x0^2 - y0^2 - z0^2)]^T
    b(i) = x^2 + y^2 + z^2.

    The index denotes, that this equation holds for a point (x(i), y(i), z(i)). Note
    that the vector p is constant for differing points of the same sphere. As you can
    see, the above equation is nothing else but a linear system. That is, in the
    above case A(i) is a matrix with one row only and p and b(i) are vectors, where
    b(i) also contains one row only. Since the above equation must hold for all
    points of a set of n points lying on the surface of a given sphere, we can
    construct a bigger matrix A and a vector b that still fullfill the above linear
    equation A * p = b.

    A := [A(1) ... A(n)]^T
    b := [b(1) ... b(n)]^T

    Now A is n times 4 matrix (which in general is non-square) and b a n times 1
    vector. As mentioned before, p is a constant vector wich contains the sought
    parameters. To solve the system, we just construct the pseudo inverse as follows:

    A * p = b

    ==> A^T * A * p = A^T * b           (now A^T * A is square!)

    ==> p = inverse(A^T * A) * A^T * b = pseudo_inverse(A) * b

    */

    if (super::m_points.cols() < 4)
    {
        ErrorObj error;
        error.setClassName("FitSphere<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to fit the sphere.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    // Construct matrix A.

    const int number_points =  super::m_points.cols();

    const VectorXT & x = super::m_points.row(0);
    const VectorXT & y = super::m_points.row(1);
    const VectorXT & z = super::m_points.row(2);

    MatrixXT A(number_points, 4);

    A.block(0, 0, number_points, 1) = 2 * x;
    A.block(0, 1, number_points, 1) = 2 * y;
    A.block(0, 2, number_points, 1) = 2 * z;
    A.block(0, 3, number_points, 1).setConstant(1);

    // Construct vector b.

    VectorXT b(number_points);

    b = x.array().square() + y.array().square() + z.array().square();

    // Compute pseudo inverse.

    MatrixXT pseudo_inverse = (A.transpose() * A).inverse() * A.transpose();

    // Compute vector p whose components are/contain the sought transformation entries.

    VectorXT p = pseudo_inverse * b; // FIXME: inverse(A^T * A) * (A^T * b) is more efficient to compute

    m_center_point[0] = p(0); // x0
    m_center_point[1] = p(1); // y0
    m_center_point[2] = p(2); // z0

    using std::sqrt;

    m_radius = sqrt(p(3) + p(0) * p(0) + p(1) * p(1) + p(2) * p(2)); // p(3) = R^2 - x0^2 - y0^2 - z0^2
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the center point.
  */

template <class T>
const Coordinate<T> & FitSphere<T>::getCenterPoint() const
{
    return m_center_point;
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] point   A 3D coordinate.
  *
  * \brief Returns the shortest distance from the given point to the sphere.
  */

template <class T>
T FitSphere<T>::getDistanceTo(const Coordinate<T> & point) const
{
    assert(point.size() == 3);

    using std::abs;
    return abs((point - m_center_point).norm() - m_radius);
}


template <class T>
inline unsigned FitSphere<T>::getNumberPointsRequired() const
{
    return 4;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the radius of the sphere.
  */

template <class T>
T FitSphere<T>::getRadius() const
{
    return m_radius;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the root mean square error.
  */

template <class T>
T FitSphere<T>::getRMS() const
{
    const int number_of_points = super::m_points.cols();

    T sum_of_squared_errors = 0.0;

    for (int i = 0; i < number_of_points; ++i)
    {
        // Compute the distance from the current point to the spehere and add it up.

        Coordinate<T> point(super::m_points.col(i));
        T distance = (point - m_center_point).norm() - m_radius;

        sum_of_squared_errors +=  distance * distance;
    }

    using std::sqrt;

    return sqrt(sum_of_squared_errors / number_of_points);
}


} // namespace TRTK


#endif // FITSPHERE_HPP_6184653510
