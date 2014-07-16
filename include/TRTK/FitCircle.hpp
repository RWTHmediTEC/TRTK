/*
    Estimates the parameters of a circle from a set of 2D points lying on
    the circle.

    Copyright (C) 2010 - 2014 Fabian Killus, Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.0 (2012-03-20)
*/

/** \file FitCircle.hpp
  * \brief This file contains the \ref TRTK::FitCircle "FitCircle" class.
  */


#ifndef FIT_CIRCLE_HPP_1950385693
#define FIT_CIRCLE_HPP_1950385693


#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/SVD>
#include <vector>

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "Fit2D.hpp"


namespace TRTK
{


/** \tparam T scalar type of the coordinates
  *
  * \brief Estimates the parameters of a circle from a set of 2D points lying on the
  *        circle.
  *
  * This class allows estimating the center point and the radius of a circle from
  * known points on the circle. The points may be erroneous since a least
  * square fitting is done.
  * If \f$ (x_i, y_i)^T \f$ are the data points, \f$ (x_0, y_0)^T \f$
  * the center point, and \f$ r \f$ the radius, then FitCircle tries to minimize
  * the sum of the squared residuals as shown below:
  *
  * \f[
  * (x_0, y_0, r) := \arg\min_{(x_0, y_0, r)}
  * \sum_{i=1}^N \left[(x_i - x_0)^2 + (y_i - y_0)^2 - r^2 \right]^2
  * \f]
  *
  * Here is a more elaborate example to see how to use the class:
  *
  * \code
  * #include <cstdlib>
  * #include <iostream>
  *
  * #include <TRTK/FitCircle.hpp>
  * #include <TRTK/Tools.hpp>
  *
  * using namespace TRTK;
  *
  * int main()
  * {
  *     double pi = 3.1415926535;
  *
  *     std::vector<Coordinate<double> > circlePoints;
  *
  *     // Construct some points lying on a circle.
  *
  *     double radius = 7;
  *     Coordinate<double> centerPoint = Coordinate<double>(4, 2);
  *
  *     for (double phi = 0.0; phi < 2.0 * pi; phi += pi/10)
  *     {
  *         using std::sin;
  *         using std::cos;
  *         using Tools::randn;
  *
  *         double x = radius * cos(phi) + randn(0.0, 0.1);
  *         double y = radius * sin(phi) + randn(0.0, 0.1);
  *
  *         Coordinate<double> point = centerPoint + Coordinate<double>(x, y);
  *
  *         circlePoints.push_back(point);
  *     }
  *
  *     // Estimate the circle parameters.
  *
  *     FitCircle<double> fitCircle(circlePoints);
  *
  *     fitCircle.compute();
  *
  *     std::cout << "Center point: " << fitCircle.getCenterPoint() << std::endl;
  *     std::cout << "Radius: " << fitCircle.getRadius() << std::endl;
  *
  *     return 0;
  * }
  *
  * \endcode
  *
  * Output:
  *
  * \code
  * Center point: (4.03589, 1.97525)
  * Radius: 7.03509
  * \endcode
  *
  * \author Fabian Killus, Christoph Haenisch
  * \version 0.2.0
  * \date last changed on 2012-03-20
  */

template <class T>
class FitCircle : public Fit2D<T>
{
private:
    typedef Fit2D<T> super;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::Vector2T Vector2T;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::MatrixXT MatrixXT;

    FitCircle();
    FitCircle(const std::vector<Coordinate<T> > & points);
    FitCircle(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & points);
    FitCircle(const std::vector<Vector3T> & points);

    virtual ~FitCircle();

    void compute();

    unsigned getNumberPointsRequired() const;

    const Coordinate<T> & getCenterPoint() const;
    T getDistanceTo(const Coordinate<T> & point) const;
    T getRadius() const;
    T getRMS() const;

    using super::NOT_ENOUGH_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_POINT_SIZE;

private:
    Coordinate<T> m_center_point;
    T m_radius;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs an empty FitCircle object.
  */

template <class T>
FitCircle<T>::FitCircle() :
    m_center_point(0, 0),
    m_radius(0)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle. The points can
  *                    be 2D or 3D homogeneous coordinates.
  *
  * \brief Constructs a FitCircle object.
  */

template <class T>
FitCircle<T>::FitCircle(const std::vector<Coordinate<T> > & points) :
    m_center_point(0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle.
  *
  * \brief Constructs a FitCircle object.
  */

template <class T>
FitCircle<T>::FitCircle(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & points) :
    m_center_point(0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle. The points are
  *                    assumed to  be homogeneous coordinates, whose third
  *                    coordinate entry is equal to one.
  *
  * \brief Constructs a FitCircle object.
  */

template <class T>
FitCircle<T>::FitCircle(const std::vector<Vector3T> & points) :
    m_center_point(0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Destructs the FitCircle object.
  */

template <class T>
FitCircle<T>::~FitCircle()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Estimates the parameters of the circle.
  *
  * There must be at least 3 points to perform the fitting.
  *
  * \throw ErrorObj If there are not enough points to fit the circle, an error
  *                 object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  *
  * \see setPoints(), getCenterPoint() and getRadius()
  */

template <class T>
void FitCircle<T>::compute()
{
    /*

    Explanation of the algorithm:

    A point (x, y) on the circle with center point (x0, y0) and radius R
    fullfills the following equation:

    (x - x0)^2 + (y - y0)^2 = R^2

    This can be rewritten as

    2*x*x0 + 2*y*y0 - x0^2 - y0^2 + R^2 = x^2 + y^2

    or written as dot product

    [2x  2y  1] * [x0  y0  (R^2 - x0^2 - y0^2)]^T = x^2 + y^2

    Now we have an equation of the form

    A(i) * p = b(i).

    where

    A(i) = [2x  2y  1]
    p    = [x0  y0  (R^2 - x0^2 - y0^2)]^T
    b(i) = x^2 + y^2

    The index denotes, that this equation holds for a point (x(i), y(i)). Note
    that the vector p is constant for differing points on the same circle. As you can
    see, the above equation is nothing else but a linear system. That is, in the
    above case A(i) is a matrix with one row only and p is a vector. Since the above
    equation must hold for all points of a set of n points lying on the given circle,
    we can construct a bigger matrix A and a vector b that still fullfill the above
    linear equation A * p = b.

    A := [A(1) ... A(n)]^T
    b := [b(1) ... b(n)]^T

    Now, A is a n times 3 matrix (which in general is non-square) and b a n times 1
    vector. As mentioned before, p is a constant vector wich contains the sought
    parameters. To solve the system, we just construct the pseudo inverse as follows:

    A * p = b

    ==> A^T * A * p = A^T * b           (now A^T * A is square!)

    ==> p = inverse(A^T * A) * A^T * b = pseudo_inverse(A) * b

    */

    if (super::m_points.cols() < 3)
    {
        ErrorObj error;
        error.setClassName("FitCircle<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to fit the circle.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    // Construct matrix A.

    const int number_points =  super::m_points.cols();

    const VectorXT & x = super::m_points.row(0);
    const VectorXT & y = super::m_points.row(1);

    MatrixXT A(number_points, 3);

    A.block(0, 0, number_points, 1) = 2 * x;
    A.block(0, 1, number_points, 1) = 2 * y;
    A.block(0, 2, number_points, 1).setConstant(1);

    // Construct vector b.

    VectorXT b(number_points);

    b = x.array().square() + y.array().square();

    // Compute pseudo inverse.

    MatrixXT pseudo_inverse = (A.transpose() * A).inverse() * A.transpose();

    // Compute vector p whose components are/contain the sought transformation entries.

    VectorXT p = pseudo_inverse * b; // FIXME: inverse(A^T * A) * (A^T * b) is more efficient to compute

    m_center_point[0] = p(0); // x0
    m_center_point[1] = p(1); // y0

    using std::sqrt;
    m_radius = sqrt(p(2) + p(0) * p(0) + p(1) * p(1)); // p(2) = R^2 - x0^2 - y0^2
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the center point of the plane.
  */

template <class T>
const Coordinate<T> & FitCircle<T>::getCenterPoint() const
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
T FitCircle<T>::getDistanceTo(const Coordinate<T> & point) const
{
    assert(point.size() == 2);

    using std::abs;
    return abs((point - m_center_point).norm() - m_radius);
}


template <class T>
inline unsigned FitCircle<T>::getNumberPointsRequired() const
{
    return 3;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the root mean square error.
  */

template <class T>
T FitCircle<T>::getRMS() const
{
    using std::sqrt;

    const int number_points = super::m_points.cols();

    T sum_of_squared_errors = 0.0;

    for (int i = 0; i < number_points; ++i)
    {
        // Compute the distance from the current point to the circle and add it up.

        Coordinate<T> point(super::m_points.col(i));
        T distance = m_radius - (point - m_center_point).norm(); // Omit abs() due to subsequent squaring.

        sum_of_squared_errors +=  distance * distance;
    }

    return sqrt(sum_of_squared_errors / number_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the radius of the circle.
  */

template <class T>
T FitCircle<T>::getRadius() const
{
    return m_radius;
}


} // namespace TRTK


#endif // FIT_CIRCLE_HPP_1950385693
