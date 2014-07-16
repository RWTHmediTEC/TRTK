/*
    Estimates the parameters of a circle with center in the origin from a set
    of 2D points lying on the circle.

    Copyright (C) 2010 - 2014 Fabian Killus, Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.0 (2012-03-20)
*/

/** \file FitCircleInOrigin.hpp
  * \brief This file contains the \ref TRTK::FitCircleInOrigin "FitCircleInOrigin" class.
  */


#ifndef FIT_CIRCLE_IN_ORIGIN_HPP_9483758184
#define FIT_CIRCLE_IN_ORIGIN_HPP_9483758184


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
  * \brief Estimates the radius of a circle centered at the origin from a
  *        set of 2D points.
  *
  * This class allows estimating the radius of a circle from known points on the
  * circle. The points may be erroneous since a least square fitting is done.
  *
  * Here is an more elaborate example to see how to use the class:
  *
  * \code
  * #include <iostream>
  *
  * #include <TRTK/FitCircleInOrigin.hpp>
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
  *     //Construct some points lying on a circle.
  *
  *     double radius = 7;
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
  *         Coordinate<double> point(x, y);
  *         circlePoints.push_back(point);
  *     }
  *
  *     // Estimate the circle parameters.
  *
  *     FitCircleInOrigin<double> fitCircleInOrigin(circlePoints);
  *     fitCircleInOrigin.compute();
  *
  *     std::cout << "Radius: " << fitCircleInOrigin.getRadius() << std::endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Radius: 7.03625
  * \endcode
  *
  * \author Fabian Killus, Christoph Haenisch
  * \version 0.2.0
  * \date last changed on 2012-03-20
  */

template <class T>
class FitCircleInOrigin : public Fit2D<T>
{
private:
    typedef Fit2D<T> super;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::Vector2T Vector2T;
    typedef typename super::Vector3T Vector3T;

    FitCircleInOrigin();
    FitCircleInOrigin(const std::vector<Coordinate<T> > & points);
    FitCircleInOrigin(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & points);
    FitCircleInOrigin(const std::vector<Vector3T> & points);

    virtual ~FitCircleInOrigin();

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
  * \brief Constructs an empty FitCircleInOrigin object.
  */

template <class T>
FitCircleInOrigin<T>::FitCircleInOrigin() :
    m_center_point(0, 0),
    m_radius(0)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle. The points can
  *                    be 2D or 3D homogeneous coordinates.
  *
  * \brief Constructs a FitCircleInOrigin object.
  */

template <class T>
FitCircleInOrigin<T>::FitCircleInOrigin(const std::vector<Coordinate<T> > & points) :
    m_center_point(0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle.
  *
  * \brief Constructs a FitCircleInOrigin object.
  */

template <class T>
FitCircleInOrigin<T>::FitCircleInOrigin(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & points) :
    m_center_point(0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the circle. The points are
  *                    assumed to  be homogeneous coordinates, whose
  *                    third coordinate entry is equal to one.
  *
  * \brief Constructs an FitCircleInOrigin object.
  */

template <class T>
FitCircleInOrigin<T>::FitCircleInOrigin(const std::vector<Vector3T> & points) :
    m_center_point(0, 0),
    m_radius(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Destructs the FitCircleInOrigin object.
  */

template <class T>
FitCircleInOrigin<T>::~FitCircleInOrigin()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Runs the fitting algorithm.
  *
  * \throw ErrorObj If there are no points to fit the circle, an error
  *                 object is thrown and its error code is set to
  *                 \c NOT_ENOUGH_POINTS.
  *
  * \see setPoints() and getRadius()
  */

template <class T>
void FitCircleInOrigin<T>::compute()
{
    /*

    Explanation of the algorithm:

    A point (x, y) on the circle with radius R fullfills the following
    equation:

    R = sqrt(x^2 + y^2) = |(x, y)| = norm((x, y))

    Because there are possibly more than just one known point on the circle,
    regard the mean value as the best fit for R.

    */

    if (super::m_points.cols() == 0)
    {
        ErrorObj error;
        error.setClassName("FitCircleInOrigin<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to fit the circle.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }


    // Calculate radius of the circle.

    m_radius = super::m_points.colwise().norm().mean();
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the center point of the plane.
  */

template <class T>
const Coordinate<T> & FitCircleInOrigin<T>::getCenterPoint() const
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
T FitCircleInOrigin<T>::getDistanceTo(const Coordinate<T> & point) const
{
    assert(point.size() == 2);

    using std::abs;
    return abs(point.norm() - m_radius);
}


template <class T>
inline unsigned FitCircleInOrigin<T>::getNumberPointsRequired() const
{
    return 1;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the radius of the circle.
  */

template <class T>
T FitCircleInOrigin<T>::getRadius() const
{
    return m_radius;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the root mean square error.
  */

template <class T>
T FitCircleInOrigin<T>::getRMS() const
{
    using std::sqrt;

    const int number_points = super::m_points.cols();

    T sum_of_squared_errors = 0.0;

    for (int i = 0; i < number_points; ++i)
    {
        T error = m_radius - super::m_points.col(i).norm();
        sum_of_squared_errors +=  error * error;
    }

    return sqrt(sum_of_squared_errors / number_points);
}


} // namespace TRTK


#endif // FIT_CIRCLE_IN_ORIGIN_HPP_9483758184
