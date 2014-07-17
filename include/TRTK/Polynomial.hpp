/*
    Estimation and computation of a polynomials.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 1.0.0 (2013-04-05)
*/

/** \file Polynomial.hpp
  * \brief This file contains the interface for all polynomials.
  */


#ifndef POLYNOMIAL_HPP_7886646936
#define POLYNOMIAL_HPP_7886646936


#include <cassert>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "Iterator.hpp"
#include "Range.hpp"


namespace TRTK
{


/** \class Polynomial
  *
  * \brief Interface class for all polynomials.
  *
  * The polynomial classes provide means to estimate and compute univariate or
  * multivariate polynomials of various degree.
  *
  *
  * \see Coordinate
  *
  * \author Christoph Haenisch
  * \version 1.0.0
  * \date last changed on 2013-04-05
  */

template <class T>
class Polynomial
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Error {DIVISION_BY_ZERO,
                INVALID_ARGUMENT,
                NOT_ENOUGH_POINTS,
                UNEQUAL_NUMBER_OF_POINTS,
                UNKNOWN_ERROR,
                WRONG_COORDINATE_SIZE};

    typedef T value_type;
    typedef Coordinate<T> Point;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXT;

    /*
    Polynomial() {};

    Polynomial(const MatrixXT &) {};

    template <class U>
    Polynomial(const Polynomial<U> &) {};

    virtual ~Polynomial() {};
    */

    virtual MatrixXT & getCoefficients() = 0;                               ///< Returns the internal coefficients in form of a matrix representation.
    virtual const MatrixXT & getCoefficients() const = 0;                   ///< Returns the internal coefficients in form of a matrix representation.

    virtual T estimate(Range<Point> source_points,
                       Range<Point> target_points,
                       Range<T> weights = Range<T>()) = 0;                  ///< Estimates the polynomial function from two corresponding point sets; points can be weighted.

    virtual T estimate(Iterator<Point> source_points_first,
                       Iterator<Point> source_points_last,
                       Iterator<Point> target_points_first,
                       Iterator<Point> target_points_last,
                       Iterator<T> weights_first = Iterator<T>(),
                       Iterator<T> weights_last = Iterator<T>()) = 0;       ///< Estimates the polynomial function from two corresponding point sets; points can be weighted.

    virtual Point operator*(const Point &) const = 0;                       ///< Transforms a point with the internally saved transformation.
    virtual Point transform(const Point &) const = 0;                       ///< Transforms a point with the internally saved transformation.

    virtual Polynomial<T> & reset() = 0;                                    ///< Sets the internal coefficients such that the polynomial function is the identity function.

protected:

    typedef Coordinate<T> coordinate_type;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXT;
    typedef Eigen::Matrix<T, 2, 1> Vector2T;
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
};


/** \fn virtual T Polynomial<T>::estimate(Range<Point> source_points,
  *                                       Range<Point> target_points,
  *                                       Range<T> weights = Range<T>())
  *
  * \tparam T scalar type
  *
  * The input parameters are ranges <TT>[first, last)</tt> of a sequence (e.g. a
  * STL container like \c vector). The last elements are not included in the
  * ranges which coincides with the convention in the STL.
  *
  * Source and target points as well as weights must correspond to each other.
  * If no weights are given the weights are assumed to be equal to one.
  *
  * Example:
  *
  * \code
  * typedef Polynomial<double>::Point Point;
  *
  * vector<Point> source_points;
  * vector<Point> target_points;
  *
  * // ...
  *
  * Polynomial<double> * polynomial = new ConcretePolynomial<double>;
  *
  * polynomial->estimate(make_range(source_points), make_range(target_points));
  * \endcode
  *
  * \return Returns the RMSE of the estimated transformation.
  */


/** \fn virtual T Polynomial<T>::estimate(Iterator<Point> source_points_first,
  *                                       Iterator<Point> source_points_last,
  *                                       Iterator<Point> target_points_first,
  *                                       Iterator<Point> target_points_last,
  *                                       Iterator<T> weights_first = Iterator<T>(),
  *                                       Iterator<T> weights_last = Iterator<T>())
  *
  * \tparam T scalar type
  *
  * The input parameters are ranges <TT>[first, last)</tt> of a sequence (e.g. a
  * STL container like \c vector). The last elements are not included in the
  * ranges which coincides with the convention in the STL.
  *
  * Source and target points as well as weights must correspond to each other.
  * If no weights are given the weights are assumed to be equal to one.
  *
  * Example:
  *
  * \code
  * typedef Polynomial<double>::Point Point;
  *
  * vector<Point> source_points;
  * vector<Point> target_points;
  *
  * // ...
  *
  * Polynomial<double> * polynomial = new ConcretePolynomial<double>;
  *
  * polynomial->estimate(make_iterator(source_points.begin()),
  *                      make_iterator(source_points.end()),
  *                      make_iterator(target_points.begin()),
  *                      make_iterator(target_points.end()));
  * \endcode
  *
  * \return Returns the RMSE of the estimated transformation.
  */


} // namespace TRTK


#endif // POLYNOMIAL_HPP_7886646936
