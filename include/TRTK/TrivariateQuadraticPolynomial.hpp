/*
    Estimation and computation of a trivariate quadratic polynomial.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.2.0 (2013-03-28)
*/

/** \file TrivariateQuadraticPolynomial.hpp
  * \brief This file contains the \ref TRTK::TrivariateQuadraticPolynomial
  *        "TrivariateQuadraticPolynomial" class.
  */


#ifndef TRIVARIATE_QUADRATIC_POLYNOMIAL_HPP_0512325448
#define TRIVARIATE_QUADRATIC_POLYNOMIAL_HPP_0512325448


#include <cassert>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "Polynomial.hpp"
#include "Range.hpp"


namespace TRTK
{


/** \class TrivariateQuadraticPolynomial
  *
  * \brief Trivariate quadratic polynomial.
  *
  * This class provides means to estimate and compute a trivariate polynomial of
  * degree 2.
  *
  * The transformation is computed as follows:
  *
  * \f[
  * \begin{pmatrix}
  *     x'  \\  y'  \\  z'
  * \end{pmatrix}
  * =
  * f(x, y ,z)
  * =
  * \begin{pmatrix}
  *     a_{11} + a_{12} x + a_{13} y + \cdots + a_{18} y^2 + a_{19} yz + a_{1,10} z^2  \\
  *     a_{21} + a_{22} x + a_{23} y + \cdots + a_{28} y^2 + a_{29} yz + a_{2,10} z^2  \\
  *     a_{31} + a_{32} x + a_{33} y + \cdots + a_{38} y^2 + a_{39} yz + a_{3,10} z^2  \\
  * \end{pmatrix}
  * \f]
  *
  * Most functions check for certain assertions (e.g., if the coordinate size is
  * valid) and trigger an assertion failure if the assumption does not hold.
  * This is meant for debugging purposes only and can be disabled by defining
  * the macro \macro{NDEBUG}.
  *
  * \par Examples
  * 
  * Here is an example of how to use the TrivariateQuadraticPolynomial class:
  *
  * \code
  * #include <iomanip>
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Tools.hpp>
  * #include <TRTK/TrivariateQuadraticPolynomial.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
  *
  * typedef TRTK::Coordinate<double> Point;
  *
  * Point f(const Point & point)
  * {
  *     double x = point.x();
  *     double y = point.y();
  *     double z = point.z();
  *
  *     return Point(-x, x + 20 * y, z + 5 * x * z);
  * };
  *
  * int main()
  * {
  *     // Generate two point sets that relate to each other via the
  *     // trivariate quadratic polynomial function f.
  *
  *     vector<Point> source_points;
  *     vector<Point> target_points;
  *
  *     for (unsigned i = 0; i < 100; ++i)
  *     {
  *         Point source_point(0, 0, 0);
  *
  *         source_point.x() = rand(-10.0, 10.0);
  *         source_point.y() = rand(-10.0, 10.0);
  *         source_point.z() = rand(-10.0, 10.0);
  *
  *         Point target_point = f(source_point);
  *
  *         // Add noise.
  *
  *         target_point.x() += 0.1 * randn();
  *         target_point.y() += 0.1 * randn();
  *         target_point.z() += 0.1 * randn();
  *
  *         source_points.push_back(source_point);
  *         target_points.push_back(target_point);
  *     }
  *
  *     // Estimate the polynomial function f from the above two sets.
  *
  *     TrivariateQuadraticPolynomial<double> polynomial;
  *
  *     // double rmse = polynomial.estimate(make_iterator(source_points.begin()),
  *     //                                   make_iterator(source_points.end()),
  *     //                                   make_iterator(target_points.begin()),
  *     //                                   make_iterator(target_points.end()));
  *     //
  *     // or simply:
  *
  *     double rmse = polynomial.estimate(make_range(source_points),
  *                                       make_range(target_points));
  *
  *     // Print out the results.
  *
  *     cout << "Root Mean Square Error: " << rmse << endl;
  *     cout << "Coefficients: " << endl << setprecision(4) << fixed
  *          << polynomial.getCoefficients().transpose() << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Root Mean Square Error: 0.162183
  * Coefficients:
  *        0.0409        0.0047        0.0017
  *       -0.9991        0.9957        0.0000
  *       -0.0019       19.9994        0.0009
  *       -0.0006       -0.0000        1.0001
  *       -0.0000        0.0002       -0.0002
  *       -0.0002       -0.0001       -0.0000
  *       -0.0005        0.0001        5.0002
  *       -0.0008       -0.0005        0.0002
  *        0.0001       -0.0001       -0.0004
  *        0.0003        0.0001        0.0002
  * \endcode
  *
  * \par Algorithm
  *
  * This algorithm estimates a linear model
  *
  * \f[
  * y_i = x_i^T \beta + \epsilon_i
  * \f]
  *
  * where the parameter vector \f$ \beta \f$ contains the sought coefficients and
  * where \f$ \epsilon_i \f$ represents the unknown error. The parameter vector is
  * estimated using weighted least squares
  *
  * \f[
  * \begin{align}
  *     \hat\beta &= \arg_{\beta}\min \sum_i w_i (x_i^T \beta - y_i)^2 \\
  *               &= \arg_{\beta}\min || W^{1/2} (A \beta - y) ||^2 \\[0.75em]
  *               &= [A^T W A]^{-1} A^T W y
  * \end{align}
  * \f]
  *
  * If no weights are given \f$ w_i \f$ is assumed to be equal to 1.
  *
  * \Note
  *
  * The algorithm returns the (weighted) root-mean-square error RMSE
  * \f$ = \sqrt{\frac{1}{N} \sum_{i=1}^N w_i (f(x_i) - y_i)^2} \f$ of the
  * estimation.
  *
  * Given constant weights \f$ w_i = w \f$ the RMSE is proportional to the RMSE
  * of an ordinary least squares estimation (\f$ w_i = 1 \f$) by a factor of
  * \f$ w^{-1/2} \f$. It follows, a smaller RMSE does not necessarily imply a better
  * estimation result. You might want to compare results by scaling the RMSE with
  * \f$ \bar{w}^{-1/2} \f$ where \f$ \bar{w} \f$ is the mean of all weights.
  *
  * \see Coordinate, Iterator
  *
  * \author Christoph Haenisch
  * \version 0.2.0
  * \date last changed on 2013-03-28
  */

template <class T>
class TrivariateQuadraticPolynomial : public Polynomial<T>
{
private:

    typedef Polynomial<T> super;

protected:

    typedef typename super::VectorXT VectorXT;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::coordinate_type coordinate_type;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::MatrixXT MatrixXT;
    typedef typename super::Point Point;

    using super::DIVISION_BY_ZERO;
    using super::INVALID_ARGUMENT;
    using super::NOT_ENOUGH_POINTS;
    using super::UNEQUAL_NUMBER_OF_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_COORDINATE_SIZE;

    TrivariateQuadraticPolynomial();

    TrivariateQuadraticPolynomial(const MatrixXT &);

    TrivariateQuadraticPolynomial(const TrivariateQuadraticPolynomial &);

    template <class U>
    TrivariateQuadraticPolynomial(const TrivariateQuadraticPolynomial<U> &);

    virtual ~TrivariateQuadraticPolynomial();

    MatrixXT & getCoefficients();
    const MatrixXT & getCoefficients() const;

    T estimate(Range<Point> source_points,
               Range<Point> target_points,
               Range<T> weights = Range<T>());

    T estimate(Iterator<Point> source_points_first,
               Iterator<Point> source_points_last,
               Iterator<Point> target_points_first,
               Iterator<Point> target_points_last,
               Iterator<T> weights_first = Iterator<T>(),
               Iterator<T> weights_last = Iterator<T>());

    Point operator*(const Point &) const;
    Point transform(const Point &) const;

    Polynomial<T> & reset();

private:

    MatrixXT m_matrix;
};


/** \tparam T scalar type
  *
  * \brief Constructs an instance of TrivariateQuadraticPolynomial.
  *
  * The transformation is initialized to be the identity function.
  *
  * \see reset()
  */

template <class T>
TrivariateQuadraticPolynomial<T>::TrivariateQuadraticPolynomial() :
    m_matrix(MatrixXT(3, 10))
{
    m_matrix << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
}


/** \tparam T scalar type
  *
  * \param [in] matrix  A 3x10 matrix containing the coefficients.
  *
  * \brief Constructs an instance of TrivariateQuadraticPolynomial.
  *
  * Sets the internal coefficients to the given \p matrix.
  *
  * \see reset()
  */

template <class T>
TrivariateQuadraticPolynomial<T>::TrivariateQuadraticPolynomial(const MatrixXT & matrix)
{
    assert(matrix.rows() == 3 && matrix.cols() == 10);
    m_matrix = matrix;
}


/** \tparam T scalar type of the newly created instance
  *
  * \brief Copy constructor.
  *
  * \see reset()
  */

template <class T>
TrivariateQuadraticPolynomial<T>::TrivariateQuadraticPolynomial(const TrivariateQuadraticPolynomial & other)
{
    m_matrix =  other.m_matrix;
}


/** \tparam T scalar type of the newly created instance
  * \tparam U scalar type of the copied instance
  *
  * \brief Copy constructor.
  *
  * \see reset()
  */

template <class T>
template <class U>
TrivariateQuadraticPolynomial<T>::TrivariateQuadraticPolynomial(const TrivariateQuadraticPolynomial<U> & other)
{
    m_matrix = other.getCoefficients().template cast<T>();
}


/** \tparam T scalar type
  *
  * \brief Destroys the instance of TrivariateQuadraticPolynomial.
  */

template <class T>
TrivariateQuadraticPolynomial<T>::~TrivariateQuadraticPolynomial()
{
}


/** \tparam T scalar type
  *
  * \return Returns the internal coefficients in form of a 3x10 matrix representation.
  *
  * \brief Returns the internal coefficients.
  */

template <class T>
typename TrivariateQuadraticPolynomial<T>::MatrixXT & TrivariateQuadraticPolynomial<T>::getCoefficients()
{
    return m_matrix;
}


/** \tparam T scalar type
  *
  * \return Returns the internal coefficients in form of a 3x10 matrix representation.
  *
  * \brief Returns the internal coefficients.
  */

template <class T>
const typename TrivariateQuadraticPolynomial<T>::MatrixXT & TrivariateQuadraticPolynomial<T>::getCoefficients() const
{
    return m_matrix;
}


/** \tparam T scalar type
  *
  * \brief Estimates a trivariate quadratic function from two point sets.
  *
  * The input parameters are ranges <TT>[first, last)</tt> of a sequence (e.g. a
  * STL container like \c vector). The last elements are not included in the
  * ranges which coincides with the convention in the STL.
  *
  * Source and target points as well as weights must correspond to each other.
  * If no weights are given, the weights are assumed to be equal to one.
  *
  * \return Returns the RMSE \f$ = \sqrt{\frac{1}{N} \sum_{i=1}^N w_i
  *         (f(x_i) - y_i)^2} \f$ of the estimation.
  *
  * \throws ErrorObj    An error object is thrown if the point sets differ
  *                     in their sizes.
  * \throws ErrorObj    An error object is thrown if not enough (less than
  *                     ten) points are given.
  */

template <class T>
T TrivariateQuadraticPolynomial<T>::estimate(Range<Point> source_points, Range<Point> target_points, Range<T> weights)
{
    return estimate(source_points.begin(),
                    source_points.end(),
                    target_points.begin(),
                    target_points.end(),
                    weights.begin(),
                    weights.end());
}


/** \tparam T scalar type
  *
  * \brief Estimates a trivariate quadratic function from two point sets.
  *
  * The input parameters are ranges <TT>[first, last)</tt> of a sequence (e.g. a
  * STL container like \c vector). The last elements are not included in the
  * ranges which coincides with the convention in the STL.
  *
  * Source and target points as well as weights must correspond to each other.
  * If no weights are given, the weights are assumed to be equal to one.
  *
  * \return Returns the RMSE \f$ = \sqrt{\frac{1}{N} \sum_{i=1}^N w_i
  *         (f(x_i) - y_i)^2} \f$ of the estimation.
  *
  * \throws ErrorObj    An error object is thrown if the point sets differ
  *                     in their sizes.
  * \throws ErrorObj    An error object is thrown if not enough (less than
  *                     ten) points are given.
  */

template <class T>
T TrivariateQuadraticPolynomial<T>::estimate(Iterator<Point> source_points_first,
                                             Iterator<Point> source_points_last,
                                             Iterator<Point> target_points_first,
                                             Iterator<Point> target_points_last,
                                             Iterator<T> weights_first,
                                             Iterator<T> weights_last)
{
    const int number_points = distance(source_points_first, source_points_last);

    if (distance(target_points_first, target_points_last) != number_points)
    {
        ErrorObj error;
        error.setClassName("TrivariateQuadraticPolynomial<T>");
        error.setFunctionName("estimate");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    if (weights_first && weights_last)
    {
        if (distance(weights_first, weights_last) != number_points)
        {
            ErrorObj error;
            error.setClassName("TrivariateQuadraticPolynomial<T>");
            error.setFunctionName("estimate");
            error.setErrorMessage("Point sets must have the same cardinality.");
            error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
            throw error;
        }
    }

    if (number_points < 10)
    {
        ErrorObj error;
        error.setClassName("TrivariateQuadraticPolynomial<T>");
        error.setFunctionName("estimate");
        error.setErrorMessage("Not enough points to estimate the transformation.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    /*

    Explanation of the algorithm:

    The trivariate quadratic function can be described as follows:

    | x' |     | m_1,1  m_1,2  ...  m_1,10 |
    | y  |  =  | m_2,1  m_2,2  ...  m_2,10 |  *  | 1  x  y  ...  y^2  yz  z^2 |^T
    | z' |     | m_3,1  m_3,2  ...  m_3,10 |

    The equation can be rewritten as:

    | x' |     | 1  ...  z^2  0  ...  0    0  ...  0   |     | m_1,1  |
    | y' |  =  | 0  ...  0    1  ...  z^2  0  ...  0   |  *  | ...    |
    | z' |     | 0  ...  0    0  ...  0    1  ...  z^2 |     | m_3,10 |

    Or in short:  b = A * x          (here, x is meant to be (m_1,1, ..., m_3,10)^T)

    By using more and more points (source_1, target_1), ..., (source_n, target_n)

         | A(1) |         | b(1) |
    A := | ...  | ,  b := | ...  |
         | A(n) |         | b(n) |

    this leads to an over-determined system. However, this can be easily solved
    using the pseudo inverse:

    <==>    A * x = b
    <==>    A^T * A * x = A^T * b
    <==>    x = inverse(A^T * A) * A^T * b = pseudo_inverse * b

    Thus, we have found the sought coefficients.

    Update: We now use weighted least squares so that x = inverse(A^T * W * A) * A^T * W * b
            where W is a diagonal matrix

    */

    // Fill the matrix A with the above described entries (different order as above).

    MatrixXT A(3 * number_points, 30);

    A.setZero();

    int i = 0;
    for (Iterator<Point> it = source_points_first; it != source_points_last; ++it, ++i)
    {
        T x = (*it).x();
        T y = (*it).y();
        T z = (*it).z();

        A(0 * number_points + i, 0) = 1;
        A(0 * number_points + i, 1) = x;
        A(0 * number_points + i, 2) = y;
        A(0 * number_points + i, 3) = z;
        A(0 * number_points + i, 4) = x * x;
        A(0 * number_points + i, 5) = x * y;
        A(0 * number_points + i, 6) = x * z;
        A(0 * number_points + i, 7) = y * y;
        A(0 * number_points + i, 8) = y * z;
        A(0 * number_points + i, 9) = z * z;

        A(1 * number_points + i, 10) = 1;
        A(1 * number_points + i, 11) = x;
        A(1 * number_points + i, 12) = y;
        A(1 * number_points + i, 13) = z;
        A(1 * number_points + i, 14) = x * x;
        A(1 * number_points + i, 15) = x * y;
        A(1 * number_points + i, 16) = x * z;
        A(1 * number_points + i, 17) = y * y;
        A(1 * number_points + i, 18) = y * z;
        A(1 * number_points + i, 19) = z * z;

        A(2 * number_points + i, 20) = 1;
        A(2 * number_points + i, 21) = x;
        A(2 * number_points + i, 22) = y;
        A(2 * number_points + i, 23) = z;
        A(2 * number_points + i, 24) = x * x;
        A(2 * number_points + i, 25) = x * y;
        A(2 * number_points + i, 26) = x * z;
        A(2 * number_points + i, 27) = y * y;
        A(2 * number_points + i, 28) = y * z;
        A(2 * number_points + i, 29) = z * z;
    }

    // Fill the vector b.

    VectorXT b(3 * number_points);

    i = 0;
    for (Iterator<Point> it = target_points_first; it != target_points_last; ++it, ++i)
    {
        b(0 * number_points + i) = (*it).x();
        b(1 * number_points + i) = (*it).y();
        b(2 * number_points + i) = (*it).z();
    }

    // Compute the vector x whose components are the sought coefficients.

    VectorXT x;
    VectorXT W = VectorXT::Ones(3 * number_points); // diagonal weighting matrix (W.asDiagonal())

    if (weights_first && weights_last)
    {
        // Weighted Least Squares

        int i = 0;
        for (Iterator<T> it = weights_first; it != weights_last; ++it, ++i)
        {
            W(0 * number_points + i) = *it;
            W(1 * number_points + i) = *it;
            W(2 * number_points + i) = *it;
        }

        x = (A.transpose() * W.asDiagonal() * A).inverse() * (A.transpose() * (W.asDiagonal() * b));
    }
    else
    {
        // Ordinary Least Squares

        MatrixXT pseudo_inverse = (A.transpose() * A).inverse() * A.transpose();
        x = pseudo_inverse * b;
    }

    // Reinterprete the solution vector as a row-major matrix

    // m_matrix = Eigen::Map<MatrixXT, Eigen::Unaligned, Eigen::Stride<1, 10> >(x.data(), 3, 10);
    m_matrix = Eigen::Map<MatrixXT>(x.data(), 10, 3).transpose();

    // Compute the RMSE.

    using std::sqrt;

    if (weights_first && weights_last)
    {
        return  (W.cwiseSqrt().asDiagonal() * (A * x - b)).norm() / sqrt(T(number_points));
    }
    else
    {
        return  (A * x - b).norm() / sqrt(T(number_points));
    }
}


/** \tparam T scalar type
  *
  * \param [in] point Point to be transformed.
  *
  * \return Transformed point.
  *
  * \brief Transforms a point with the internally saved transformation.
  *
  * The given \p point must be of size three.
  */

template <class T>
typename TrivariateQuadraticPolynomial<T>::Point TrivariateQuadraticPolynomial<T>::operator* (const Point & point) const
{
    assert(point.size() == 3);

    return transform(point);
}


/** \tparam T scalar type
  *
  * \brief Resets the transformation.
  *
  * Sets the internal coefficients such that the transformation is the identity
  * function.
  *
  * \return \c *this
  */

template <class T>
Polynomial<T> & TrivariateQuadraticPolynomial<T>::reset()
{
    m_matrix << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 1, 0, 0, 0, 0, 0, 0;

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] point Point to be transformed.
  *
  * \return Transformed point.
  *
  * \brief Transforms a point with the internally saved transformation.
  *
  * The given \p point must be of size two.
  */

template <class T>
typename TrivariateQuadraticPolynomial<T>::Point TrivariateQuadraticPolynomial<T>::transform(const Point & point) const
{
    assert(point.size() == 3);

    Eigen::Matrix<T, 10, 1> vec;

    const T & x = point.x();
    const T & y = point.y();
    const T & z = point.z();

    vec(0) = 1;
    vec(1) = x;
    vec(2) = y;
    vec(3) = z;
    vec(4) = x * x;
    vec(5) = x * y;
    vec(6) = x * z;
    vec(7) = y * y;
    vec(8) = y * z;
    vec(9) = z * z;

    return Point(m_matrix * vec);
}


} // namespace TRTK


#endif // TRIVARIATE_QUADRATIC_POLYNOMIAL_HPP_0512325448
