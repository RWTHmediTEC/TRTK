/*
    Estimates the parameters of a 2D line from a set of points lying on
    the line.

    Copyright (C) 2010 - 2014 Christoph Haenisch, Fabian Killus

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.6.0 (2012-03-29)
*/

/** \file FitLine.hpp
  * \brief This file contains the \ref TRTK::FitLine "FitLine" class.
  */


#ifndef FIT_LINE_HPP_8904538197
#define FIT_LINE_HPP_8904538197


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/StdVector>

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "Fit2D.hpp"


namespace TRTK
{


/** \tparam T scalar type of the coordinates
  *
  * \brief Estimates the parameters of a line from a set of 2D points.
  *
  * This class allows estimating the slope, the intercepts, the normal, et
  * cetera of a line from a given set of points. The points may be erroneous
  * since a least square fitting is done. If \f$ (x_i, y_i)^T \f$ are the data
  * points, if \f$ m \f$ is the slope and if \f$ b \f$ is the y-intercept, then
  * FitLine tries to minimize the sum of the squared residuals as shown below:
  *
  * \f[
  * (m, b) := \arg\min_{(m, b)}
  * \sum_{i=1}^N (x_i m + b - y_i)^2
  * \f]
  *
  * Here is an more elaborate example to see how to use the class:
  *
  * \code
  * #include <iostream>
  *
  * #include <TRTK/FitLine.hpp>
  * #include <TRTK/Tools.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  *
  * int main()
  * {
  *
  *     vector<Coordinate<double> > points_on_line;
  *
  *     // Construct some points lying on a line and add some noise.
  *
  *     double slope = 0.7;
  *     double y_intercept = -3;
  *
  *     for (int i = -10; i < 10; ++i)
  *     {
  *         using Tools::randn;
  *
  *         double x = i + randn(0.0, 0.1);
  *         double y = i * slope + y_intercept + randn(0.0, 0.1);
  *
  *         Coordinate<double> point(x, y);
  *
  *         points_on_line.push_back(point);
  *     }
  *
  *     // Estimate the line parameters.
  *
  *     FitLine<double> fitLine(points_on_line);
  *
  *     fitLine.compute();
  *
  *     cout << "Slope: " << fitLine.getSlope() << endl;
  *     cout << "Y-intercept: " << fitLine.getYIntercept() << endl;
  *     cout << "Direction Vector: " << fitLine.getDirectionVector() << endl;
  *     cout << "Distance from origin: " << fitLine.getDistanceFromOrigin() << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Slope: 0.695251
  * Y-intercept: -3.02231
  * Direction Vector: (-0.82106, -0.570842)
  * Distance from origin: 2.4815
  * \endcode
  *
  * \author Christoph Haenisch, Fabian Killus
  * \version 0.6.0
  * \date last changed on 2012-03-29
  */

template <class T>
class FitLine : public Fit2D<T>
{
private:
    typedef Fit2D<T> super;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::Vector2T Vector2T;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::Matrix2T Matrix2T;
    typedef typename super::MatrixXT MatrixXT;

    FitLine();
    FitLine(const std::vector<Coordinate<T> > & points);
    FitLine(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & points);
    FitLine(const std::vector<Vector3T> & points);

    virtual ~FitLine();

    void compute();
    T getRMS() const;

    unsigned getNumberPointsRequired() const;

    const Coordinate<T> & getDirectionVector() const;
    const Coordinate<T> & getNormalVector() const;
    const Coordinate<T> & getPointOnLineSegment() const;

    T getSlope() const;
    T getYIntercept() const;
    T getXIntercept() const;
    T getDistanceFromOrigin() const;
    T getDistanceTo(const Coordinate<T> & point) const;

    using super::DATA_POINTS_TOO_SIMILAR;
    using super::INFINITY_NOT_AVAILABLE;
    using super::NAN_NOT_AVAILABLE;
    using super::NOT_ENOUGH_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_POINT_SIZE;

private:
    T m_rms;
    T m_slope;
    T m_y_intercept;
    T m_x_intercept;
    Coordinate<T> m_direction_vector;
    Coordinate<T> m_normal_vector;
    Coordinate<T> m_point_on_line_segment;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs an empty FitLine object.
  */

template <class T>
FitLine<T>::FitLine() :
    m_rms(0),
    m_slope(0),
    m_y_intercept(0),
    m_x_intercept(0),
    m_direction_vector(0, 0),
    m_normal_vector(0, 0),
    m_point_on_line_segment(0, 0)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the line. The points can
  *             be 2D or 3D homogeneous coordinates.
  *
  * \brief Constructs a FitLine object.
  */

template <class T>
FitLine<T>::FitLine(const std::vector<Coordinate<T> > & points) :
    m_rms(0),
    m_slope(0),
    m_y_intercept(0),
    m_x_intercept(0),
    m_direction_vector(0, 0),
    m_normal_vector(0, 0),
    m_point_on_line_segment(0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the line.
  *
  * \brief Constructs a FitLine object.
  */

template <class T>
FitLine<T>::FitLine(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & points) :
    m_rms(0),
    m_slope(0),
    m_y_intercept(0),
    m_x_intercept(0),
    m_direction_vector(0, 0),
    m_normal_vector(0, 0),
    m_point_on_line_segment(0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the line. The points are assumed
  *                    to be homogeneous coordinates, whose third coordinate
  *                    entry is equal to one.
  *
  * \brief Constructs an FitLine object.
  */

template <class T>
FitLine<T>::FitLine(const std::vector<Vector3T> & points) :
    m_rms(0),
    m_slope(0),
    m_y_intercept(0),
    m_x_intercept(0),
    m_direction_vector(0, 0),
    m_normal_vector(0, 0),
    m_point_on_line_segment(0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Destructs the FitLine object.
  */

template <class T>
FitLine<T>::~FitLine()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Runs the fitting algorithm.
  *
  * There must be at least 2 known points on the line to perform the fitting.
  *
  * \throw ErrorObj If there are not enough points to fit the circle, an error
  *                 object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  *
  * \see setPoints(), getSlope() and getYIntercept()
  */

template <class T>
void FitLine<T>::compute()
{
    /*

    Explanation of the algorithm:

    First we compute a point lying on the line segment (the mean works) and
    move the line such that it pass through the origin

        x_i := x_i - E[x]       (E[x] = expected value = population mean)

    Then the difference vector between a point on the line (x_i, y_i) and the
    point of origin (0, 0) is collinear to the direction vector of the line.
    The direction vector in turn is orthogonal to the normal n of the line.
    Thus, for every point x_i on the line it holds that

        x_i * n = 0

    This can be used to define an error which is to be minimized yielding the
    sought normal n

        Error = sum_i (x_i^T * n)^2
              = sum_i [ (x_i^T * n)^T * (x_i^T * n) ]
              = sum_i [ n^T * (x_i * x_i^T) * n ]
              = n^T * [ sum_i (x_i * x_i^T) ] * n     (note the outer product!)
              = n^T * A * n

    The above quadratic form can be solved using Lagrange multipliers

        L := n^T * A * n  -  lambda * (n^T * n - 1)

        grad L = 0

    where n^T * n = 1 is an auxiliary condition avoiding the trival solution
    n = (0, 0). As we will see, this is equivalent to an eigenvalue problem

        grad L = 2 * A * n - 2 * lambda * I * n = 0

        ==>  A n = lambda n

    And since

        Error = n^T * A * n
              = n^T * lambda * n
              = lambda

    is to be minimized, we search for an eigenvector of A corresponding to the
    absolute smallest eigenvalue.

    Finally, the sought direction vector of the line is an orthogonal vector
    to the normal n.

    */

    using namespace std;
    using namespace Eigen;

    const int number_of_points = super::m_points.cols();

    if (number_of_points < 2)
    {
        ErrorObj error;
        error.setClassName("FitLine<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to fit the line.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    MatrixXT X = super::m_points;

    // subtract the population mean which is a point on the line segment

    VectorXT mean = X.rowwise().mean();

    X.colwise() -= mean;

    m_point_on_line_segment = Coordinate<T>(mean);

    // Construct the matrix A = sum ( x_i^T * x_i )

    Matrix<T, 2, 2> A = X * X.transpose();

    if (A.isZero())
    {
        ErrorObj error;
        error.setClassName("FitLine<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Data points are too similar.");
        error.setErrorCode(DATA_POINTS_TOO_SIMILAR);
        throw error;
    }

    // Compute the eigenvalues and the eigenvectors of A.

    // Note: - Since A is symmetric, there are only real eigenvalues.
    //         ==> Use SelfAdjointEigenSolver.
    //       - SelfAdjointEigenSolver normalizes the eigenvectors.

    SelfAdjointEigenSolver<Matrix2T> eigen_solver(A);

    VectorXT eigen_values = eigen_solver.eigenvalues();
    MatrixXT eigen_vectors = eigen_solver.eigenvectors();

    // Find the absolute minimum eigenvalue and its corresponding eigenvector.
    // Do also compute the RMS from the smallest eigenvalue.

    using std::sqrt;

    if (abs(eigen_values(0)) < abs(eigen_values(1)))
    {
        m_normal_vector = Coordinate<T>(eigen_vectors.col(0));

        T sum_of_squared_errors = abs(eigen_values(0));
        m_rms = sqrt(sum_of_squared_errors / number_of_points);
    }
    else
    {
        m_normal_vector = Coordinate<T>(eigen_vectors.col(1));

        T sum_of_squared_errors = abs(eigen_values(1));
        m_rms = sqrt(sum_of_squared_errors / number_of_points);
    }

    // Now, we can easily compute the direction vector and the intercepts.

    m_direction_vector = Coordinate<T>(-m_normal_vector.y(), m_normal_vector.x());

    // There are three cases:
    // (i)    the line is parallel to the x-intercept
    // (ii)   the line is parallel to the y-intercept
    // (iii)  the line is not parallel to any axis

    T epsilon = numeric_limits<T>::is_specialized ? 10 * numeric_limits<T>::epsilon() : 1e-8;

    if (abs(m_direction_vector.y()) < epsilon) // case (i)
    {
        // y = mx + b = b

        m_slope = 0.0;

        if (numeric_limits<T>::has_quiet_NaN)
        {
            m_x_intercept = numeric_limits<T>::quiet_NaN();
        }
        else
        {
            ErrorObj error;
            error.setClassName("FitLine<T>");
            error.setFunctionName("compute");
            error.setErrorMessage("Slope is zero but the x-intercept cannot be set to NaN.");
            error.setErrorCode(NAN_NOT_AVAILABLE);
            throw error;
        }

        m_y_intercept = m_point_on_line_segment.y();
    }
    else if (abs(m_direction_vector.x()) < epsilon) // case (ii)
    {
        // x = a

        if (numeric_limits<T>::has_infinity)
        {
            m_slope = numeric_limits<T>::infinity();
        }
        else
        {
            ErrorObj error;
            error.setClassName("FitLine<T>");
            error.setFunctionName("compute");
            error.setErrorMessage("Slope is infinity but T cannot represent this number.");
            error.setErrorCode(INFINITY_NOT_AVAILABLE);
            throw error;
        }

        m_x_intercept = m_point_on_line_segment.x();

        if (numeric_limits<T>::has_quiet_NaN)
        {
            m_y_intercept = numeric_limits<T>::quiet_NaN();
        }
        else
        {
            ErrorObj error;
            error.setClassName("FitLine<T>");
            error.setFunctionName("compute");
            error.setErrorMessage("Slope is zero but the y-intercept cannot be set to NaN.");
            error.setErrorCode(NAN_NOT_AVAILABLE);
            throw error;
        }
    }
    else // case (iii)
    {
        // direction vector = (1, slope)^T

        m_slope = m_direction_vector.y() / m_direction_vector.x();

        // y = mx + b   ==>   b = y - mx

        m_y_intercept = m_point_on_line_segment.y() - m_slope * m_point_on_line_segment.x();

        // y = mx + b = 0  ==>   x = -b/m

        m_x_intercept = - m_y_intercept / m_slope;
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the center of the line segment.
  */

template <class T>
const Coordinate<T> & FitLine<T>::getPointOnLineSegment() const
{
    return m_point_on_line_segment;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the direction vector of the line.
  */

template <class T>
const Coordinate<T> & FitLine<T>::getDirectionVector() const
{
    return m_direction_vector;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the shortest distance from the line to the origin.
  */

template <class T>
T FitLine<T>::getDistanceFromOrigin() const
{
    using std::abs;

    // The shortest distance can be found by projecting a vector pointing
    // to an arbitrary point on the line (we just use the population mean
    // of all points) onto the normal vector of the line.

    return abs(m_point_on_line_segment.dot(m_normal_vector));
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] point   A 2D coordinate.
  *
  * \brief Returns the shortest distance from the line to the given point.
  */

template <class T>
T FitLine<T>::getDistanceTo(const Coordinate<T> & point) const
{
    assert(point.size() == 2);

    // If d is the difference vector between an arbitrary point on the line
    // (we simply use the center of the line segment) and the currently processed
    // point, then the shortest distance to the line equals d * n where n is
    // then normal of the line.

    using std::abs;
    return abs((point - m_point_on_line_segment).dot(m_normal_vector)); // = d * n
}


template <class T>
inline unsigned FitLine<T>::getNumberPointsRequired() const
{
    return 2;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the normal vector of the line.
  */

template <class T>
const Coordinate<T> & FitLine<T>::getNormalVector() const
{
    return m_normal_vector;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the root mean square error.
  */

template <class T>
T FitLine<T>::getRMS() const
{
    return m_rms;

    /*

    const int num_points = super::m_points.cols();

    T sum_of_squared_errors = 0.0;

    for (int i = 0; i < num_points; ++i)
    {
        Coordinate<T> point(super::m_points.col(i));

        // If d is the difference vector between an arbitrary point on the line
        // (we simply use the center of the line segment) and the currently processed
        // point, then the shortest distance to the line equals d * n where n is
        // then normal of the line. Ideally the distance is zero, so every deviation
        // constitutes an error.

        T error = (point - m_point_on_line_segment).dot(m_normal_vector); // = d * n
        sum_of_squared_errors +=  error * error;
    }

    return sqrt(sum_of_squared_errors / num_points);

    */
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the slope of the line.
  *
  * \note If the line is parallel to the y-axis, slope is set to
  *       <tt>numeric_limits<T>::infinity()</tt>.
  */

template <class T>
T FitLine<T>::getSlope() const
{
    return m_slope;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the x-intercept of the line.
  *
  * \note If the line is parallel to the x-axis, x-intercept is
  *       set to <tt>numeric_limits<T>::quiet_NaN()</tt>.
  */

template <class T>
T FitLine<T>::getXIntercept() const
{
    return m_x_intercept;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns y-intercept of the line.
  *
  * \note If the line is parallel to the y-axis, y-intercept is
  *       set to <tt>numeric_limits<T>::quiet_NaN()</tt>.
  */

template <class T>
T FitLine<T>::getYIntercept() const
{
    return m_y_intercept;
}


} // namespace TRTK


#endif // FIT_LINE_HPP_8904538197
