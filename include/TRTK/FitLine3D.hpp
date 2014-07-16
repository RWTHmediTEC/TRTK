/*
    Estimates the parameters of a line in 3D space from a set of points lying on
    the line.

    Copyright (C) 2010 - 2014 Christoph Haenisch, Fabian Killus

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.4.1 (2013-08-20)
*/

/** \file FitLine3D.hpp
  * \brief This file contains the \ref TRTK::FitLine3D "FitLine3D" class.
  */


#ifndef FIT_LINE_3D_HPP_9482948571
#define FIT_LINE_3D_HPP_9482948571


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/SVD>

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "Fit3D.hpp"


namespace TRTK
{


/** \tparam T scalar type of the coordinates
  *
  * \brief Estimates the parameters of a line in 3D space from a set of points lying on
  *        the line.
  *
  * This class allows estimating the direction vector, normal vector, et cetera of
  * a line from a given set of points. The points may be erroneous since a principal
  * component analysis (PCA) is performed; it is assumed, that the line propagates
  * in the direction of largest variance.
  *
  * Here is an more elaborate example to see how to use the class:
  *
  * \code
  * #include <iostream>
  *
  * #include <TRTK/FitLine3D.hpp>
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
  *     // Construct some points lying on a line.
  *
  *     vector<Coordinate> points;
  *
  *     Coordinate direction_vector = Coordinate(4, 0, 3);
  *     Coordinate point_on_line = Coordinate(1, 3, -1);
  *
  *     for (int i = 0; i < 100; ++i)
  *     {
  *         Coordinate point = point_on_line + rand(-10.0, 10.0) * direction_vector;
  *
  *         // Add some noise.
  *
  *         point.x() += 0.1 * randn();
  *         point.y() += 0.1 * randn();
  *         point.z() += 0.1 * randn();
  *
  *         points.push_back(point);
  *     }
  *
  *     // Estimate the line parameters.
  *
  *     FitLine3D<double> fitLine3D(points);
  *     fitLine3D.compute();
  *
  *     cout << "The direction vector is: " << fitLine3D.getDirectionVector() << endl;
  *     cout << "A point on the line is: " << fitLine3D.getPointOnLineSegment() << endl;
  *     cout << "The distance from the origin is: " << fitLine3D.getDistanceFromOrigin() << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * The direction vector is: (0.800062, 0.000370052, 0.599917)
  * A point on the line is: (2.81097, 3.00661, 0.351312)
  * The distance from the origin is: 3.31798
  * \endcode
  *
  * \author Christoph Haenisch, Fabian Killus
  * \version 0.4.1
  * \date last changed on 2013-08-20
  */

template <class T>
class FitLine3D : public Fit3D<T>
{
private:
    typedef Fit3D<T> super;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::MatrixXT MatrixXT;

    FitLine3D();
    FitLine3D(const std::vector<Coordinate<T> > & points);
    FitLine3D(const std::vector<Vector3T> & points);
    FitLine3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points);

    virtual ~FitLine3D();

    unsigned getNumberPointsRequired() const;
    void compute();
    T getRMS() const;

    const Coordinate<T> & getDirectionVector() const;
    Coordinate<T> getNormalVector() const;
    const Coordinate<T> & getPointOnLineSegment() const;
    T getDistanceFromOrigin() const;
    T getDistanceTo(const Coordinate<T> & point) const;

    enum Error {
        NOT_ENOUGH_POINTS,
        UNKNOWN_ERROR,
        WRONG_POINT_SIZE
    };

private:
    Coordinate<T> m_direction_vector;
    Coordinate<T> m_point_on_line;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs an empty FitLine3D object.
  */

template <class T>
FitLine3D<T>::FitLine3D() :
    m_direction_vector(0, 0, 0),
    m_point_on_line(0, 0, 0)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the line. The points can
  *                    be 3D or 4D homogeneous coordinates.
  *
  * \brief Constructs a FitLine3D object.
  */

template <class T>
FitLine3D<T>::FitLine3D(const std::vector<Coordinate<T> > & points) :
    m_direction_vector(0, 0, 0),
    m_point_on_line(0, 0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the line.
  *
  * \brief Constructs a FitLine3D object.
  */

template <class T>
FitLine3D<T>::FitLine3D(const std::vector<Vector3T> & points) :
    m_direction_vector(0, 0, 0),
    m_point_on_line(0, 0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the line. The points are assumed to
  *                    be homogeneous coordinates, whose forth coordinate
  *                    entry is equal to one.
  *
  * \brief Constructs an FitLine3D object.
  */

template <class T>
FitLine3D<T>::FitLine3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points) :
    m_direction_vector(0, 0, 0),
    m_point_on_line(0, 0, 0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Destructs the FitLine3D object.
  */

template <class T>
FitLine3D<T>::~FitLine3D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Runs the fitting algorithm.
  *
  * There must be at least 2 known points on the line to perform the fitting.
  *
  * \throw ErrorObj If there are not enough points to fit the line, an error
  *                 object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  *
  * \see setPoints(), getPointOnLineSegment() and getGradient()
  */

template <class T>
void FitLine3D<T>::compute()
{
    /*

    Explanation of the algorithm:

    A reasonable assumption is that the line propagates along the direction of largest
    variance of all known points on the line. Doing a principal component analysis (PCA)
    of the given data the orientation of the first principal component corresponds to
    the sought direction of the line.

    The PCA can be computed with a SVD. If we compute the SVD of a matrix M whose columns
    are the given data points

        M = U * S * V^T,

    then the columns of the unitary matrix V are the principal components of M. See
    http://www.snl.salk.edu/~shlens/pca.pdf for an extreme good and detailed description.

    */

    using namespace std;
    using namespace Eigen;

    const int num_points = super::m_points.cols();

    if (num_points < 2)
    {
        ErrorObj error;
        error.setClassName("FitLine3D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to fit the line.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }


    MatrixXT X = super::m_points;

    // Subtract the population mean and store it. This is inevitable for doing a PCA.

    Vector3T mean = X.rowwise().mean();
    X.colwise() -= mean;

    m_point_on_line = Coordinate<T>(mean);

    // Construct the matrix Y (see scriptum).

    using std::sqrt;
    MatrixXT Y = X.transpose() / sqrt(num_points - 1.0);

    // Compute the SVD.
    // Note: Eigen sorts the singular values in decreasing order.

    JacobiSVD<MatrixXT> svd(Y, ComputeThinU | ComputeThinV);
    svd.computeV();

    // Extract the principal components.

    MatrixXT P = svd.matrixV();

    // The examined line propagates along the first principal component.

    m_direction_vector = Coordinate<T>(P.col(0)).normalize();
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the direction vector of the line.
  */

template <class T>
const Coordinate<T> & FitLine3D<T>::getDirectionVector() const
{
    return m_direction_vector;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns a normal vector.
  *
  * The normal is defined such that starting at the origin and
  * running along the normal yields a point on the line.
  */

template <class T>
Coordinate<T> FitLine3D<T>::getNormalVector() const
{
    /*

    The shortest distance between a line and the origin is found along a
    line segment perpendicular to that line (i.e. a normal running through
    the origin).

    Given a point P on the line, we might decompose it into two vectors a
    and b, one (a) being collinear with direction vector v and the other
    one (b) being collinear with a normal n (with this respect, n and v
    form a new basis)

        P = a + b = alpha * v + beta * n.

    Then it holds, that

        P * v = alpha * 1 + beta * 0 = alpha

    where alpha is the length of the line segment between P and P', and
    where P' is an interception point between the normal and the line.
    beta = |P'| is the sought distance between the line and the origin.
    Since P, P' and the origin form a right angle triangle, we are able
    to compute the absolute value of beta by

        beta = sqrt{ |P|^2 - alpha^2 } = sqrt{ |P|^2 - (P * v)^2 }

    Then the normal can be easily computed by rearranging the first equation

        n = ( P - alpha * v) / beta

    Since we only have the absolute value of beta, we might still need to
    negate the normal n.

    */

    using std::sqrt;

    T alpha = m_point_on_line.dot(m_direction_vector);
    T beta = sqrt(m_point_on_line.norm() * m_point_on_line.norm() - alpha * alpha);
    Coordinate<T> normal = (m_point_on_line - alpha * m_direction_vector).normalize();
    // Coordinate<T> normal = (m_point_on_line - alpha * m_direction_vector) / beta;

    T residual = (m_point_on_line - alpha * m_direction_vector - beta * normal).norm();

    if (Tools::isZero(residual))
    {
        return normal;
    }
    else
    {
        return -1.0 * normal;
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the shortest distance between the line and the origin.
  */

template <class T>
T FitLine3D<T>::getDistanceFromOrigin() const
{
    /*

    The shortest distance between a line and the origin is found along a
    line segment perpendicular to that line (i.e. a normal running through
    the origin).

    Given a point P on the line, we might decompose it into two vectors a
    and b, one (a) being collinear with direction vector v and the other
    one (b) being collinear with a normal n (with this respect, n and v
    form a new basis)

        P = a + b = alpha * v + beta * n.

    Then it holds, that

        P * v = alpha * 1 + beta * 0 = alpha

    where alpha is the length of the line segment between P and P', and
    where P' is an interception point between the normal and the line.
    beta = |P'| is the sought distance between the line and the origin.
    Since P, P' and the origin form a right angle triangle, we are able
    to compute beta by

        beta = sqrt{ |P|^2 - alpha^2 } = sqrt{ |P|^2 - (P * v)^2 }

    */

    using std::sqrt;

    T alpha = m_point_on_line.dot(m_direction_vector);
    T beta = sqrt(m_point_on_line.norm() * m_point_on_line.norm() - alpha * alpha);

    return beta;
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] point   A 3D coordinate.
  *
  * \brief Returns the shortest distance from the line to the given point.
  */

template <class T>
T FitLine3D<T>::getDistanceTo(const Coordinate<T> & point) const
{
    assert(point.size() == 3);

    // To get the distance between a point and the line, we can create
    // a difference vector between the given point and a point on the
    // line. Then this difference vector can be projected to the line
    // forming a right angle triangle (similar as described above).
    // Eventually, using the Pythagorean theorem, the distance can be
    // computed.

    using std::abs;
    using std::sqrt;

    Coordinate<T> difference_vector  = point - m_point_on_line;

    T projection = difference_vector.dot(m_direction_vector);

    T distance = sqrt(abs(difference_vector.norm() * difference_vector.norm() -
                          projection * projection));

    return distance;
}


template <class T>
inline unsigned FitLine3D<T>::getNumberPointsRequired() const
{
    return 2;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns a point on the line.
  */

template <class T>
const Coordinate<T> & FitLine3D<T>::getPointOnLineSegment() const
{
    return m_point_on_line;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the root mean square error.
  */

template <class T>
T FitLine3D<T>::getRMS() const
{
    using std::abs;

    const int number_of_points = super::m_points.cols();

    T sum_of_squared_errors = 0;

    for (int i = 0; i < number_of_points; ++i)
    {
        // To get the distance between a point and the line, we can create
        // a difference vector between the given point and a point on the
        // line. Then this difference vector can be projected to the line
        // forming a right angle triangle (similar as described above).
        // Eventually, using the Pythagorean theorem, the distance can be
        // computed.

        Coordinate<T> point(super::m_points.col(i));
        Coordinate<T> difference_vector  = point - m_point_on_line;

        T projection = difference_vector.dot(m_direction_vector);

        T squared_distance = abs(difference_vector.norm() * difference_vector.norm() -
                             projection * projection);

        sum_of_squared_errors += squared_distance;
    }

    using std::sqrt;

    return sqrt(sum_of_squared_errors / number_of_points);
}


} // namespace TRTK


#endif // FIT_LINE_3D_HPP_9482948571
