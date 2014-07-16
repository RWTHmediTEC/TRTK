/*
    Copyright (C) 2010 - 2014 Christoph Haenisch, Fabian Killus

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.5.0 (2012-03-19)
*/

/** \file FitPlane.hpp
  * \brief This file contains the \ref TRTK::FitPlane "FitPlane" class.
  */


#ifndef FIT_PLANE_HPP_5948294843
#define FIT_PLANE_HPP_5948294843


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "Fit3D.hpp"

namespace TRTK
{


/** \tparam T scalar type of the coordinates
  *
  * \brief Estimates the parameters of a 2D plane in 3D space from a set of
  *        points lying on the surface.
  *
  * This class allows estimating the normal of a plane as well as a representative
  * position of a point in the plane. The points may be erroneous since a least
  * square fitting is done. If \f$ x_i \f$ are the data points, \f$ \bar{x} \f$ is
  * a point in the plane and \f$ n \f$ is the normal of the plane, then FitPlane
  * tries to minimize the following objective function:
  *
  * \f[
  *     n := \arg\min_{n} \sum_{i=1}^N ( (x_i - \bar{x})^T \cdot n )^2
  * \f]
  *
  *
  * Here is an more elaborate example to see how to use the class:
  *
  * \code
  * #include <iostream>
  *
  * #include "TRTK/Coordinate.hpp"
  * #include "TRTK/FitPlane.hpp"
  * #include "TRTK/Tools.hpp"
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
  *
  * int main()
  * {
  *     typedef Coordinate<double> Coordinate;
  *
  *     vector<Coordinate> points;
  *
  *     // Construct points lying on a plane.
  *
  *     Coordinate v1(3, 2, 4);
  *     Coordinate v2(1, 0, 5);
  *     Coordinate origin_point(10.0, 30.0, 50.0);
  *
  *     for (int i = 0; i < 100; ++i)
  *     {
  *         Coordinate point_in_plane = origin_point + rand(-10, 10) * v1 + rand(-10, 10) * v2;
  *
  *         // Add some noise.
  *
  *         point_in_plane += Coordinate(randn(), randn(), randn());
  *
  *         points.push_back(point_in_plane);
  *     }
  *
  *     // Estimate the plane parameters.
  *
  *     FitPlane<double> fitPlane;
  *     fitPlane.setPoints(points);
  *
  *     fitPlane.compute();
  *
  *     // Print the estimated values.
  *
  *     cout << "Normal vector: " << v1.cross(v2).normalize() << endl;
  *     cout << "Estimated normal vector: " << fitPlane.getNormal() << endl;
  *     cout << "Estimated center point: " << fitPlane.getPointInPlane();
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Normal vector: (0.666667, -0.733333, -0.133333)
  * Estimated normal vector: (-0.674243, 0.726738, 0.131331)
  * Estimated center point: (9.91952, 30.2422, 47.7684)
  * \endcode
  *
  * \author Christoph Haenisch, Fabian Killus
  * \version 0.5.0
  * \date last changed on 2012-03-19
  */

template <class T>
class FitPlane : public Fit3D<T>
{
private:
    typedef Fit3D<T> super;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::Matrix3T Matrix3T;
    typedef typename super::MatrixXT MatrixXT;

    FitPlane();
    FitPlane(const std::vector<Coordinate<T> > & points);
    FitPlane(const std::vector<Vector3T> & points);
    FitPlane(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points);

    virtual ~FitPlane();

    unsigned getNumberPointsRequired() const;
    void compute();
    T getRMS() const;

    const Coordinate<T> & getNormal() const;
    const Coordinate<T> & getPointInPlane() const;
    T getDistanceFromOrigin() const;
    T getDistanceTo(const Coordinate<T> & point) const;

    using super::DATA_POINTS_TOO_SIMILAR;
    using super::NOT_ENOUGH_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_POINT_SIZE;

private:
    Coordinate<T> m_normal;
    Coordinate<T> m_point_in_plane;
    T m_rms;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs an empty FitPlane object.
  */

template <class T>
FitPlane<T>::FitPlane() :
    m_normal(0, 0, 0),
    m_point_in_plane(0, 0, 0),
    m_rms(0)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the surface of a plane. The points can
  *                    be 3D or 4D homogeneous coordinates.
  *
  * \brief Constructs a FitPlane object.
  */

template <class T>
FitPlane<T>::FitPlane(const std::vector<Coordinate<T> > & points) :
    m_normal(0, 0, 0),
    m_point_in_plane(0, 0, 0),
    m_rms(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the surface of a plane.
  *
  * \brief Constructs a FitPlane object.
  */

template <class T>
FitPlane<T>::FitPlane(const std::vector<Vector3T> & points) :
    m_normal(0, 0, 0),
    m_point_in_plane(0, 0, 0),
    m_rms(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points Points lying on the surface of a plane. The points are
  *                    assumed to be homogeneous coordinates, whose forth
  *                    coordinate entry is equal to one.
  *
  * \brief Constructs an FitPlane object.
  */

template <class T>
FitPlane<T>::FitPlane(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points) :
    m_normal(0, 0, 0),
    m_point_in_plane(0, 0, 0),
    m_rms(0)
{
    super::setPoints(points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Destructs the FitPlane object.
  */

template <class T>
FitPlane<T>::~FitPlane()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Runs the fitting algorithm.
  *
  * There must be at least 3 surface points to perform the fitting.
  *
  * \throw ErrorObj If there are not enough points to fit the sphere, an error
  *                 object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  *
  * \see setPoints(), getPointInPlane() and getNormal()
  */

template <class T>
void FitPlane<T>::compute()
{
    /*

    Explanation of the algorithm:

    The columns of matrix X contain the provided points on the surface of the plane.
    First we subtract the mean of the point set from each point in the set such that
    the new set of points is centered around the point of origin.

    x = X - mean(X)

    With n being the normal vector of the plane, we should ideally get x_i^T * n = 0
    for all columns x_i of the matrix x. Therefore we define an error measure as

    E = sum_i ( x_i^T * n )^2

    This can be rearranged to

    E = sum_i ( x_i^T * n )^2
      = sum_i ( (x_i^T * n)^T * (x_i^T * n) )             because x_i^T * n is a scalar
      = sum_i ( n^T * x_i * x_i^T * n )
      = n^T * sum_i ( x_i * x_i^T ) * n
      = n^T * A * n                                       with the matrix A = x * x^T

    Minimizing the above term yields the sought/unknown normal n. To avoid the trivial
    solution n = 0, we use the Lagrange formalism, i.e. Lagrange multipliers. Our
    auxiliary condition is n * n^T = 1.

    L(n, lambda) := n^T * A * n - lambda * ( n^T * n - 1)   and   Grad( L(n, lambda) ) != 0

    => 2 A n - A lambda I n = 0                           with I being the indentity matrix
    => A n = lambda n

    Thus, the minimization problem reduces to an eigenvalue problem. Since

    E = n^T * A * n = n^T * lambda * n = lambda

    the sought normal is the eigenvector corresponding to the smallest eigenvalue.

    */

    using namespace std;
    using namespace Eigen;

    using std::sqrt;

    const int number_of_points = super::m_points.cols();

    if (number_of_points < 3)
    {
        ErrorObj error;
        error.setClassName("FitPlane<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to fit the plane.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    MatrixXT X = super::m_points;

    // Subtract the population mean and which is a point in the plane.

    VectorXT mean = X.rowwise().mean();
    X.colwise() -= mean;

    m_point_in_plane = Coordinate<T>(mean);

    // Construct the matrix A = sum ( x_i^T * x_i ).

    Matrix3T A = X * X.transpose();

    if (A.isZero())
    {
        ErrorObj error;
        error.setClassName("FitPlane<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Data points are too similar.");
        error.setErrorCode(DATA_POINTS_TOO_SIMILAR);
        throw error;
    }

    // Compute the eigenvalues and the eigenvectors of A.

    // Note: - Since A is symmetric, there are only real eigenvalues.
    //         ==> Use SelfAdjointEigenSolver.
    //       - SelfAdjointEigenSolver normalizes the eigenvectors.

    SelfAdjointEigenSolver<Matrix3T> eigensolver(A);

    VectorXT eigen_values = eigensolver.eigenvalues();
    MatrixXT eigen_vectors = eigensolver.eigenvectors();

    // Find the absolute minimum eigenvalue and its corresponding eigenvector.
    // Do also compute the RMS from the smallest eigenvalue.

    int index = 0;
    T minimum_eigenvalue = abs(eigen_values(index));

    for (int i = 1; i < eigen_values.rows(); ++i)
    {
        if (abs(eigen_values[i]) < minimum_eigenvalue)
        {
            index = i;
            minimum_eigenvalue = abs(eigen_values[index]);
        }
    }

    m_normal = Coordinate<T>(eigen_vectors.col(index));

    T sum_of_squared_errors = minimum_eigenvalue;
    m_rms = sqrt(sum_of_squared_errors / number_of_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the shortest distance between the plane and the origin.
  */

template <class T>
T FitPlane<T>::getDistanceFromOrigin() const
{
    using std::abs;

    // The shortest distance can be found by projecting a vector pointing
    // to an arbitrary point in the plane (we just use the population mean
    // of all points) onto the normal vector of the plane.

    return abs(m_point_in_plane.dot(m_normal));
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the shortest distance between the plane and the given point.
  */

template <class T>
T FitPlane<T>::getDistanceTo(const Coordinate<T> & point) const
{
    using std::abs;

    /*

    The shortest distance can be found by projecting a difference vector
    formed from the given point and an arbitrary point in the plane (we
    just use the population mean of all points) onto the normal vector of
    the plane.

    If we take the normal n and two orthonormal vectors alpha and beta
    which altogether form a new basis, then we can express the above
    difference vector d by a linear combination as follows:

        d = a * alpha + b * beta + distance_to_plane * n

    Now, a simple inner product with n yields the sought distance

        d * n = 0 + 0 + distance_to_plane * 1

    */

    return abs((point - m_point_in_plane).dot(m_normal));
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the normal of the plane.
  */

template <class T>
const Coordinate<T> & FitPlane<T>::getNormal() const
{
    return m_normal;
}


template <class T>
inline unsigned FitPlane<T>::getNumberPointsRequired() const
{
    return 3;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns a point lying in the plane.
  */

template <class T>
const Coordinate<T> & FitPlane<T>::getPointInPlane() const
{
    return m_point_in_plane;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the root mean square error.
  */

template <class T>
T FitPlane<T>::getRMS() const
{
    return m_rms;
}


} // namespace TRTK


#endif // FITSPHERE_HPP_6184653510
