/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.4 (2013-08-20)
*/

/** \file EstimateProjectiveTransformation3D.hpp
  * \brief This file contains the \ref TRTK::EstimateProjectiveTransformation3D
  *        "EstimateProjectiveTransformation3D" class.
  */


#ifndef ESTIMATE_PROJECTIVE_TRANSFORMATION_3D_HPP_1231333218
#define ESTIMATE_PROJECTIVE_TRANSFORMATION_3D_HPP_1231333218


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "EstimateTransformation3D.hpp"
#include "Transform3D.hpp"


namespace TRTK
{


/** \class EstimateProjectiveTransformation3D
  *
  * \brief Estimates a 3D projective transformation from two point sets.
  *
  * This class estimates a 3D projective transformation between two point sets
  * by using Newton's method. The point sets must have the same cardinality,
  * and the points must correspond to each other. There must be at least five
  * corresponding point pairs.
  *
  * The algorithm estimates a transformation matrix as shown below:
  *
  * \f[
  *     \begin{pmatrix}
  *         y_1  \\  y_2  \\  y_3
  *     \end{pmatrix}
  *     =
  *     \frac{1}{y'_4}
  *     \begin{pmatrix}
  *         y'_1  \\  y'_2  \\  y'_3
  *     \end{pmatrix}
  *
  *     \quad \wedge \quad
  *
  *     \begin{pmatrix}
  *         y'_1  \\  y'_2  \\  y'_3  \\  y'_4
  *     \end{pmatrix}
  *     =
  *     \begin{pmatrix}
  *         a_{11}  &  a_{12}  &  a_{13}  &  a_{14}  \\
  *         a_{21}  &  a_{22}  &  a_{23}  &  a_{24}  \\
  *         a_{31}  &  a_{32}  &  a_{33}  &  a_{34}  \\
  *         a_{41}  &  a_{42}  &  a_{43}  &  a_{44}
  *     \end{pmatrix}
  *     \begin{pmatrix}
  *         x_1  \\  x_2  \\  x_3  \\  1
  *     \end{pmatrix}
  * \f]
  *
  * The source points are the set of all \f$ x \f$ and the target points are
  * the set of all \f$ y \f$.
  *
  * \note
  *   - To obtain stable estimation results, at least six point pairs should
  *     be given.
  *   - Some functions might throw an \ref ErrorObj "error object". See the
  *     appropriate function for more details.
  *   - For a more detailed explanation of the algorithm, please have a look
  *     at the source code.
  *
  * \see \ref HomogeneousCoordinates
  *
  * Here is an more elaborate example to see, how to use the class:
  *
  * \code
  *
  * #include <cstdlib>
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Transform3D.hpp>
  * #include <TRTK/EstimateProjectiveTransformation3D.hpp>
  *
  * using std::cout;
  * using std::endl;
  * using std::vector;
  *
  * using namespace TRTK;
  *
  *
  * int main()
  * {
  *     // Construct a projective transformation.
  *
  *     Transform3D<double> transform;
  *     transform.a12() = 1;
  *     transform.a14() = 3;
  *     transform.a21() = 2;
  *     transform.a32() = 3;
  *     transform.a34() = 2;
  *     transform.a42() = 1;
  *     transform.a43() = 1;
  *
  *     // Construct two sets with source and target points, respectively.
  *
  *     vector<Coordinate<double> > source_points;
  *     vector<Coordinate<double> > target_points;
  *
  *     for (unsigned i = 0; i < 6; ++i)
  *     {
  *         double x = std::rand() % 100;
  *         double y = std::rand() % 100;
  *         double z = std::rand() % 100;
  *
  *         Coordinate<double> source_point(x, y, z);
  *         Coordinate<double> target_point = transform * source_point;
  *
  *         source_points.push_back(source_point);
  *         target_points.push_back(target_point);
  *     }
  *
  *     // Perform the transformation estimation.
  *
  *     EstimateProjectiveTransformation3D<double> estimateProjectiveTransformation3D(source_points,
  *                                                                                   target_points);
  *     estimateProjectiveTransformation3D.setMaxIterations(20);
  *
  *     estimateProjectiveTransformation3D.compute();
  *
  *     // Display the results.
  *
  *     cout.precision(4);
  *     cout << std::fixed;
  *
  *     cout << "Original transformation matrix:" << endl << endl
  *          << transform.getTransformationMatrix() << endl << endl;
  *
  *     cout << "Estimated transformation matrix:" << endl << endl
  *          << estimateProjectiveTransformation3D.getTransformationMatrix() << endl << endl;
  *
  *     // Example of how to use the result.
  *
  *     Transform3D<double> transform2 = estimateProjectiveTransformation3D.getTransformationMatrix();
  *
  *     cout << "Source point 1:               " << source_points[0] << endl
  *          << "Target point 1 (original):    " << target_points[0] << endl
  *          << "Target point 1 (transformed): " << transform2 * source_points[0] << endl << endl;
  *
  *     cout << "RMS: " << estimateProjectiveTransformation3D.getRMS() << endl;
  *
  *     return 0;
  * }
  *
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * Original transformation matrix:
  *
  * 1.0000 1.0000 0.0000 3.0000
  * 2.0000 1.0000 0.0000 0.0000
  * 0.0000 3.0000 1.0000 2.0000
  * 0.0000 1.0000 1.0000 1.0000
  *
  * Estimated transformation matrix:
  *
  *      0.0929      0.0689     -0.0043      2.7760
  *      0.1954      0.0466      0.0066      3.4745
  *     -0.0280      0.2753      0.1490      1.3863
  *     -0.0096      0.0830      0.1229      1.0000
  *
  * Source point 1:               (41.0000, 67.0000, 34.0000)
  * Target point 1 (original):    (1.0882, 1.4608, 2.3235)
  * Target point 1 (transformed): (1.0688, 1.4343, 2.2961)
  *
  * RMS: 0.0557
  *
  * \endcode
  *
  * \note If you plan to use this class with an STL container, please have a
  *       look at <a href=http://eigen.tuxfamily.org/dox/TopicStlContainers.html>
  *       this site. </a>
  *
  * \see EstimateRigidTransformation3D, EstimateProjectiveTransformation3D
  *
  * \author Christoph Haenisch
  * \version 0.2.4
  * \date 2013-08-20
  */

template <class T>
class EstimateProjectiveTransformation3D : public EstimateTransformation3D<T>
{
private:

    typedef EstimateTransformation3D<T> super;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;

    typedef typename super::ArrayXT ArrayXT;
    typedef typename super::MatrixXT MatrixXT;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::Vector2T Vector2T;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::RowVector2T RowVector2T;
    typedef typename super::RowVector3T RowVector3T;
    typedef typename super::Matrix2T Matrix2T;
    typedef typename super::Matrix3T Matrix3T;
    typedef typename super::Matrix4T Matrix4T;

    using super::NOT_ENOUGH_POINTS;
    using super::UNEQUAL_NUMBER_OF_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_POINT_SIZE;

    EstimateProjectiveTransformation3D();

    EstimateProjectiveTransformation3D(const std::vector<Coordinate<T> > & source_points,
                                       const std::vector<Coordinate<T> > & target_points);

    EstimateProjectiveTransformation3D(const std::vector<Vector3T> & source_points,
                                       const std::vector<Vector3T> & target_points);

    EstimateProjectiveTransformation3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & source_points,
                                       const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & target_points);

    virtual ~EstimateProjectiveTransformation3D();

    virtual void compute();

    void setMaxIterations(int value);
    void setMaxRMS(value_type value);

private:

    int m_max_iterations;
    static const int MAX_ITERATIONS = 10;

    value_type m_max_RMS;
    static const value_type MAX_RMS;
};


template <class T>
const T EstimateProjectiveTransformation3D<T>::MAX_RMS = 0.1;


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new instance of this class.
  */

template <class T>
EstimateProjectiveTransformation3D<T>::EstimateProjectiveTransformation3D() :
    m_max_iterations(MAX_ITERATIONS),
    m_max_RMS(MAX_RMS)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   The points can either be plain 3D or 4D
  *                             \ref HomogeneousCoordinates "homogeneous"
  *                             coordinates.
  * \param [in] target_points   The points can either be plain 3D or 4D
  *                             \ref HomogeneousCoordinates "homogeneous"
  *                             coordinates.
  *
  * \brief Constructs a new instance of this class.
  *
  * No transformation estimation is done. To do so, please call compute().
  *
  * \throw ErrorObj If there are any coordinates other than 3D or 4D coordinates,
  *                 an error object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  *
  * \note The point sets must have the same cardinality, and the points must
  *       correspond to each other.
  */

template <class T>
EstimateProjectiveTransformation3D<T>::EstimateProjectiveTransformation3D(const std::vector<Coordinate<T> > & source_points,
                                                                          const std::vector<Coordinate<T> > & target_points) :
    m_max_iterations(MAX_ITERATIONS),
    m_max_RMS(MAX_RMS)
{
    super::setSourcePoints(source_points);
    super::setTargetPoints(target_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   3D coordinates.
  * \param [in] target_points   3D coordinates.
  *
  * \brief Constructs a new instance of this class.
  *
  * No transformation estimation is done. To do so, please call compute().
  *
  * \note The point sets must have the same cardinality, and the points must
  *       correspond to each other.
  */

template <class T>
EstimateProjectiveTransformation3D<T>::EstimateProjectiveTransformation3D(const std::vector<Vector3T> & source_points,
                                                                          const std::vector<Vector3T> & target_points) :
    m_max_iterations(MAX_ITERATIONS),
    m_max_RMS(MAX_RMS)
{
    super::setSourcePoints(source_points);
    super::setTargetPoints(target_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   4D \ref HomogeneousCoordinates "homogeneous" coordinates.
  * \param [in] target_points   4D \ref HomogeneousCoordinates "homogeneous" coordinates.
  *
  * \brief Constructs a new instance of this class.
  *
  * No transformation estimation is done. To do so, please call compute().
  *
  * \note The point sets must have the same cardinality, and the points must
  *       correspond to each other.
  */

template <class T>
EstimateProjectiveTransformation3D<T>::EstimateProjectiveTransformation3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & source_points,
                                                                          const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & target_points) :
    m_max_iterations(MAX_ITERATIONS),
    m_max_RMS(MAX_RMS)
{
    super::setSourcePoints(source_points);
    super::setTargetPoints(target_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Deletes the instance.
  */

template <class T>
EstimateProjectiveTransformation3D<T>::~EstimateProjectiveTransformation3D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Computes the transformation estimation.
  *
  * This function uses Newton's method (after Simpson) to estimate the sought
  * transformation matrix entries. For a more detailed explanation of the
  * algorithm, please have a look at the source code. (A link should be below.)
  *
  * The direction of the transformation is from the source points to the target
  * points: \f$ p_{target} = \cal{T} \; \{ p_{source} \} \f$.
  *
  * \throw ErrorObj If the point sets do not have the same cardinality, an error
  *                 object is thrown and its error code is set to
  *                 \c UNEQUAL_NUMBER_OF_POINTS.
  *
  * \throw ErrorObj If there are not enough points (at least four) to perform
  *                 the transformation estimation, an error object is thrown and
  *                 its error code is set to \c NOT_ENOUGH_POINTS.
  */

template <class T>
void EstimateProjectiveTransformation3D<T>::compute()
{
    if (super::m_source_points.size() != super::m_target_points.size())
    {
        ErrorObj error;
        error.setClassName("EstimateProjectiveTransformation3D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    if (super::m_source_points.cols() < 5 || super::m_target_points.cols() < 5)
    {
        ErrorObj error;
        error.setClassName("EstimateProjectiveTransformation3D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to compute transformation matrix.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    /*

    Explanation of the algorithm (in 2D):

    By using homogeneous coordinates an projective transformation can be described as follows:

        | a_11  a_12  a_13 |     | source_x |    | target_x' |
        | a_21  a_22  a_23 |  *  | source_y | =  | target_y' |
        | a_31  a_32  1    |     | 1        |    | target_z' |.   <-- note: target_z' is *not* known

    Then the sought target coordinates are:

        | target_x |     | target_x' / target_z' |
        | target_y |  =  | target_y' / target_z' |
        | target_z |     | 1                     |.

    The matrix A can be normalized such that component a_33 is equal to one, i.e. A := A' / a'_33, because of the
    subsequent normalisation step. Hence A has only 8 DoF. Since the normalisation step is nonlinear, we use
    Newton's method (after Simpson) to solve for the entries a_11, ..., a_32.

    The equation can be rewritten as a function f of the sought variables a_11, ..., a_32:

              | f_1 |                       1                        | a_11 * source_x + a_12 * source_y + a_13 |
        f  =  |     |  =  --------------------------------------  *  |                                          |
              | f_2 |      a_31 * source_x + a_32 * source_y + 1     | a_21 * source_x + a_22 * source_y + a_23 |.

    Then we try to find the zero-crossing of the residual (and we are done):

        residual(a_11, ..., a_32) = residual(A) := f(a_11, ..., a_32) - |target_x, target_y|^T.

    This can be done iteratively by computing

        A_new = A_old - Jacobian(residual)^(-1) * residual

    where

                   | c_3*source_x   c_3*source_y   c_3   0              0              0     -c_1*c_4*source_x  -c_1*c_4*source_y |
        Jacobian = |                                                                                                              |
                   | 0              0              0     c_3*source_x   c_3*source_y   c_3   -c_2*c_4*source_x  -c_2*c_4*source_y |

    and

        c_1 = a_11 * source_x + a_12 * source_y + a_13
        c_2 = a_21 * source_x + a_22 * source_y + a_23
        c_3 = 1 / (a_31 * source_x + a_31 * source_y + 1)
        c_4 = c_3^2.

    Instead of inverting the Jacobian directly we rewrite the above equation:

        delta_A := A_new - A_old = - Jacobian(residual)^(-1) * residual

    and solve the system of linear equations with respect to delta_A:

        delta_A * Jacobian = -residual.

    By using the method of least squares (or the Moore-Penrose pseudoinverse) we get

        delta_A = - (Jacobian^T * Jacobian)^(-1) * (Jacobian^T * residual).

    Then (in each iteration step) the new solution vector becomes:

        A_new = A_old + delta_A.

    */

    const int number_points =  super::m_source_points.cols();

    // initialize solution matrix

    Matrix4T A;
    A.setIdentity();

    const ArrayXT & source_x = super::m_source_points.block(0, 0, 1, number_points).transpose();
    const ArrayXT & source_y = super::m_source_points.block(1, 0, 1, number_points).transpose();
    const ArrayXT & source_z = super::m_source_points.block(2, 0, 1, number_points).transpose();

    const ArrayXT & target_x = super::m_target_points.block(0, 0, 1, number_points).transpose();
    const ArrayXT & target_y = super::m_target_points.block(1, 0, 1, number_points).transpose();
    const ArrayXT & target_z = super::m_target_points.block(2, 0, 1, number_points).transpose();

    for (int i = 0; i < m_max_iterations; ++i)
    {
        // construct Jacobian matrix

        ArrayXT ones = ArrayXT::Ones(number_points);

        ArrayXT c_1(number_points, 1);
        ArrayXT c_2(number_points, 1);
        ArrayXT c_3(number_points, 1);
        ArrayXT c_4(number_points, 1);
        ArrayXT c_5(number_points, 1);

        c_1 = A(0, 0) * source_x + A(0, 1) * source_y + A(0, 2) * source_z + A(0, 3) * ones;
        c_2 = A(1, 0) * source_x + A(1, 1) * source_y + A(1, 2) * source_z + A(1, 3) * ones;
        c_3 = A(2, 0) * source_x + A(2, 1) * source_y + A(2, 2) * source_z + A(2, 3) * ones;
        c_4 = (A(3, 0) * source_x + A(3, 1) * source_y + A(3, 2) * source_z + ones).inverse();
        c_5 = c_4 * c_4;

        MatrixXT Jacobian(3 * number_points, 15);

        Jacobian.setZero();

        Jacobian.block(0 * number_points,  0, number_points, 1) = c_4 * source_x;
        Jacobian.block(0 * number_points,  1, number_points, 1) = c_4 * source_y;
        Jacobian.block(0 * number_points,  2, number_points, 1) = c_4 * source_z;
        Jacobian.block(0 * number_points,  3, number_points, 1) = c_4;
        Jacobian.block(0 * number_points, 12, number_points, 1) = -c_1 * c_5 * source_x;
        Jacobian.block(0 * number_points, 13, number_points, 1) = -c_1 * c_5 * source_y;
        Jacobian.block(0 * number_points, 14, number_points, 1) = -c_1 * c_5 * source_z;

        Jacobian.block(1 * number_points,  4, number_points, 1) = c_4 * source_x;
        Jacobian.block(1 * number_points,  5, number_points, 1) = c_4 * source_y;
        Jacobian.block(1 * number_points,  6, number_points, 1) = c_4 * source_z;
        Jacobian.block(1 * number_points,  7, number_points, 1) = c_4;
        Jacobian.block(1 * number_points, 12, number_points, 1) = -c_2 * c_5 * source_x;
        Jacobian.block(1 * number_points, 13, number_points, 1) = -c_2 * c_5 * source_y;
        Jacobian.block(1 * number_points, 14, number_points, 1) = -c_2 * c_5 * source_z;

        Jacobian.block(2 * number_points,  8, number_points, 1) = c_4 * source_x;
        Jacobian.block(2 * number_points,  9, number_points, 1) = c_4 * source_y;
        Jacobian.block(2 * number_points, 10, number_points, 1) = c_4 * source_z;
        Jacobian.block(2 * number_points, 11, number_points, 1) = c_4;
        Jacobian.block(2 * number_points, 12, number_points, 1) = -c_3 * c_5 * source_x;
        Jacobian.block(2 * number_points, 13, number_points, 1) = -c_3 * c_5 * source_y;
        Jacobian.block(2 * number_points, 14, number_points, 1) = -c_3 * c_5 * source_z;

        // compute residual vector

        VectorXT residual(3 * number_points);

        residual.block(0 * number_points, 0, number_points, 1)  = c_1 * c_4 - target_x;
        residual.block(1 * number_points, 0, number_points, 1)  = c_2 * c_4 - target_y;
        residual.block(2 * number_points, 0, number_points, 1)  = c_3 * c_4 - target_z;

        // compute difference vector

        VectorXT delta_A = - ((Jacobian.transpose() * Jacobian).inverse() * (Jacobian.transpose() * residual));

        // compute estimate of transformation matrix

        A(0, 0) += delta_A( 0);
        A(0, 1) += delta_A( 1);
        A(0, 2) += delta_A( 2);
        A(0, 3) += delta_A( 3);
        A(1, 0) += delta_A( 4);
        A(1, 1) += delta_A( 5);
        A(1, 2) += delta_A( 6);
        A(1, 3) += delta_A( 7);
        A(2, 0) += delta_A( 8);
        A(2, 1) += delta_A( 9);
        A(2, 2) += delta_A(10);
        A(2, 3) += delta_A(11);
        A(3, 0) += delta_A(12);
        A(3, 1) += delta_A(13);
        A(3, 2) += delta_A(14);

        // compute MSE as stop criterion

        Transform3D<T> transform(A);

        value_type RMS = 0;

        for (int n = 0; n < number_points; ++n)
        {
            Vector3T source = Vector3T(source_x(n), source_y(n), source_z(n));
            Vector3T target = Vector3T(target_x(n), target_y(n), target_z(n));

            RMS += (transform * source - target).squaredNorm();
        }

        using std::sqrt;

        RMS = sqrt(RMS / number_points);

        if (RMS < m_max_RMS) break;
    }

    super::m_transformation_matrix = A;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Sets the maximum number of iterations within Newton's method.
  *
  * Sets the maximum number of iterations within Newton's method which is used
  * to compute the transformation matrix.
  *
  * The default value is 0.1.
  *
  * \see setMaxRMS()
  */

template <class T>
void EstimateProjectiveTransformation3D<T>::setMaxIterations(int value)
{
    m_max_iterations = value;
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Sets the maximum root mean square (RMS) error value.
  *
  * Sets the maximum root mean square (RMS) error value within Newton's method
  * which is used to compute the transformation matrix. That is, the iteration
  * is stopped, if the RMS is below \p value.
  *
  * The default value is 10.
  *
  * \see setMaxIterations()
  */

template <class T>
void EstimateProjectiveTransformation3D<T>::setMaxRMS(value_type value)
{
    m_max_RMS = value;
}


} // namespace TRTK


#endif // ESTIMATE_PROJECTIVE_TRANSFORMATION_3D_HPP_1231333218
