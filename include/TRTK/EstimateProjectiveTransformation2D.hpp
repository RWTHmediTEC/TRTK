/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.4 (2013-08-20)
*/

/** \file EstimateProjectiveTransformation2D.hpp
  * \brief This file contains the \ref TRTK::EstimateProjectiveTransformation2D
  *        "EstimateProjectiveTransformation2D" class.
  */


#ifndef ESTIMATE_PROJECTIVE_TRANSFORMATION_2D_HPP_7460134016
#define ESTIMATE_PROJECTIVE_TRANSFORMATION_2D_HPP_7460134016


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "EstimateTransformation2D.hpp"


namespace TRTK
{


/** \class EstimateProjectiveTransformation2D
  *
  * \brief Estimates a 2D projective transformation from two point sets.
  *
  * This class estimates a 2D projective transformation between two point sets
  * by using Newton's method. The point sets must have the same cardinality,
  * and the points must correspond to each other. There must be at least four
  * corresponding point pairs.
  *
  * The algorithm estimates a transformation matrix as shown below:
  *
  * \f[
  *     \begin{pmatrix}
  *         y_1  \\  y_2
  *     \end{pmatrix}
  *     =
  *     \frac{1}{y'_3}
  *     \begin{pmatrix}
  *         y'_1  \\  y'_2
  *     \end{pmatrix}
  *
  *     \quad \wedge \quad
  *
  *     \begin{pmatrix}
  *         y'_1  \\  y'_2  \\  y'_3
  *     \end{pmatrix}
  *     =
  *     \begin{pmatrix}
  *         a_{11}  &  a_{12}  &  a_{13}  \\
  *         a_{21}  &  a_{22}  &  a_{23}  \\
  *         a_{31}  &  a_{32}  &  a_{33}
  *     \end{pmatrix}
  *     \begin{pmatrix}
  *         x_1  \\  x_2  \\  1
  *     \end{pmatrix}
  * \f]
  *
  * The source points are the set of all \f$ x \f$ and the target points are
  * the set of all \f$ y \f$.
  *
  * \note
  *     - To obtain stable estimation results, at least six point pairs should
  *       be given.
  *     - Some functions might throw an \ref ErrorObj "error object". See the
  *       appropriate function for more details.
  *     - The template parameter \p T is the internally used value type and
  *       must be a floating number type.
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
  * #include <TRTK/Transform2D.hpp>
  * #include <TRTK/EstimateProjectiveTransformation2D.hpp>
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
  *     Transform2D<double> transform;
  *     transform.a12() = 1;
  *     transform.a13() = 3;
  *     transform.a21() = 2;
  *     transform.a31() = 3;
  *     transform.a33() = 1;
  *
  *     // Construct two sets with source and target points, respectively.
  *     // Add some noise to the target points.
  *
  *     vector<Coordinate<double> > source_points;
  *     vector<Coordinate<double> > target_points;
  *
  *     Coordinate<double> source_point1( 1,  2);
  *     Coordinate<double> source_point2( 3, -2);
  *     Coordinate<double> source_point3(-1,  2);
  *     Coordinate<double> source_point4( 2,  0);
  *     Coordinate<double> source_point5(-1, -1);
  *     Coordinate<double> source_point6( 3,  3);
  *
  *     source_points.push_back(source_point1);
  *     source_points.push_back(source_point2);
  *     source_points.push_back(source_point3);
  *     source_points.push_back(source_point4);
  *     source_points.push_back(source_point5);
  *     source_points.push_back(source_point6);
  *
  *     Coordinate<double> target_point1 = transform * source_point1 + 0.1 * double(std::rand()) / RAND_MAX;
  *     Coordinate<double> target_point2 = transform * source_point2 + 0.1 * double(std::rand()) / RAND_MAX;
  *     Coordinate<double> target_point3 = transform * source_point3 + 0.1 * double(std::rand()) / RAND_MAX;
  *     Coordinate<double> target_point4 = transform * source_point4 + 0.1 * double(std::rand()) / RAND_MAX;
  *     Coordinate<double> target_point5 = transform * source_point5 + 0.1 * double(std::rand()) / RAND_MAX;
  *     Coordinate<double> target_point6 = transform * source_point6 + 0.1 * double(std::rand()) / RAND_MAX;
  *
  *     target_points.push_back(target_point1);
  *     target_points.push_back(target_point2);
  *     target_points.push_back(target_point3);
  *     target_points.push_back(target_point4);
  *     target_points.push_back(target_point5);
  *     target_points.push_back(target_point6);
  *
  *     // Perform the transformation estimation.
  *
  *     EstimateProjectiveTransformation2D<double> estimateProjectiveTransformation2D(source_points,
  *                                                                                   target_points);
  *     estimateProjectiveTransformation2D.compute();
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
  *          << estimateProjectiveTransformation2D.getTransformationMatrix() << endl << endl;
  *
  *     // Example of how to use the result.
  *
  *     Transform2D<double> transform2 = estimateProjectiveTransformation2D.getTransformationMatrix();
  *
  *     cout << "Source point 1:               " << source_point1 << endl
  *          << "Target point 1 (original):    " << target_point1 << endl
  *          << "Target point 1 (transformed): " << transform2 * source_point1 << endl << endl;
  *
  *     cout << "RMS: " << estimateProjectiveTransformation2D.getRMS() << endl;
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
  * 1.0000 1.0000 3.0000
  * 2.0000 1.0000 0.0000
  * 3.0000 0.0000 1.0000
  *
  * Estimated transformation matrix:
  *
  *     1.0286     0.7833     2.6236
  *     1.7938     0.7647     0.2280
  *     2.5321    -0.0311     1.0000
  *
  * Source point 1:               (1.0000, 2.0000)
  * Target point 1 (original):    (1.5001, 1.0001)
  * Target point 1 (transformed): (1.5039, 1.0234)
  *
  * RMS: 0.0458
  *
  * \endcode
  *
  * \see EstimateRigidTransformation2D, EstimateProjectiveTransformation2D
  *
  * \bug Depending on the compiler this function may be numerically unstable.
  *      It has to be investigated what is the reason for it.
  *
  * \todo Check the estimation algorithm for numerical issues.
  *
  * \author Christoph Haenisch
  * \version 0.2.4
  * \date 2019-07-12
  */

template <class T>
class EstimateProjectiveTransformation2D : public EstimateTransformation2D<T>
{
private:

    typedef EstimateTransformation2D<T> super;

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

    EstimateProjectiveTransformation2D();

    EstimateProjectiveTransformation2D(const std::vector<Coordinate<T> > & source_points,
                                       const std::vector<Coordinate<T> > & target_points);

    EstimateProjectiveTransformation2D(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & source_points,
                                       const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & target_points);

    EstimateProjectiveTransformation2D(const std::vector<Vector3T> & source_points,
                                       const std::vector<Vector3T> & target_points);

    virtual ~EstimateProjectiveTransformation2D();

    void compute();

    void setMaxIterations(int value);
    void setMaxRMS(value_type value);

private:

    int m_max_iterations;
    static const int MAX_ITERATIONS = 10;

    value_type m_max_RMS;
    static const value_type MAX_RMS;
};


template <class T>
const T EstimateProjectiveTransformation2D<T>::MAX_RMS = 0.1;


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new instance of this class.
  */

template <class T>
EstimateProjectiveTransformation2D<T>::EstimateProjectiveTransformation2D() :
    m_max_iterations(MAX_ITERATIONS),
    m_max_RMS(MAX_RMS)
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   The points can either be plain 2D or 3D
  *                             \ref HomogeneousCoordinates "homogeneous"
  *                             coordinates.
  * \param [in] target_points   The points can either be plain 2D or 3D
  *                             \ref HomogeneousCoordinates "homogeneous"
  *                             coordinates.
  *
  * \brief Constructs a new instance of this class.
  *
  * No transformation estimation is done. To do so, please call compute().
  *
  * \throw ErrorObj If there are any coordinates other than 2D or 3D coordinates,
  *                 an error object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  *
  * \note The point sets must have the same cardinality, and the points must
  *       correspond to each other.
  */

template <class T>
EstimateProjectiveTransformation2D<T>::EstimateProjectiveTransformation2D(const std::vector<Coordinate<T> > & source_points,
                                                                          const std::vector<Coordinate<T> > & target_points) :
    m_max_iterations(MAX_ITERATIONS),
    m_max_RMS(MAX_RMS)
{
    super::setSourcePoints(source_points);
    super::setTargetPoints(target_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   2D coordinates.
  * \param [in] target_points   2D coordinates.
  *
  * \brief Constructs a new instance of this class.
  *
  * No transformation estimation is done. To do so, please call compute().
  *
  * \note The point sets must have the same cardinality, and the points must
  *       correspond to each other.
  */

template <class T>
EstimateProjectiveTransformation2D<T>::EstimateProjectiveTransformation2D(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & source_points,
                                                                          const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & target_points) :
    m_max_iterations(MAX_ITERATIONS),
    m_max_RMS(MAX_RMS)
{
    super::setSourcePoints(source_points);
    super::setTargetPoints(target_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   3D \ref HomogeneousCoordinates "homogeneous" coordinates.
  * \param [in] target_points   3D \ref HomogeneousCoordinates "homogeneous" coordinates.
  *
  * \brief Constructs a new instance of this class.
  *
  * No transformation estimation is done. To do so, please call compute().
  *
  * \note The point sets must have the same cardinality, and the points must
  *       correspond to each other.
  */

template <class T>
EstimateProjectiveTransformation2D<T>::EstimateProjectiveTransformation2D(const std::vector<Vector3T> & source_points,
                                                                          const std::vector<Vector3T> & target_points) :
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
EstimateProjectiveTransformation2D<T>::~EstimateProjectiveTransformation2D()
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
  * \throw ErrorObj If there are not enough points (at least three) to perform the
  *                 transformation estimation, an error object is thrown and its
  *                 error code is set to \c NOT_ENOUGH_POINTS.
  */

template <class T>
void EstimateProjectiveTransformation2D<T>::compute()
{
    if (super::m_source_points.size() != super::m_target_points.size())
    {
        ErrorObj error;
        error.setClassName("EstimateProjectiveTransformation2D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    if (super::m_source_points.cols() < 4 || super::m_target_points.cols() < 4)
    {
        ErrorObj error;
        error.setClassName("EstimateProjectiveTransformation2D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to compute transformation matrix.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    /*

    Explanation of the algorithm:

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

        residual(a_11, ..., a_32) := f(a_11, ..., a_32) - |target_x, target_y|^T.

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

    Matrix3T A;
    A.setIdentity();

    const ArrayXT & source_x = super::m_source_points.block(0, 0, 1, number_points).transpose();
    const ArrayXT & source_y = super::m_source_points.block(1, 0, 1, number_points).transpose();

    const ArrayXT & target_x = super::m_target_points.block(0, 0, 1, number_points).transpose();
    const ArrayXT & target_y = super::m_target_points.block(1, 0, 1, number_points).transpose();

    for (int i = 0; i < m_max_iterations; ++i)
    {

        // construct Jacobian matrix

        ArrayXT ones = ArrayXT::Ones(number_points);

        ArrayXT c_1(number_points, 1);
        ArrayXT c_2(number_points, 1);
        ArrayXT c_3(number_points, 1);
        ArrayXT c_4(number_points, 1);

        c_1 = A(0, 0) * source_x + A(0, 1) * source_y + A(0, 2) * ones;
        c_2 = A(1, 0) * source_x + A(1, 1) * source_y + A(1, 2) * ones;
        c_3 = (A(2, 0) * source_x + A(2, 1) * source_y + ones).inverse();
        c_4 = c_3 * c_3;

        MatrixXT Jacobian(2 * number_points, 8);

        Jacobian.setZero();

        Jacobian.block(0 * number_points, 0, number_points, 1) = c_3 * source_x;
        Jacobian.block(0 * number_points, 1, number_points, 1) = c_3 * source_y;
        Jacobian.block(0 * number_points, 2, number_points, 1) = c_3;
        Jacobian.block(0 * number_points, 6, number_points, 1) = -c_1 * c_4 * source_x;
        Jacobian.block(0 * number_points, 7, number_points, 1) = -c_1 * c_4 * source_y;

        Jacobian.block(1 * number_points, 3, number_points, 1) = c_3 * source_x;
        Jacobian.block(1 * number_points, 4, number_points, 1) = c_3 * source_y;
        Jacobian.block(1 * number_points, 5, number_points, 1) = c_3;
        Jacobian.block(1 * number_points, 6, number_points, 1) = -c_2 * c_4 * source_x;
        Jacobian.block(1 * number_points, 7, number_points, 1) = -c_2 * c_4 * source_y;

        // compute residual vector

        VectorXT residual(2 * number_points);

        residual.block(0 * number_points, 0, number_points, 1)  = c_1 * c_3 - target_x;
        residual.block(1 * number_points, 0, number_points, 1)  = c_2 * c_3 - target_y;

        // compute difference vector

        VectorXT delta_A = - ((Jacobian.transpose() * Jacobian).inverse() * (Jacobian.transpose() * residual));

        // compute estimate of transformation matrix

        A(0, 0) += delta_A(0);
        A(0, 1) += delta_A(1);
        A(0, 2) += delta_A(2);
        A(1, 0) += delta_A(3);
        A(1, 1) += delta_A(4);
        A(1, 2) += delta_A(5);
        A(2, 0) += delta_A(6);
        A(2, 1) += delta_A(7);

        // compute MSE as stop criterion

        Transform2D<T> transform(A);

        value_type RMS = 0;

        for (int n = 0; n < number_points; ++n)
        {
            Vector2T source = Vector2T(source_x(n), source_y(n));
            Vector2T target = Vector2T(target_x(n), target_y(n));

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
void EstimateProjectiveTransformation2D<T>::setMaxIterations(int value)
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
void EstimateProjectiveTransformation2D<T>::setMaxRMS(value_type value)
{
    m_max_RMS = value;
}


} // namespace TRTK


#endif // ESTIMATE_PROJECTIVE_TRANSFORMATION_2D_HPP_7460134016
