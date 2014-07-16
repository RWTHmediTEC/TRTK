/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.3 (2011-11-10)
*/

/** \file EstimateAffineTransformation3D.hpp
  * \brief This file contains the \ref TRTK::EstimateAffineTransformation3D
  *        "EstimateAffineTransformation3D" class.
  */


#ifndef ESTIMATE_AFFINE_TRANSFORMATION_3D_HPP_1790843183
#define ESTIMATE_AFFINE_TRANSFORMATION_3D_HPP_1790843183


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "EstimateTransformation3D.hpp"


namespace TRTK
{


/** \class EstimateAffineTransformation3D
  *
  * \brief Estimates a 3D affine transformation from two point sets.
  *
  * This class estimates a 3D affine transformation between two point sets
  * in terms of least squares. The point sets must have the same cardinality,
  * and the points must correspond to each other. There must be at least four
  * corresponding point pairs.
  *
  * The algorithm estimates a transformation matrix as shown below:
  *
  * \f[
  *     \begin{pmatrix}
  *         y_1  \\  y_2  \\  y_3  \\  1
  *     \end{pmatrix}
  *     =
  *     \begin{pmatrix}
  *         a_{11}  &  a_{12}  &  a_{13}  &  b_1  \\
  *         a_{21}  &  a_{22}  &  a_{23}  &  b_2  \\
  *         a_{31}  &  a_{32}  &  a_{33}  &  b_3  \\
  *         0       &  0       &  0       &  1
  *     \end{pmatrix}
  *     \begin{pmatrix}
  *         x_1  \\  x_2  \\  x_3  \\  1
  *     \end{pmatrix}
  *
  *     \qquad \qquad \text{or} \qquad \qquad
  *
  *     y = Ax + b
  * \f]
  *
  * The source points are the set of all \f$ x \f$ and the target points are
  * the set of all \f$ y \f$.
  *
  * \note Some functions might throw an \ref ErrorObj "error object". See the
  *       appropriate function for more details.
  *
  * </p>
  *
  * \note For a more detailed explanation of the algorithm, please have a look
  *       at the source code.
  *
  * Here is an more elaborate example to see, how to use the class:
  *
  * \code
  *
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Transform3D.hpp>
  * #include <TRTK/EstimateAffineTransformation3D.hpp>
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
  *     // Construct an affine transformation.
  *
  *     Transform3D<double> transform;
  *     transform.a12() =  1;
  *     transform.a14() =  1;
  *     transform.a31() =  2;
  *     transform.a34() =  3;
  *
  *     // Construct two sets with source and target points, respectively.
  *
  *     vector<Coordinate<double> > source_points;
  *     vector<Coordinate<double> > target_points;
  *
  *     Coordinate<double> source_point1( 1,  2,  0);
  *     Coordinate<double> source_point2( 3, -2,  0);
  *     Coordinate<double> source_point3(-1,  1,  1);
  *     Coordinate<double> source_point4( 2,  0, -1);
  *
  *     source_points.push_back(source_point1);
  *     source_points.push_back(source_point2);
  *     source_points.push_back(source_point3);
  *     source_points.push_back(source_point4);
  *
  *     Coordinate<double> target_point1 = transform * source_point1;
  *     Coordinate<double> target_point2 = transform * source_point2;
  *     Coordinate<double> target_point3 = transform * source_point3;
  *     Coordinate<double> target_point4 = transform * source_point4;
  *
  *     target_points.push_back(target_point1);
  *     target_points.push_back(target_point2);
  *     target_points.push_back(target_point3);
  *     target_points.push_back(target_point4);
  *
  *     // Perform the transformation estimation.
  *
  *     EstimateAffineTransformation3D<double> estimateAffineTransformation3D(source_points,
  *                                                                           target_points);
  *     estimateAffineTransformation3D.compute();
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
  *          << estimateAffineTransformation3D.getTransformationMatrix() << endl << endl;
  *
  *     // Example of how to use the result.
  *
  *     Transform3D<double> transform2 = estimateAffineTransformation3D.getTransformationMatrix();
  *
  *     cout << "Source point 1: " << source_point1 << endl
  *          << "Target point 1: " << target_point1 << endl
  *          << "Target point 1: " << transform2 * source_point1 << endl;
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
  * 1.0000 1.0000 0.0000 1.0000
  * 0.0000 1.0000 0.0000 0.0000
  * 2.0000 0.0000 1.0000 3.0000
  * 0.0000 0.0000 0.0000 1.0000
  *
  * Estimated transformation matrix:
  *
  *        1.0000        1.0000       -0.0000        1.0000
  *       -0.0000        1.0000        0.0000        0.0000
  *        2.0000        0.0000        1.0000        3.0000
  *        0.0000        0.0000        0.0000        1.0000
  *
  * Source point 1: (1.0000, 2.0000, 0.0000)
  * Target point 1: (4.0000, 2.0000, 5.0000)
  * Target point 1: (4.0000, 2.0000, 5.0000)
  *
  * \endcode
  *
  * \see EstimateRigidTransformation3D, EstimateProjectiveTransformation3D
  *
  * \author Christoph Haenisch
  * \version 0.2.3
  * \date 2011-11-10
  */

template <class T>
class EstimateAffineTransformation3D : public EstimateTransformation3D<T>
{
private:

    typedef EstimateTransformation3D<T> super;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;

    typedef typename super::VectorXT VectorXT;
    typedef typename super::MatrixXT MatrixXT;
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

    EstimateAffineTransformation3D();

    EstimateAffineTransformation3D(const std::vector<Coordinate<T> > & source_points,
                                   const std::vector<Coordinate<T> > & target_points);

    EstimateAffineTransformation3D(const std::vector<Vector3T> & source_points,
                                   const std::vector<Vector3T> & target_points);

    EstimateAffineTransformation3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & source_points,
                                   const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & target_points);

    virtual ~EstimateAffineTransformation3D();

    virtual void compute();

    Matrix3T getLinearTransformationMatrix() const;
    Vector3T getTranslationVector() const;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new instance of this class.
  */

template <class T>
EstimateAffineTransformation3D<T>::EstimateAffineTransformation3D()
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
EstimateAffineTransformation3D<T>::EstimateAffineTransformation3D(const std::vector<Coordinate<T> > & source_points,
                                                                  const std::vector<Coordinate<T> > & target_points)
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
EstimateAffineTransformation3D<T>::EstimateAffineTransformation3D(const std::vector<Vector3T> & source_points,
                                                                  const std::vector<Vector3T> & target_points)
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
EstimateAffineTransformation3D<T>::EstimateAffineTransformation3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & source_points,
                                                                  const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & target_points)
{
    super::setSourcePoints(source_points);
    super::setTargetPoints(target_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Deletes the instance.
  */

template <class T>
EstimateAffineTransformation3D<T>::~EstimateAffineTransformation3D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Computes the transformation estimation.
  *
  * The transformation estimation uses a least square scheme. For a more
  * detailed explanation of the algorithm, please have a look at the source
  * code. (A link should be below.)
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
void EstimateAffineTransformation3D<T>::compute()
{
    if (super::m_source_points.size() != super::m_target_points.size())
    {
        ErrorObj error;
        error.setClassName("EstimateAffineTransformation3D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    if (super::m_source_points.cols() < 4 || super::m_target_points.cols() < 4)
    {
        ErrorObj error;
        error.setClassName("EstimateAffineTransformation3D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to compute transformation matrix.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    /*

    Explanation of the algorithm (in 2D):

    By using homogeneous coordinates an affine transformation can be described as follows:

    | t_11  t_12  t_13 |     | source_x |    | target_x |
    | t_21  t_22  t_23 |  *  | source_y | =  | target_y |
    | 0     0     1    |     | 1        |    | 1        |

    The equation can be rewritten as:
                                                         | t_11 |
                                                         | t_12 |
    | source_x  source_y  1  0         0         0 |  *  | t_13 |  =  | target_x |
    | 0         0         0  source_x  source_y  1 |     | t_21 |     | target_y |
                                                         | t_22 |
                                                         | t_23 |

    Or in short:  A * x = b

    By using more and more points this leads to an over-determined system, which can
    be solved via the pseudo inverse:

    <==>    A * x = b
    <==>    A^T * A * x = A^T * b
    <==>    inverse(A^T * A) * A^T * A * x = x = inverse(A^T * A) * A^T * b

    Thus:

    x = pseudo_inverse * b = [ inverse(A^T * A) * A^T ] * b

    */

    const int number_points =  super::m_source_points.cols();

    // fill matrix A with the above described entries

    MatrixXT A(3 * number_points, 12);

    const VectorXT ones = VectorXT::Ones(number_points);

    A.setZero();
    A.block(0 * number_points,  0, number_points, 1) = super::m_source_points.block(0, 0, 1, number_points).transpose();
    A.block(0 * number_points,  1, number_points, 1) = super::m_source_points.block(1, 0, 1, number_points).transpose();
    A.block(0 * number_points,  2, number_points, 1) = super::m_source_points.block(2, 0, 1, number_points).transpose();
    A.block(0 * number_points,  3, number_points, 1) = ones;
    A.block(1 * number_points,  4, number_points, 1) = super::m_source_points.block(0, 0, 1, number_points).transpose();
    A.block(1 * number_points,  5, number_points, 1) = super::m_source_points.block(1, 0, 1, number_points).transpose();
    A.block(1 * number_points,  6, number_points, 1) = super::m_source_points.block(2, 0, 1, number_points).transpose();
    A.block(1 * number_points,  7, number_points, 1) = ones;
    A.block(2 * number_points,  8, number_points, 1) = super::m_source_points.block(0, 0, 1, number_points).transpose();
    A.block(2 * number_points,  9, number_points, 1) = super::m_source_points.block(1, 0, 1, number_points).transpose();
    A.block(2 * number_points, 10, number_points, 1) = super::m_source_points.block(2, 0, 1, number_points).transpose();
    A.block(2 * number_points, 11, number_points, 1) = ones;

    // fill vector b

    VectorXT b(3 * number_points, 1);
    b.block(0 * number_points, 0, number_points, 1) = super::m_target_points.block(0, 0, 1, number_points).transpose();
    b.block(1 * number_points, 0, number_points, 1) = super::m_target_points.block(1, 0, 1, number_points).transpose();
    b.block(2 * number_points, 0, number_points, 1) = super::m_target_points.block(2, 0, 1, number_points).transpose();

    // compute pseudo inverse

    MatrixXT pseudo_inverse = (A.transpose() * A).inverse() * A.transpose();

    // compute vector x whose components are the sought transformation entries

    VectorXT x = pseudo_inverse * b;

    super::m_transformation_matrix(0, 0) = x(0);
    super::m_transformation_matrix(0, 1) = x(1);
    super::m_transformation_matrix(0, 2) = x(2);
    super::m_transformation_matrix(0, 3) = x(3);
    super::m_transformation_matrix(1, 0) = x(4);
    super::m_transformation_matrix(1, 1) = x(5);
    super::m_transformation_matrix(1, 2) = x(6);
    super::m_transformation_matrix(1, 3) = x(7);
    super::m_transformation_matrix(2, 0) = x(8);
    super::m_transformation_matrix(2, 1) = x(9);
    super::m_transformation_matrix(2, 2) = x(10);
    super::m_transformation_matrix(2, 3) = x(11);
    super::m_transformation_matrix(3, 0) = 0;
    super::m_transformation_matrix(3, 1) = 0;
    super::m_transformation_matrix(3, 2) = 0;
    super::m_transformation_matrix(3, 3) = 1;
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the linear transformation matrix as part of the sought transformation.
  *
  * \see compute(), getTranslationVector() and getTransformationMatrix()
  */

template <class T>
typename EstimateAffineTransformation3D<T>::Matrix3T EstimateAffineTransformation3D<T>::getLinearTransformationMatrix() const
{
    return super::m_transformation_matrix.block(0, 0, 3, 3);
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the translation vector as part of the sought transformation.
  *
  * \see compute(), getLinearTransformationMatrix() and getTransformationMatrix()
  */

template <class T>
typename EstimateAffineTransformation3D<T>::Vector3T EstimateAffineTransformation3D<T>::getTranslationVector() const
{
    return super::m_transformation_matrix.block(0, 3, 3, 1);
}


} // namespace TRTK


#endif // ESTIMATE_AFFINE_TRANSFORMATION_3D_HPP_1790843183
