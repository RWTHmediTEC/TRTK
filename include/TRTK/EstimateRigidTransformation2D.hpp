/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.3 (2011-11-10)
*/

/** \file EstimateRigidTransformation2D.hpp
  * \brief This file contains the \ref TRTK::EstimateRigidTransformation2D
  *        "EstimateRigidTransformation2D" class.
  */


#ifndef ESTIMATE_RIGID_TRANSFORMATION_2D_HPP_4587327344
#define ESTIMATE_RIGID_TRANSFORMATION_2D_HPP_4587327344


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "EstimateTransformation2D.hpp"


namespace TRTK
{


/** \class EstimateRigidTransformation2D
  *
  * \brief Estimates a 2D rigid transformation from two point sets.
  *
  * This class estimates a 2D rigid transformation between two point sets
  * in terms of least squares. The point sets must have the same cardinality,
  * and the points must correspond to each other. There must be at least two
  * corresponding point pairs.
  *
  * The algorithm estimates a transformation matrix as shown below:
  *
  * \f[
  *     \begin{pmatrix}
  *         y_1  \\  y_2  \\  1
  *     \end{pmatrix}
  *     =
  *     \begin{pmatrix}
  *         \cos(\varphi)  &  -\sin(\varphi)  &  b_1  \\
  *         \sin(\varphi)  &  \cos(\varphi)   &  b_2  \\
  *         0              &  0               &  1
  *     \end{pmatrix}
  *     \begin{pmatrix}
  *         x_1  \\  x_2  \\  1
  *     \end{pmatrix}
  *
  *     \qquad \qquad \text{or} \qquad \qquad
  *
  *     y = Rx + b
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
  * Here is an more elaborate example to see, how to use the class:
  *
  * \code
  *
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Transform2D.hpp>
  * #include <TRTK/EstimateRigidTransformation2D.hpp>
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
  *     // Construct a transformation which rotates 90 degrees counter-clockwise
  *     // around the point of (1, 3).
  *
  *     Transform2D<double> transform;
  *     const double pi = Transform2D<double>::pi;
  *     transform.translate(-1, -3).rotate(pi/2).translate(1, 3);
  *
  *     // Construct two sets with source and target points, respectively.
  *
  *     vector<Coordinate<double> > source_points;
  *     vector<Coordinate<double> > target_points;
  *
  *     Coordinate<double> source_point1( 1,  2);
  *     Coordinate<double> source_point2( 3, -2);
  *     Coordinate<double> source_point3(-1,  1);
  *     Coordinate<double> source_point4( 2,  0);
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
  *     EstimateRigidTransformation2D<double> EstimateRigidTransformation2D(source_points,
  *                                                                         target_points);
  *     EstimateRigidTransformation2D.compute();
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
  *          << EstimateRigidTransformation2D.getTransformationMatrix() << endl << endl;
  *
  *     // Example of how to use the result.
  *
  *     Transform2D<double> transform2 = EstimateRigidTransformation2D.getTransformationMatrix();
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
  *       0.0000      -1.0000       4.0000
  *       1.0000       0.0000       2.0000
  *       0.0000       0.0000       1.0000
  *
  * Estimated transformation matrix:
  *
  *       0.0000      -1.0000       4.0000
  *       1.0000       0.0000       2.0000
  *       0.0000       0.0000       1.0000
  *
  * Source point 1: (1.0000, 2.0000)
  * Target point 1: (2.0000, 3.0000)
  * Target point 1: (2.0000, 3.0000)
  *
  * \endcode
  *
  * \see EstimateAffineTransformation2D, EstimateProjectiveTransformation2D
  *
  * \author Christoph Haenisch
  * \version 0.2.3
  * \date 2011-11-10
  */

template <class T>
class EstimateRigidTransformation2D : public EstimateTransformation2D<T>
{
private:

    typedef EstimateTransformation2D<T> super;

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

    EstimateRigidTransformation2D();

    EstimateRigidTransformation2D(const std::vector<Coordinate<T> > & source_points,
                                  const std::vector<Coordinate<T> > & target_points);

    EstimateRigidTransformation2D(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & source_points,
                                  const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & target_points);

    EstimateRigidTransformation2D(const std::vector<Vector3T> & source_points,
                                  const std::vector<Vector3T> & target_points);

    virtual ~EstimateRigidTransformation2D();

    virtual void compute();

    Matrix2T getRotationMatrix() const;
    Vector2T getTranslationVector() const;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new instance of this class.
  */

template <class T>
EstimateRigidTransformation2D<T>::EstimateRigidTransformation2D()
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
EstimateRigidTransformation2D<T>::EstimateRigidTransformation2D(const std::vector<Coordinate<T> > & source_points,
                                                                const std::vector<Coordinate<T> > & target_points)
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
EstimateRigidTransformation2D<T>::EstimateRigidTransformation2D(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & source_points,
                                                                const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & target_points)
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
EstimateRigidTransformation2D<T>::EstimateRigidTransformation2D(const std::vector<Vector3T> & source_points,
                                                                const std::vector<Vector3T> & target_points)
{
    super::setSourcePoints(source_points);
    super::setTargetPoints(target_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Deletes the instance.
  */

template <class T>
EstimateRigidTransformation2D<T>::~EstimateRigidTransformation2D()
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
  * \throw ErrorObj If there are not enough points (at least two) to perform the
  *                 transformation estimation, an error object is thrown and its
  *                 error code is set to \c NOT_ENOUGH_POINTS.
  */

template <class T>
void EstimateRigidTransformation2D<T>::compute()
{
    if (super::m_source_points.size() != super::m_target_points.size())
    {
        ErrorObj error;
        error.setClassName("EstimateRigidTransformation2D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    if (super::m_source_points.cols() < 2 || super::m_target_points.cols() < 2)
    {
        ErrorObj error;
        error.setClassName("EstimateRigidTransformation2D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to compute transformation matrix.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    /*

    Explanation of the algorithm:

    By using homogeneous coordinates a rigid transformation can be described as follows:

    | cos phi  -sin phi  t_x |     | source_x |    | target_x |
    | sin phi  cos phi   t_y |  *  | source_y | =  | target_y |
    | 0        0         1   |     | 1        |    | 1        |

    The equation can be rewritten as:

                                      | cos phi |
    | source_x  -source_y  1  0 |  *  | sin phi |  =  | target_x |
    | source_y  source_x   0  1 |     | t_x     |     | target_y |
                                      | t_Y     |

    Or in short:  A * x = b

    By using more and more points this leads to an over-determined system, which can
    be solved via the pseudo inverse (i.e. A becomes A = [A1 A2 A3 ...]^T):

    <==>    A * x = b
    <==>    A^T * A * x = A^T * b
    <==>    inverse(A^T * A) * A^T * A * x = x = inverse(A^T * A) * A^T * b

    Thus:

    x = pseudo_inverse * b = [inverse(A^T * A) * A^T] * b

    */

    const int number_points =  super::m_source_points.cols();

    // fill matrix A with the above described entries

    MatrixXT A(2*number_points, 4);

    A.setZero();
    A.block(0*number_points, 0, number_points, 1) =  super::m_source_points.block(0, 0, 1, number_points).transpose();
    A.block(0*number_points, 1, number_points, 1) = -super::m_source_points.block(1, 0, 1, number_points).transpose();
    A.block(0*number_points, 2, number_points, 1) =  VectorXT::Ones(number_points);
    A.block(1*number_points, 0, number_points, 1) =  super::m_source_points.block(1, 0, 1, number_points).transpose();
    A.block(1*number_points, 1, number_points, 1) =  super::m_source_points.block(0, 0, 1, number_points).transpose();
    A.block(1*number_points, 3, number_points, 1) =  VectorXT::Ones(number_points);

    // fill vector b

    VectorXT b(2*number_points, 1);
    b.block(0*number_points, 0, number_points, 1) = super::m_target_points.block(0, 0, 1, number_points).transpose();
    b.block(1*number_points, 0, number_points, 1) = super::m_target_points.block(1, 0, 1, number_points).transpose();

    // compute pseudo inverse

    MatrixXT pseudo_inverse = (A.transpose() * A).inverse() * A.transpose();

    // compute vector x whose components are/contain the sought transformation entries

    VectorXT x = pseudo_inverse * b;

    using std::atan2;
    using std::cos;
    using std::sin;

    T phi = atan2(x(1), x(0)); // cos(phi) and sin(phi) by oneself might diverge from each other

    super::m_transformation_matrix(0, 0) = cos(phi);
    super::m_transformation_matrix(0, 1) = -sin(phi);
    super::m_transformation_matrix(0, 2) = x(2);
    super::m_transformation_matrix(1, 0) = sin(phi);
    super::m_transformation_matrix(1, 1) = cos(phi);
    super::m_transformation_matrix(1, 2) = x(3);
    super::m_transformation_matrix(2, 0) = 0;
    super::m_transformation_matrix(2, 1) = 0;
    super::m_transformation_matrix(2, 2) = 1;
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the rotation matrix as part of the sought transformation.
  *
  * \see compute(), getTranslationVector() and getTransformationMatrix()
  */

template <class T>
typename EstimateRigidTransformation2D<T>::Matrix2T EstimateRigidTransformation2D<T>::getRotationMatrix() const
{
    return super::m_transformation_matrix.block(0, 0, 2, 2);
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the translation vector as part of the sought transformation.
  *
  * \see compute(), getRotationMatrix() and getTransformationMatrix()
  */

template <class T>
typename EstimateRigidTransformation2D<T>::Vector2T EstimateRigidTransformation2D<T>::getTranslationVector() const
{
    return super::m_transformation_matrix.block(0, 2, 2, 1);
}


} // namespace TRTK


#endif // ESTIMATE_RIGID_TRANSFORMATION_2D_HPP_4587327344
