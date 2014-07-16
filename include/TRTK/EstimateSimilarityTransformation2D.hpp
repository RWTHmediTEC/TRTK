/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.1.1 (2011-11-10)
*/

/** \file EstimateSimilarityTransformation2D.hpp
  * \brief This file contains the \ref TRTK::EstimateSimilarityTransformation2D
  *        "EstimateSimilarityTransformation2D" class.
  */


#ifndef ESTIMATE_SIMILARITY_TRANSFORMATION_2D_HPP_7322190897
#define ESTIMATE_SIMILARITY_TRANSFORMATION_2D_HPP_7322190897


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


/** \class EstimateSimilarityTransformation2D
  *
  * \brief Estimates a 2D similarity transformation from two point sets.
  *
  * This class estimates a 2D similarity transformation between two point sets
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
  *         \alpha \cos(\varphi)  &  -\alpha \sin(\varphi)  &  b_1  \\
  *         \alpha \sin(\varphi)  &   \alpha \cos(\varphi)  &  b_2  \\
  *         0                     &   0                     &  1
  *     \end{pmatrix}
  *     \begin{pmatrix}
  *         x_1  \\  x_2  \\  1
  *     \end{pmatrix}
  *
  *     \qquad \qquad \text{or} \qquad \qquad
  *
  *     y = \alpha R x + b
  *     \quad \wedge \quad
  *     RR^T = I
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
  * #include <TRTK/EstimateSimilarityTransformation2D.hpp>
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
  *     transform.translate(-1, -3).rotate(pi/2).scale(2, 2).translate(1, 3);
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
  *     EstimateSimilarityTransformation2D<double> EstimateSimilarityTransformation2D(source_points,
  *                                                                                   target_points);
  *     EstimateSimilarityTransformation2D.compute();
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
  *          << EstimateSimilarityTransformation2D.getTransformationMatrix() << endl << endl;
  *
  *     // Example of how to use the result.
  *
  *     Transform2D<double> transform2 = EstimateSimilarityTransformation2D.getTransformationMatrix();
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
  *       0.0000      -2.0000       7.0000
  *       2.0000       0.0000       1.0000
  *       0.0000       0.0000       1.0000
  *
  * Estimated transformation matrix:
  *
  *      -0.0000      -2.0000       7.0000
  *       2.0000      -0.0000       1.0000
  *       0.0000       0.0000       1.0000
  *
  * Source point 1: (1.0000, 2.0000)
  * Target point 1: (3.0000, 3.0000)
  * Target point 1: (3.0000, 3.0000)
  *
  * \endcode
  *
  * \see EstimateRigidTransformation2D, EstimateAffineTransformation2D,
  *      EstimateProjectiveTransformation2D
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date 2011-11-10
  */

template <class T>
class EstimateSimilarityTransformation2D : public EstimateTransformation2D<T>
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

    EstimateSimilarityTransformation2D();

    EstimateSimilarityTransformation2D(const std::vector<Coordinate<T> > & source_points,
                                       const std::vector<Coordinate<T> > & target_points);

    EstimateSimilarityTransformation2D(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & source_points,
                                       const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & target_points);

    EstimateSimilarityTransformation2D(const std::vector<Vector3T> & source_points,
                                       const std::vector<Vector3T> & target_points);

    virtual ~EstimateSimilarityTransformation2D();

    virtual void compute();

    Matrix2T getRotationMatrix() const;
    Vector2T getTranslationVector() const;

    value_type getRotationAngle() const;
    value_type getScalingFactor() const;

private:

    value_type rotation_angle;
    value_type scaling_factor;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new instance of this class.
  */

template <class T>
EstimateSimilarityTransformation2D<T>::EstimateSimilarityTransformation2D() :
    rotation_angle(0),
    scaling_factor(0)
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
EstimateSimilarityTransformation2D<T>::EstimateSimilarityTransformation2D(const std::vector<Coordinate<T> > & source_points,
                                                                          const std::vector<Coordinate<T> > & target_points) :
    rotation_angle(0),
    scaling_factor(0)

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
EstimateSimilarityTransformation2D<T>::EstimateSimilarityTransformation2D(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & source_points,
                                                                          const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & target_points) :
    rotation_angle(0),
    scaling_factor(0)
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
EstimateSimilarityTransformation2D<T>::EstimateSimilarityTransformation2D(const std::vector<Vector3T> & source_points,
                                                                          const std::vector<Vector3T> & target_points) :
    rotation_angle(0),
    scaling_factor(0)
{
    super::setSourcePoints(source_points);
    super::setTargetPoints(target_points);
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Deletes the instance.
  */

template <class T>
EstimateSimilarityTransformation2D<T>::~EstimateSimilarityTransformation2D()
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
void EstimateSimilarityTransformation2D<T>::compute()
{
    if (super::m_source_points.size() != super::m_target_points.size())
    {
        ErrorObj error;
        error.setClassName("EstimateSimilarityTransformation2D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    if (super::m_source_points.cols() < 2 || super::m_target_points.cols() < 2)
    {
        ErrorObj error;
        error.setClassName("EstimateSimilarityTransformation2D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to compute transformation matrix.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    /*

    Explanation of the algorithm:

    By using homogeneous coordinates a rigid transformation can be described as follows:

    | alpha*cos(phi)  -alpha*sin(phi)  t_x |     | source_x |    | target_x |
    | alpha*sin(phi)  alpha*cos(phi)   t_y |  *  | source_y | =  | target_y |
    | 0               0                1   |     | 1        |    | 1        |

    The equation can be rewritten as:

                                      | alpha * cos(phi) |
    | source_x  -source_y  1  0 |  *  | alpha * sin(phi) |  =  | target_x |
    | source_y  source_x   0  1 |     | t_x              |     | target_y |
                                      | t_Y              |

    Or in short:  A * x = b

    By using more and more points this leads to an over-determined system, which can
    be solved via the pseudo inverse (i.e. A becomes A = [A1 A2 A3 ...]^T):

    <==>    A * x = b
    <==>    A^T * A * x = A^T * b
    <==>    inverse(A^T * A) * A^T * A * x = x = inverse(A^T * A) * A^T * b

    Thus:

    x = pseudo_inverse * b = [inverse(A^T * A) * A^T] * b

    ==>     phi = atan2(x(1), x(0))                         (angle of rotation)
    ==>     alpha = sqrt(x(0) * x(0) + x(1) * x(1))         (scaling factor)

    */

    using std::atan2;
    using std::cos;
    using std::sin;

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

    // compute the angle of rotation as well as the scaling factor

    rotation_angle = atan2(x(1), x(0)); // cos(phi) and sin(phi) by oneself might diverge from each other

    using std::sqrt;
    scaling_factor = sqrt(x(0) * x(0) + x(1) * x(1));

    super::m_transformation_matrix(0, 0) = scaling_factor * cos(rotation_angle);
    super::m_transformation_matrix(0, 1) = scaling_factor * -sin(rotation_angle);
    super::m_transformation_matrix(0, 2) = x(2);
    super::m_transformation_matrix(1, 0) = scaling_factor * sin(rotation_angle);
    super::m_transformation_matrix(1, 1) = scaling_factor * cos(rotation_angle);
    super::m_transformation_matrix(1, 2) = x(3);
    super::m_transformation_matrix(2, 0) = 0;
    super::m_transformation_matrix(2, 1) = 0;
    super::m_transformation_matrix(2, 2) = 1;
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the rotation matrix as part of the sought transformation.
  *
  * \note The rotation matrix also comprises the scaling factor.
  *
  * \see compute(), getTranslationVector() and getTransformationMatrix()
  */

template <class T>
typename EstimateSimilarityTransformation2D<T>::Matrix2T EstimateSimilarityTransformation2D<T>::getRotationMatrix() const
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
typename EstimateSimilarityTransformation2D<T>::Vector2T EstimateSimilarityTransformation2D<T>::getTranslationVector() const
{
    return super::m_transformation_matrix.block(0, 2, 2, 1);
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the angle of rotation as part of the sought transformation.
  *
  * \see compute(), getScalingFactor()
  */

template <class T>
typename EstimateSimilarityTransformation2D<T>::value_type EstimateSimilarityTransformation2D<T>::getRotationAngle() const
{
    return rotation_angle;
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the scaling factor as part of the sought transformation.
  *
  * \see compute(), getRotationAngle()
  */

template <class T>
typename EstimateSimilarityTransformation2D<T>::value_type EstimateSimilarityTransformation2D<T>::getScalingFactor() const
{
    return scaling_factor;
}


} // namespace TRTK


#endif // ESTIMATE_SIMILARITY_TRANSFORMATION_2D_HPP_7322190897
