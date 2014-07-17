/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.3.3 (2012-10-12)
*/

/** \file EstimateRigidTransformation3D.hpp
  * \brief This file contains the \ref TRTK::EstimateRigidTransformation3D
  *        "EstimateRigidTransformation3D" class.
  */


#ifndef ESTIMATE_RIGID_TRANSFORMATION_3D_HPP_1230814474
#define ESTIMATE_RIGID_TRANSFORMATION_3D_HPP_1230814474


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <Eigen/SVD>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "EstimateTransformation3D.hpp"


namespace TRTK
{


/** \class EstimateRigidTransformation3D
  *
  * \brief Estimates a 3D rigid transformation from two point sets.
  *
  * This class estimates a 3D rigid transformation between two point sets
  * in terms of least squares. The point sets must have the same cardinality,
  * and the points must correspond to each other. There must be at least two
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
  *         r_{11}  &  r_{12}  &  r_{13}  &  b_1  \\
  *         r_{21}  &  r_{22}  &  r_{23}  &  b_2  \\
  *         r_{31}  &  r_{32}  &  r_{33}  &  b_3  \\
  *         0       &  0       &  0       &  1
  *     \end{pmatrix}
  *     \begin{pmatrix}
  *         x_1  \\  x_2  \\  x_3  \\  1
  *     \end{pmatrix}
  *
  *     \qquad \qquad \text{or} \qquad \qquad
  *
  *     y = Rx + b
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
  * \note The transformation estimation is a least square scheme which decomposes
  *       a correlation matrix between the two point sets using the singular value
  *       decomposition. For a more detailed explanation of the algorithm, please
  *       have a look at the source code.
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
  * #include <TRTK/EstimateRigidTransformation3D.hpp>
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
  *     // in the x-y plane with a center of rotation of (1, 3, 0).
  *
  *     Transform3D<double> transform;
  *     const double pi = Transform3D<double>::pi;
  *     transform.translate(-1, -3, 0).rotateZ(pi/2).translate(1, 3, 0);
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
  *     EstimateRigidTransformation3D<double> estimateRigidTransformation3D(source_points,
  *                                                                         target_points);
  *     estimateRigidTransformation3D.compute();
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
  *          << estimateRigidTransformation3D.getTransformationMatrix() << endl << endl;
  *
  *     // Example of how to use the result.
  *
  *     Transform3D<double> transform2 = estimateRigidTransformation3D.getTransformationMatrix();
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
  *     0.0000      -1.0000       0.0000       4.0000
  *     1.0000       0.0000       0.0000       2.0000
  *     0.0000       0.0000       1.0000       0.0000
  *     0.0000       0.0000       0.0000       1.0000
  *
  * Estimated transformation matrix:
  *
  *    -0.0000       -1.0000        0.0000        4.0000
  *     1.0000        0.0000       -0.0000        2.0000
  *     0.0000        0.0000        1.0000       -0.0000
  *     0.0000        0.0000        0.0000        1.0000
  *
  * Source point 1: (1.0000, 2.0000, 0.0000)
  * Target point 1: (2.0000, 3.0000, 0.0000)
  * Target point 1: (2.0000, 3.0000, 0.0000)
  *
  * \endcode
  *
  * \note If you plan to use this class with an STL container, please have a
  *       look at <a href=http://eigen.tuxfamily.org/dox/TopicStlContainers.html>
  *       this site. </a>
  *
  * \see EstimateRigidTransformation2D
  *
  * \author Christoph Haenisch
  * \version 0.3.3
  * \date 2012-10-12
  */

template <class T>
class EstimateRigidTransformation3D : public EstimateTransformation3D<T>
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

    EstimateRigidTransformation3D();

    EstimateRigidTransformation3D(const std::vector<Coordinate<T> > & source_points,
                                  const std::vector<Coordinate<T> > & target_points);

    EstimateRigidTransformation3D(const std::vector<Vector3T> & source_points,
                                  const std::vector<Vector3T> & target_points);

    EstimateRigidTransformation3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & source_points,
                                  const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & target_points);

    virtual ~EstimateRigidTransformation3D();

    virtual void compute();

    Matrix3T getRotationMatrix() const;
    Vector3T getTranslationVector() const;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new instance of this class.
  */

template <class T>
EstimateRigidTransformation3D<T>::EstimateRigidTransformation3D()
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
EstimateRigidTransformation3D<T>::EstimateRigidTransformation3D(const std::vector<Coordinate<T> > & source_points,
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
EstimateRigidTransformation3D<T>::EstimateRigidTransformation3D(const std::vector<Vector3T> & source_points,
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
EstimateRigidTransformation3D<T>::EstimateRigidTransformation3D(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & source_points,
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
EstimateRigidTransformation3D<T>::~EstimateRigidTransformation3D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Computes the transformation estimation.
  *
  * The transformation estimation is a least square scheme which decomposes a
  * correlation matrix between the two point sets using the singular value
  * decomposition. For a more detailed explanation of the algorithm, please have
  * a look at the source code. (A link should be below.)
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
void EstimateRigidTransformation3D<T>::compute()
{
    if (super::m_source_points.size() != super::m_target_points.size())
    {
        ErrorObj error;
        error.setClassName("EstimateRigidTransformation3D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    if (super::m_source_points.cols() < 2 || super::m_target_points.cols() < 2)
    {
        ErrorObj error;
        error.setClassName("EstimateRigidTransformation3D<T>");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough points to compute transformation matrix.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    /*

    Explanation of the algorithm (see also "Image Guided Therapy" lecture):

    We try to estimate the rotation around the centroids of both data sets. Thus we first
    subtract the mean of both point sets such that the sets are centered around the point
    of origin. Then we compute the correlation matrix between both point sets:

    R_XY = (X - mean(X)) * (Y - mean(Y))^T.

    By the way, R_XY is nothing else but the cross-covariance matrix of X and Y, cov(X,Y).
    Now, R_XY can be (equivalently) decomposed in two ways, first by means of a singular
    value decomposition (SVD)

    R_XY = U * Sigma * V^T,

    where U and V are orthogonal matrices and Sigma is a diagonal matrix, and second
    by using an Eigen decomposition

    R_XY = (X - mean(X)) * (Y - mean(Y))^T
         = x * y^T
         = x * [R * x]^T                (use y = R * x)
         = x * x^T * R^T
         = S * D * S^T * R^T            (Eigen decomposition of x * x^T)

    where x and y are the zero mean vectors of X and Y, respectively, D is a diagonal
    matrix, S is an orthogonal matrix and R is the sought rotation matrix. Comparison of

    R_XY = U * Sigma * V^T,
         = S * D     * S^T * R^T

    yields

    V^T = S^T * R^T = U^T * R^T.

    It follows that

    R = V * U^T.

    Now we can construct the homogeneous transformation matrix as follows:

    | R | T |     | I | mean(Y) |     | R | 0 |     | I | -mean(X) |
    | --+-- |  =  | --+-------- |  *  | --+-- |  *  | --+--------- |
    | 0 | 1 |     | 0 |   1     |     | 0 | 1 |     | 0 |    1     |

                  | R | mean(Y) - R * mean(X) |
               =  | --+---------------------- |
                  | 0 |          1            |

    Note: The above equation basically says: move the centroid of the source data to
          the point of origin, rotate the data and move it to the position of the
          centroid of the target data.

    Note: I is the identity matrix and T is an arbitrary translation vector.

    Note: Actually the rotation matrix on the left hand side should have had a name
          different from R, but since the entries are the same, we kept the name.
    */

    MatrixXT X = super::m_source_points;
    MatrixXT Y = super::m_target_points;

    // subtract mean

    Vector3T mean_X = X.rowwise().mean();
    Vector3T mean_Y = Y.rowwise().mean();

    X.colwise() -= mean_X;
    Y.colwise() -= mean_Y;

    // compute SVD of cross-covariance matrix

    MatrixXT R_XY = X * Y.adjoint();

    Eigen::JacobiSVD<MatrixXT> svd(R_XY, Eigen::ComputeThinU | Eigen::ComputeThinV);

    // compute estimate of the rotation matrix

    Matrix3T R = svd.matrixV() * svd.matrixU().adjoint();

    // assure a right-handed coordinate system

    if (R.determinant() < 0)
    {
        R = svd.matrixV() * Vector3T(1, 1, -1).asDiagonal() * svd.matrixU().transpose();
    }

    // construct homogeneous transformation matrix

    super::m_transformation_matrix.block(0, 0, 3, 3) = R;
    super::m_transformation_matrix.block(0, 3, 3, 1) = mean_Y - R * mean_X;
    super::m_transformation_matrix.block(3, 0, 1, 3) = RowVector3T::Zero();
    // super::m_transformation_matrix(3, 3) = 1.0; // already one from initialisation
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the rotation matrix as part of the sought transformation.
  *
  * \see compute(), getTranslationVector() and getTransformationMatrix()
  */

template <class T>
typename EstimateRigidTransformation3D<T>::Matrix3T EstimateRigidTransformation3D<T>::getRotationMatrix() const
{
    return super::m_transformation_matrix.block(0, 0, 3, 3);
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the translation vector as part of the sought transformation.
  *
  * \see compute(), getRotationMatrix() and getTransformationMatrix()
  */

template <class T>
typename EstimateRigidTransformation3D<T>::Vector3T EstimateRigidTransformation3D<T>::getTranslationVector() const
{
    return super::m_transformation_matrix.block(0, 3, 3, 1);
}


} // namespace TRTK


#endif // ESTIMATE_RIGID_TRANSFORMATION_3D_HPP_1230814474
