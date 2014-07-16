/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.1.1 (2011-08-05)
*/

/** \file EstimateTransformation3D.hpp
  * \brief This file contains the \ref TRTK::EstimateTransformation3D
  *        "base class" of all 3D transformation estimations.
  */


#ifndef ESTIMATE_TRANSFORMATION_3D_HPP_5431312489
#define ESTIMATE_TRANSFORMATION_3D_HPP_5431312489


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "EstimateTransformation.hpp"
#include "Transform3D.hpp"


namespace TRTK
{


/** \class EstimateTransformation3D
  *
  * \brief Base class for 3D transformation estimations from two point sets.
  *
  * \note Some functions might throw an \ref ErrorObj "error object". See the
  *       appropriate function for more details.
  *
  * \see EstimateTransformation
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date 2011-08-05
  */

template <class T>
class EstimateTransformation3D : public EstimateTransformation<T>
{
private:

    typedef EstimateTransformation<T> super;

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

    EstimateTransformation3D();
    virtual ~EstimateTransformation3D();

    virtual void compute() = 0;

    virtual value_type getRMS() const;

    virtual const Matrix4T & getTransformationMatrix() const;

    void setSourcePoints(const std::vector<Coordinate<T> > &);
    void setSourcePoints(const std::vector<Vector3T> &);
    void setSourcePoints(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > &);

    void setTargetPoints(const std::vector<Coordinate<T> > &);
    void setTargetPoints(const std::vector<Vector3T> &);
    void setTargetPoints(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > &);

protected:
    MatrixXT m_source_points; // each column contains a coordinate vector
    MatrixXT m_target_points; // each column contains a coordinate vector

    Matrix4T m_transformation_matrix;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new initialized instance of this class.
  */

template <class T>
EstimateTransformation3D<T>::EstimateTransformation3D()
{
    m_transformation_matrix.setIdentity();
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Deletes the instance.
  */

template <class T>
EstimateTransformation3D<T>::~EstimateTransformation3D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Returns the root mean square (RMS) error of the estimated
  *        transformation.
  *
  * It is assumed, that the computation was done before.
  *
  * The value type \p T must provide a function <tt> T sqrt(T value) </tt> which
  * yields the square root of \p value.
  *
  * \throw ErrorObj If the point sets do not have the same cardinality, an error
  *                 object is thrown and its error code is set to
  *                 \c UNEQUAL_NUMBER_OF_POINTS.
  *
  * \see compute()
  */

template <class T>
typename EstimateTransformation3D<T>::value_type EstimateTransformation3D<T>::getRMS() const
{
    using std::sqrt;

    if (m_source_points.size() != m_target_points.size())
    {
        ErrorObj error;
        error.setClassName("EstimateTransformation3D<T>");
        error.setFunctionName("getRMS");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    const int number_points =  m_source_points.cols();

    if (number_points == 0) return 0;

    Transform3D<T> transform = getTransformationMatrix();

    value_type RMS = 0;

    for (int i = 0; i < number_points; ++i)
    {
        const Vector3T & source = m_source_points.block(0, i, 3, 1);
        const Vector3T & target = m_target_points.block(0, i, 3, 1);

        RMS += (transform * source - target).squaredNorm();
    }

    RMS = sqrt(RMS / number_points);

    return RMS;
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the sought transformation matrix in the form of a
  *         homogeneous 4x4 matrix. (This comprises the rotation as well as
  *         the translation, for instances.)
  *
  * \see compute()
  */

template <class T>
const typename EstimateTransformation3D<T>::Matrix4T & EstimateTransformation3D<T>::getTransformationMatrix() const
{
    return m_transformation_matrix;
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   The points can either be plain 3D or 4D
  *                             \ref HomogeneousCoordinates "homogeneous"
  *                             coordinates.
  *
  * \brief Sets the source points.
  *
  * \throw ErrorObj If there are any coordinates other than 3D or 4D coordinates,
  *                 an error object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  */

template <class T>
void EstimateTransformation3D<T>::setSourcePoints(const std::vector<Coordinate<T> > & source_points)
{
    const int m = 3;
    const int n = source_points.size();

    m_source_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        if (!(source_points[i].size() == 3 || source_points[i].size() == 4))
        {
            ErrorObj error;
            error.setClassName("EstimateTransformation3D<T>");
            error.setFunctionName("setSourcePoints");
            error.setErrorMessage("One or more source points are of wrong size.");
            error.setErrorCode(WRONG_POINT_SIZE);
            throw error;
        }

        m_source_points.col(i) = source_points[i].toArray().head(3);
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   3D coordinates.
  *
  * \brief Sets the source points.
  */

template <class T>
void EstimateTransformation3D<T>::setSourcePoints(const std::vector<Vector3T> & source_points)
{
    const int m = 3;
    const int n = source_points.size();
    // we use the fact that std::vector is contiguous; use a C cast instead of an
    // C++ cast (reinterpret_cast + const_cast) to make the line easier to read
    m_source_points = Eigen::Map<MatrixXT>((T *)(&source_points[0]), m, n);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   4D \ref HomogeneousCoordinates "homogeneous" coordinates.
  *
  * \brief Sets the source points.
  */

template <class T>
void EstimateTransformation3D<T>::setSourcePoints(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & source_points)
{
    const int m = 3;
    const int n = source_points.size();

    m_source_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        m_source_points.col(i) = source_points[i].head(3);
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] target_points   The points can either be plain 3D or 4D
  *                             \ref HomogeneousCoordinates "homogeneous"
  *                             coordinates.
  *
  * \brief Sets the target points.
  *
  * \throw ErrorObj If there are any coordinates other than 3D or 4D coordinates,
  *                 an error object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  */

template <class T>
void EstimateTransformation3D<T>::setTargetPoints(const std::vector<Coordinate<T> > & target_points)
{
    const int m = 3;
    const int n = target_points.size();

    m_target_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        if (!(target_points[i].size() == 3 || target_points[i].size() == 4))
        {
            ErrorObj error;
            error.setClassName("EstimateTransformation3D<T>");
            error.setFunctionName("setTargetPoints");
            error.setErrorMessage("One or more source points are of wrong size.");
            error.setErrorCode(WRONG_POINT_SIZE);
            throw error;
        }

        m_target_points.col(i) = target_points[i].toArray().head(3);
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] target_points   3D coordinates.
  *
  * \brief Sets the target points.
  */

template <class T>
void EstimateTransformation3D<T>::setTargetPoints(const std::vector<Vector3T> & target_points)
{
    const int m = 3;
    const int n = target_points.size();
    // we use the fact that std::vector is contiguous; use a C cast instead of an
    // C++ cast (reinterpret_cast + const_cast) to make the line easier to read
    m_target_points = Eigen::Map<MatrixXT>((T *)(&target_points[0]), m, n);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] target_points   4D \ref HomogeneousCoordinates "homogeneous" coordinates.
  *
  * \brief Sets the target points.
  */

template <class T>
void EstimateTransformation3D<T>::setTargetPoints(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & target_points)
{
    const int m = 3;
    const int n = target_points.size();

    m_target_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        m_target_points.col(i) = target_points[i].head(3);
    }
}


} // namespace TRTK


# endif // ESTIMATE_TRANSFORMATION_3D_HPP_5431312489
