/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.1.1 (2011-08-05)
*/

/** \file EstimateTransformation2D.hpp
  * \brief This file contains the \ref TRTK::EstimateTransformation2D
  *        "base class" of all 2D transformation estimations.
  */


#ifndef ESTIMATE_TRANSFORMATION_2D_HPP_4376432766
#define ESTIMATE_TRANSFORMATION_2D_HPP_4376432766


#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "EstimateTransformation.hpp"
#include "Transform2D.hpp"


namespace TRTK
{


/** \class EstimateTransformation2D
  *
  * \brief Base class for 2D transformation estimations from two point sets.
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
class EstimateTransformation2D : public EstimateTransformation<T>
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

    EstimateTransformation2D();
    virtual ~EstimateTransformation2D();

    virtual void compute() = 0;

    virtual value_type getRMS() const;

    virtual const Matrix3T & getTransformationMatrix() const;

    void setSourcePoints(const std::vector<Coordinate<T> > &);
    void setSourcePoints(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > &);
    void setSourcePoints(const std::vector<Vector3T> &);

    void setTargetPoints(const std::vector<Coordinate<T> > &);
    void setTargetPoints(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > &);
    void setTargetPoints(const std::vector<Vector3T> &);

protected:
    MatrixXT m_source_points; // each column contains a coordinate vector
    MatrixXT m_target_points; // each column contains a coordinate vector

    Matrix3T m_transformation_matrix;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new initialized instance of this class.
  */

template <class T>
EstimateTransformation2D<T>::EstimateTransformation2D()
{
    m_transformation_matrix.setIdentity();
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Deletes the instance.
  */

template <class T>
EstimateTransformation2D<T>::~EstimateTransformation2D()
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
typename EstimateTransformation2D<T>::value_type EstimateTransformation2D<T>::getRMS() const
{
    using std::sqrt;

    if (m_source_points.size() != m_target_points.size())
    {
        ErrorObj error;
        error.setClassName("EstimateTransformation2D<T>");
        error.setFunctionName("getRMS");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    const int number_points =  m_source_points.cols();

    if (number_points == 0) return 0;

    Transform2D<T> transform = getTransformationMatrix();

    value_type RMS = 0;

    for (int i = 0; i < number_points; ++i)
    {
        const Vector2T & source = m_source_points.block(0, i, 2, 1);
        const Vector2T & target = m_target_points.block(0, i, 2, 1);

        RMS += (transform * source - target).squaredNorm();
    }

    RMS = sqrt(RMS / number_points);

    return RMS;
}


/** \tparam T scalar type of the coordinates
  *
  * \return Returns the sought transformation matrix in the form of a
  *         homogeneous 3x3 matrix. (This comprises the rotation as well as
  *         the translation, for instance.)
  *
  * \see compute()
  */

template <class T>
const typename EstimateTransformation2D<T>::Matrix3T & EstimateTransformation2D<T>::getTransformationMatrix() const
{
    return m_transformation_matrix;
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   The points can either be plain 2D or 3D
  *                             \ref HomogeneousCoordinates "homogeneous"
  *                             coordinates.
  *
  * \brief Sets the source points.
  *
  * \throw ErrorObj If there are any coordinates other than 2D or 3D coordinates,
  *                 an error object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  */

template <class T>
void EstimateTransformation2D<T>::setSourcePoints(const std::vector<Coordinate<T> > & source_points)
{
    const int m = 2;
    const int n = source_points.size();

    m_source_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        if (!(source_points[i].size() == 2 || source_points[i].size() == 3))
        {
            ErrorObj error;
            error.setClassName("EstimateTransformation2D<T>");
            error.setFunctionName("setSourcePoints");
            error.setErrorMessage("One or more source points are of wrong size.");
            error.setErrorCode(WRONG_POINT_SIZE);
            throw error;
        }

        m_source_points.col(i) = source_points[i].toArray().head(2);
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   2D coordinates.
  *
  * \brief Sets the source points.
  */

template <class T>
void EstimateTransformation2D<T>::setSourcePoints(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & source_points)
{
    const int m = 2;
    const int n = source_points.size();
    // we use the fact that std::vector is contiguous; use a C cast instead of an
    // C++ cast (reinterpret_cast + const_cast) to make the line easier to read
    m_source_points = Eigen::Map<MatrixXT>((T *)(&source_points[0]), m, n);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] source_points   3D \ref HomogeneousCoordinates "homogeneous" coordinates.
  *
  * \brief Sets the source points.
  */

template <class T>
void EstimateTransformation2D<T>::setSourcePoints(const std::vector<Vector3T> & source_points)
{
    const int m = 2;
    const int n = source_points.size();

    m_source_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        m_source_points.col(i) = source_points[i].head(2);
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] target_points   The points can either be plain 2D or 3D
  *                             \ref HomogeneousCoordinates "homogeneous"
  *                             coordinates.
  *
  * \brief Sets the target points.
  *
  * \throw ErrorObj If there are any coordinates other than 2D or 3D coordinates,
  *                 an error object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  */

template <class T>
void EstimateTransformation2D<T>::setTargetPoints(const std::vector<Coordinate<T> > & target_points)
{
    const int m = 2;
    const int n = target_points.size();

    m_target_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        if (!(target_points[i].size() == 2 || target_points[i].size() == 3))
        {
            ErrorObj error;
            error.setClassName("EstimateTransformation2D<T>");
            error.setFunctionName("setTargetPoints");
            error.setErrorMessage("One or more source points are of wrong size.");
            error.setErrorCode(WRONG_POINT_SIZE);
            throw error;
        }

        m_target_points.col(i) = target_points[i].toArray().head(2);
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] target_points   2D coordinates.
  *
  * \brief Sets the target points.
  */

template <class T>
void EstimateTransformation2D<T>::setTargetPoints(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & target_points)
{
    const int m = 2;
    const int n = target_points.size();
    // we use the fact that std::vector is contiguous; use a C cast instead of an
    // C++ cast (reinterpret_cast + const_cast) to make the line easier to read
    m_target_points = Eigen::Map<MatrixXT>((T *)(&target_points[0]), m, n);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] target_points   3D \ref HomogeneousCoordinates "homogeneous" coordinates.
  *
  * \brief Sets the target points.
  */

template <class T>
void EstimateTransformation2D<T>::setTargetPoints(const std::vector<Vector3T> & target_points)
{
    const int m = 2;
    const int n = target_points.size();

    m_target_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        m_target_points.col(i) = target_points[i].head(2);
    }
}


} // namespace TRTK


#endif // ESTIMATE_TRANSFORMATION_2D_HPP_4376432766
