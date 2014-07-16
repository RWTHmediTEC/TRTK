/*
    Copyright (C) 2010 - 2014 Fabian Killus, Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.3.0 (2012-03-29)
*/

/** \file Fit3D.hpp
  * \brief This file contains the base class for all 3D fitting classes.
  */


#ifndef FIT_3D_HPP_3453465476
#define FIT_3D_HPP_3453465476


#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "Transform3D.hpp"
#include "Fit.hpp"

namespace TRTK
{


/** \class Fit3D
  *
  * \brief Base class for 3D fitting algorithms where fitting is done
  *        to a 3D point set.
  *
  * \note Some functions might throw an \ref ErrorObj "error object". See
  *       the appropriate function for more details.
  *
  * \see Fit
  *
  * \author Fabian Killus, Christoph Haenisch
  * \version 0.3.0
  * \date 2012-03-29
  */

template <class T>
class Fit3D : public Fit<T>
{
private:

    typedef Fit<T> super;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;

    typedef typename super::ArrayXT ArrayXT;
    typedef typename super::MatrixXT MatrixXT;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::Vector2T Vector2T;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::Matrix2T Matrix2T;
    typedef typename super::Matrix3T Matrix3T;

    using super::DATA_POINTS_TOO_SIMILAR;
    using super::INFINITY_NOT_AVAILABLE;
    using super::NAN_NOT_AVAILABLE;
    using super::NOT_ENOUGH_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_POINT_SIZE;

    Fit3D();
    virtual ~Fit3D();

    void compute() = 0;

    T getDistanceTo(const Coordinate<T> & point) const = 0;
    T getRMS() const = 0;
    unsigned getNumberPointsRequired() const = 0;

    void setPoints(const std::vector<Coordinate<T> > &);
    void setPoints(const std::vector<Vector3T> &);
    void setPoints(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > &);

protected:
    MatrixXT m_points;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new initialized instance of this class.
  */

template <class T>
Fit3D<T>::Fit3D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Deletes the instance.
  */

template <class T>
Fit3D<T>::~Fit3D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points   The points can either be plain 3D or 4D
  *                      \ref HomogeneousCoordinates "homogeneous"
  *                      coordinates.
  *
  * \brief Sets the point set.
  *
  * \throw ErrorObj If there are any coordinates other than 3D or 4D coordinates,
  *                 an error object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  */

template <class T>
void Fit3D<T>::setPoints(const std::vector<Coordinate<T> > & points)
{
    const int m = 3;
    const int n = points.size();

    m_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {

        if (!(points[i].size() == 3 || points[i].size() == 4))
        {
            ErrorObj error;
            error.setClassName("Fit3D<T>");
            error.setFunctionName("setPoints");
            error.setErrorMessage("One or more source points are of wrong size.");
            error.setErrorCode(WRONG_POINT_SIZE);
            throw error;
        }

        m_points.col(i) = points[i].toArray().head(3);
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points   3D coordinates.
  *
  * \brief Sets the point set.
  */

template <class T>
void Fit3D<T>::setPoints(const std::vector<Vector3T> & points)
{
    const int m = 3;
    const int n = points.size();

    // we use the fact that std::vector is contiguous; use a C cast instead of an
    // C++ cast (reinterpret_cast + const_cast) to make the line easier to read
    m_points = Eigen::Map<MatrixXT>((T *)(&points[0]), m, n);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points   4D \ref HomogeneousCoordinates "homogeneous" coordinates.
  *
  * \brief Sets the point set.
  */

template <class T>
void Fit3D<T>::setPoints(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & points)
{
    const int m = 3;
    const int n = points.size();

    m_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        m_points.col(i) = points[i].head(3);
    }
}


} // namespace TRTK


# endif // FIT_3D_HPP_3453465476
