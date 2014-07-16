/*
    Copyright (C) 2010 - 2014 Fabian Killus, Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.0 (2012-03-29)
*/

/** \file Fit2D.hpp
  * \brief This file contains the base class for all 2D fitting classes.
  */


#ifndef FIT_2D_HPP_9582945877
#define FIT_2D_HPP_9582945877


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


/** \class Fit2D
  *
  * \brief Base class for 2D fitting algorithms where fitting is done
  *        to a 2D point set.
  *
  * \note Some functions might throw an \ref ErrorObj "error object". See
  *       the appropriate function for more details.
  *
  * \see Fit
  *
  * \author Fabian Killus, Christoph Haenisch
  * \version 0.2.0
  * \date 2012-03-29
  */

template <class T>
class Fit2D : public Fit<T>
{
private:

    typedef Fit<T> super;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;

    typedef typename super::MatrixXT MatrixXT;
    typedef typename super::Matrix2T Matrix2T;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::Vector2T Vector2T;
    typedef typename super::Vector3T Vector3T;

    using super::DATA_POINTS_TOO_SIMILAR;
    using super::INFINITY_NOT_AVAILABLE;
    using super::NAN_NOT_AVAILABLE;
    using super::NOT_ENOUGH_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_POINT_SIZE;

    Fit2D();
    virtual ~Fit2D();

    void compute() = 0;

    T getDistanceTo(const Coordinate<T> & point) const = 0;
    T getRMS() const = 0;
    unsigned getNumberPointsRequired() const = 0;

    void setPoints(const std::vector<Coordinate<T> > &);
    void setPoints(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > &);
    void setPoints(const std::vector<Vector3T> &);

protected:
    MatrixXT m_points;
};


/** \tparam T scalar type of the coordinates
  *
  * \brief Constructs a new initialized instance of this class.
  */

template <class T>
Fit2D<T>::Fit2D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \brief Deletes the instance.
  */

template <class T>
Fit2D<T>::~Fit2D()
{
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points   The points can either be plain 2D or 3D
  *                      \ref HomogeneousCoordinates "homogeneous"
  *                      coordinates.
  *
  * \brief Sets the point set.
  *
  * \throw ErrorObj If there are any coordinates other than 2D or 3D coordinates,
  *                 an error object is thrown and its error code is set to
  *                 \c WRONG_POINT_SIZE.
  */

template <class T>
void Fit2D<T>::setPoints(const std::vector<Coordinate<T> > & points)
{
    const int m = 2;
    const int n = points.size();

    m_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {

        if (!(points[i].size() == 2 || points[i].size() == 3))
        {
            ErrorObj error;
            error.setClassName("Fit3D<T>");
            error.setFunctionName("setPoints");
            error.setErrorMessage("One or more source points are of wrong size.");
            error.setErrorCode(WRONG_POINT_SIZE);
            throw error;
        }

        m_points.col(i) = points[i].toArray().head(2);
    }
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points   2D coordinates.
  *
  * \brief Sets the point set.
  */

template <class T>
void Fit2D<T>::setPoints(const std::vector<Vector2T, Eigen::aligned_allocator<Vector2T> > & points)
{
    const int m = 2;
    const int n = points.size();

    // we use the fact that std::vector is contiguous; use a C cast instead of an
    // C++ cast (reinterpret_cast + const_cast) to make the line easier to read
    m_points = Eigen::Map<MatrixXT>((T *)(&points[0]), m, n);
}


/** \tparam T scalar type of the coordinates
  *
  * \param [in] points   3D \ref HomogeneousCoordinates "homogeneous" coordinates.
  *
  * \brief Sets the point set.
  */

template <class T>
void Fit2D<T>::setPoints(const std::vector<Vector3T> & points)
{
    const int m = 2;
    const int n = points.size();

    m_points.resize(m, n);

    for (int i = 0; i < n; ++i)
    {
        m_points.col(i) = points[i].head(2);
    }
}


} // namespace TRTK


# endif // FIT_2D_HPP_9582945877
