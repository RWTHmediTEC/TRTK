/*
    Copyright (C) 2010 - 2014 Fabian Killus, Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.1.0 (2012-03-05)
*/

/** \file Fit.hpp
  * \brief This file contains the base class for all fitting classes.
  */


#ifndef FIT_HPP_2938475321
#define FIT_HPP_2938475321

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Coordinate.hpp"


namespace TRTK
{


/** \class Fit
  *
  * \tparam T floating point type
  *
  * \brief Base class for all fitting classes.
  *
  * Every fitting algorithm must provide the method \ref setPoints() with which
  * the data points can be passed. In general, a model is only computed by
  * calling the member function \ref compute(). Bear that in mind before calling
  * the getter functions. After having estimated the regression model, the
  * estimation error can be determined by \ref getRMS(). The minimum number of
  * points needed to be able to compute a model is given by
  * \ref getMinimumNumberOfPoints().
  *
  * \see Fit2D, Fit3D
  *
  * \author Christoph Haenisch, Fabian Killus
  * \version 0.2.0
  * \date 2012-03-29
  */

template <class T>
class Fit
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;                                               //!< Internally used value type (should be a floating point type).
    typedef Eigen::Array<T, Eigen::Dynamic, 1> ArrayXT;                 //!< General-purpose array of arbitrary size with value type T.
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXT;  //!< Matrix of arbitrary size with value type T.
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXT;               //!< Column vector of arbitrary size with value type T.
    typedef Eigen::Matrix<T, 2, 1> Vector2T;                            //!< 2D column vector with value type T.
    typedef Eigen::Matrix<T, 3, 1> Vector3T;                            //!< 3D column vector with value type T.
    typedef Eigen::Matrix<T, 4, 1> Vector4T;                            //!< 4D column vector with value type T.
    typedef Eigen::Matrix<T, 2, 2> Matrix2T;                            //!< 2 x 2 matrix with value type T.
    typedef Eigen::Matrix<T, 3, 3> Matrix3T;                            //!< 3 x 3 matrix with value type T.

    enum Error {
        DATA_POINTS_TOO_SIMILAR,                                        //!< The data points are to similar.
        INFINITY_NOT_AVAILABLE,                                         //!< The type T cannot represent infinity (see \c std::numeric_limits<T>).
        NAN_NOT_AVAILABLE,                                              //!< The type T cannot represent NaN (see \c std::numeric_limits<T>).
        NOT_ENOUGH_POINTS,                                              //!< More points are required to estimate the transformation.
        UNKNOWN_ERROR,                                                  //!< An unknown error occured.
        WRONG_POINT_SIZE                                                //!< One or more points have a wrong size.
    };

    virtual ~Fit();                                                     //!< Destructor.

    virtual void compute() = 0;                                         //!< Performs the regression analysis.

    virtual T getDistanceTo(const Coordinate<T> & point) const = 0;     //!< Returns the distance/deviation of the given data point to the estimated model.
    virtual T getRMS() const = 0;                                       //!< Returns the root mean square error of the model fitting.
    virtual unsigned getNumberPointsRequired() const = 0;               //!< Returns the minimum number of data points required to compute a model.

    virtual void setPoints(const std::vector<Coordinate<T> > &) = 0;    //!< Sets the data point set.
};


template <class T>
Fit<T>::~Fit()
{
}


} // namespace TRTK


#endif // FIT_HPP_2938475321