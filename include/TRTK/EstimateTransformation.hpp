/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.1.1 (2011-08-05)
*/

/** \file EstimateTransformation.hpp
  * \brief This file contains the \ref TRTK::EstimateTransformation
  *        "base class" of all transformation estimations.
  */


#ifndef ESTIMATE_TRANSFORMATION_HPP_1072314347
#define ESTIMATE_TRANSFORMATION_HPP_1072314347


#include <Eigen/Core>
#include <Eigen/Dense>


namespace TRTK
{


/** \class EstimateTransformation
  *
  * \brief Base class for all transformation estimations from two point sets.
  *
  * \see EstimateTransformation2D, EstimateTransformation3D
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date 2011-08-05
  */

template <class T>
class EstimateTransformation
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
    typedef Eigen::Matrix<T, 1, 2> RowVector2T;                         //!< 2D row vector with value type T.
    typedef Eigen::Matrix<T, 1, 3> RowVector3T;                         //!< 3D row vector with value type T.
    typedef Eigen::Matrix<T, 2, 2> Matrix2T;                            //!< 2 x 2 matrix with value type T.
    typedef Eigen::Matrix<T, 3, 3> Matrix3T;                            //!< 3 x 3 matrix with value type T.
    typedef Eigen::Matrix<T, 4, 4> Matrix4T;                            //!< 4 x 4 matrix with value type T.

    enum Error {
        NOT_ENOUGH_POINTS,                                              //!< More points are required to estimate the transformation.
        UNEQUAL_NUMBER_OF_POINTS,                                       //!< The two point sets do not have the same cardinality.
        UNKNOWN_ERROR,                                                  //!< An unknown error occured.
        WRONG_POINT_SIZE                                                //!< One or more points have a wrong size.
    };

    virtual ~EstimateTransformation();                                  //!< Destructor.

    virtual void compute() = 0;                                         //!< Performs the transformation estimation.

    virtual value_type getRMS() const = 0;                              //!< Returns the root mean square error of the estimated transformation.

    // virtual const MatrixXT & getTransformationMatrix() const = 0;    //!< Returns the estimated transformation matrix.
};


template <class T>
EstimateTransformation<T>::~EstimateTransformation()
{
}


} // namespace TRTK


#endif // ESTIMATE_TRANSFORMATION_HPP_1072314347
