/*
    Computes a 3D transform in homogeneous coordinates.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.5.0 (2013-04-04)
*/

/** \file Transform3D.hpp
  * \brief This file contains the \ref TRTK::Transform3D "Transform3D" class.
  */

/* Note: The concatenation of several affine matrices to one single
 *       matrix is possible without any problems. However, if a
 *       non-affine transform (e.g. a projective transform) is used,
 *       it may only be used once and at the very end (i.e. multiplied
 *       from left) due to the subsequent non-linear normalisation step.
 */


#ifndef TRANSFORM3D_HPP_0386836041
#define TRANSFORM3D_HPP_0386836041


#include <cassert>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "Tools.hpp"


namespace TRTK
{


/** \class Transform3D
  *
  * \brief Affine or projective transformation in 3D.
  *
  * This class provides means to transform 3D coordinates via an affine or a
  * projective transformation. The transformation can be composed from a
  * sequence of elementary transformations such as translation, scaling,
  * rotation etc. or by directly setting the entries of a \f$ 4 \times 4 \f$
  * transformation matrix. The coordinates may be given as 3D or as 4D
  * coordinates in case of \ref HomogeneousCoordinates "homogeneous coordinates".
  *
  * The transformation is computed by multiplying a \f$ 4 \times 4 \f$
  * matrix with a 4D homogeneous coordinate followed by a potential normalization
  * step. That is, if \f$ x \f$ is transformed to \f$ y \f$, first an intermediate
  * vector \f$ w \f$ is computed
  *
  * \f[
  * \begin{pmatrix}
  *     w_1  \\  w_2  \\  w_3  \\  w_4
  * \end{pmatrix}
  * =
  * \begin{pmatrix}
  *     a_{11}  &  a_{12}  &  a_{13}  &  a_{14}  \\
  *     a_{21}  &  a_{22}  &  a_{23}  &  a_{24}  \\
  *     a_{31}  &  a_{32}  &  a_{33}  &  a_{34}  \\
  *     a_{41}  &  a_{42}  &  a_{43}  &  a_{44}
  * \end{pmatrix}
  * \begin{pmatrix}
  *     x_1  \\  x_2  \\  x_3  \\  1
  * \end{pmatrix}
  * \f]
  *
  * followed by a normalization step
  *
  * \f[
  * \begin{pmatrix}
  *     y_1  \\  y_2  \\  y_3  \\  1
  * \end{pmatrix}
  * = \frac{1}{w_4}
  * \begin{pmatrix}
  *     w_1  \\  w_2  \\  w_3  \\  w_4
  * \end{pmatrix}
  * \f]
  *
  * In case of affine transformations
  *
  * \f[
  * y = Ax + b
  * \f]
  *
  * or
  *
  * \f[
  * \begin{pmatrix}
  *     y_1  \\  y_2  \\  y_3  \\  1
  * \end{pmatrix}
  * =
  * \begin{pmatrix}
  *     a_{11}  &  a_{12}  &  a_{13}  &  b_1  \\
  *     a_{21}  &  a_{22}  &  a_{23}  &  b_2  \\
  *     a_{31}  &  a_{32}  &  a_{33}  &  b_3  \\
  *     0       &  0       &  0       &  1
  * \end{pmatrix}
  * \begin{pmatrix}
  *     x_1  \\  x_2  \\  x_3  \\  1
  * \end{pmatrix}
  * \f]
  *
  * the subsequent normalization step can be omitted since the last entry of the
  * above intermediate vector \f$ w \f$ would be \f$ 1 \f$ in any event.
  *
  * Most functions check for certain assertions (e.g. if the coordinate size is
  * valid) and trigger an assertion failure if the assumption does not hold.
  * This is meant for debugging purposes only and can be disabled by defining
  * the macro \macro{NDEBUG}.
  *
  * Here are some examples of how to use the Transform3D class:
  *
  * Example 1:
  *
  * \code
  *
  * TRTK::Coordinate<double> coordinate(1, 4, 3);
  * TRTK::Transform3D<double> transform;
  *
  * const double pi = transform::pi;
  * transform.rotateZ(pi/2).translate(2, 0, 0);
  *
  * cout << transform * coordinate << endl;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * (-2, 1, 3)
  *
  * \endcode
  *
  * Example 2:
  *
  * \code
  *
  * using namespace TRTK;
  *
  * Coordinate<double> coordinate(1, 4, 3);
  *
  * Transform3D<double> transform1;
  * Transform3D<double> transform2;
  * Transform3D<double> transform3;
  * Transform3D<double> transform4;
  *
  * transform1.translate(1, 1, 0);
  *
  * // rotation of 90 degrees
  * transform2.a11() =  0;
  * transform2.a12() =  1;
  * transform2.a21() = -1;
  * transform2.a22() =  0;
  *
  * transform3.translate(-1, 0, 0);
  *
  * transform4 = transform3 * transform2 * transform1;
  *
  * cout << transform4 * coordinate << endl;
  *
  * transform4.reset() = transform1 >> transform2 >> transform3;
  *
  * cout << transform4 * coordinate << endl;
  *
  * cout << endl << transform4.getTransformationMatrix() << endl;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * (4, -2, 3)
  * (4, -2, 3)
  *
  *  0  1  0  0
  * -1  0  0 -1
  *  0  0  1  0
  *  0  0  0  1
  *
  * \endcode
  *
  * \note If you plan to use this class with an STL container, please have a
  *       look at <a href=http://eigen.tuxfamily.org/dox/TopicStlContainers.html>
  *       this site. </a>
  *
  * \see Transform2D and Coordinate
  *
  * \author Christoph Haenisch
  * \version 0.5.0
  * \date last changed on 2013-04-04
  */

template <class T>
class Transform3D
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Axis  {X_AXIS, Y_AXIS, Z_AXIS};
    enum Plane {XY_PLANE, XZ_PLANE, YZ_PLANE};
    enum Unit  {DEGREES, RADIANS};

    enum Error {DIVISION_BY_ZERO,
                INVALID_ARGUMENT,
                INVALID_AXIS,
                INVALID_UNIT,
                UNKNOWN_ERROR,
                WRONG_COORDINATE_SIZE};

    typedef T value_type;
    typedef Eigen::Matrix<T, 4, 4> Matrix4T;
    typedef Eigen::Matrix<T, 4, 1> Vector4T;
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Coordinate<T> coordinate_type;

    Transform3D();

    Transform3D(const Matrix4T &);

    Transform3D(T a11, T a12, T a13, T a14,
                T a21, T a22, T a23, T a24,
                T a31, T a32, T a33, T a34,
                T a41, T a42, T a43, T a44);

    Transform3D(const Transform3D<T> & transform3D);

    template <class U>
    Transform3D(const Transform3D<U> & transform3D);

    virtual ~Transform3D();

    T & a11();
    T & a12();
    T & a13();
    T & a14();
    T & a21();
    T & a22();
    T & a23();
    T & a24();
    T & a31();
    T & a32();
    T & a33();
    T & a34();
    T & a41();
    T & a42();
    T & a43();
    T & a44();

    const T & a11() const;
    const T & a12() const;
    const T & a13() const;
    const T & a14() const;
    const T & a21() const;
    const T & a22() const;
    const T & a23() const;
    const T & a24() const;
    const T & a31() const;
    const T & a32() const;
    const T & a33() const;
    const T & a34() const;
    const T & a41() const;
    const T & a42() const;
    const T & a43() const;
    const T & a44() const;

    const Coordinate<T> operator* (const Coordinate<T> &) const;
    const Matrix4T operator* (const Matrix4T &) const;
    const Vector3T operator* (const Vector3T &) const;
    const Vector4T operator* (const Vector4T &) const;
    const Transform3D operator* (const Transform3D &) const;
    const Transform3D operator>> (const Transform3D &) const;
    Transform3D& operator= (const Transform3D &);

    Matrix4T & getTransformationMatrix();
    const Matrix4T & getTransformationMatrix() const;

    const Transform3D inverse() const;

    Transform3D orthographicProjection(const Plane = XY_PLANE) const;
    Transform3D perspectiveProjection(const T distance, const Plane = XY_PLANE) const;

    Transform3D & reset();

    Transform3D & rotate(const double angle, const Axis = Z_AXIS, const Unit unit = RADIANS);
    Transform3D & rotateX(const double angle, const Unit unit = RADIANS);
    Transform3D & rotateY(const double angle, const Unit unit = RADIANS);
    Transform3D & rotateZ(const double angle, const Unit unit = RADIANS);
    Transform3D & rotateAxis(const double angle, const Coordinate<T> & axis, const Unit unit = RADIANS);
    Transform3D & rotateAxis(const double angle, const Vector3T & axis, const Unit unit = RADIANS);
    Transform3D & rotateAxis(const double angle, const Vector4T & axis, const Unit unit = RADIANS);

    Transform3D & scale(const T sx, const T sy, const T sz);

    // Transform3D & shear(const T sx, const T sy, const T sz);

    Transform3D & translate(const T dx, const T dy, const T dz);
    Transform3D & translate(const Coordinate<T> & position);

    bool is_affine() const;

    static const double pi;

private:

    void normalize(Vector4T &) const;

    Matrix4T * m_matrix;
    bool m_is_affine;
};


template <class T>
const double Transform3D<T>::pi = 3.14159265358979323846264338327950288419716939937510;


/** \tparam T scalar type
  *
  * \brief Constructs an instance of Transform3D.
  *
  * The internal matrix is set to the identity matrix.
  *
  * \see reset()
  */

template <class T>
Transform3D<T>::Transform3D() : m_matrix(new Matrix4T), m_is_affine(true)
{
    m_matrix->setIdentity();
}


/** \tparam T scalar type
  *
  * \param [in] matrix  Tranformation matrix.
  *
  * \brief Constructs an instance of Transform3D.
  *
  * Sets the internal matrix to the given \p matrix.
  *
  * \see reset()
  */

template <class T>
Transform3D<T>::Transform3D(const Matrix4T & matrix)
    : m_matrix(new Matrix4T())
{
    using Tools::isEqual;

    *m_matrix = matrix;

    if (isEqual<T>(matrix(3,0), 0) &&
        isEqual<T>(matrix(3,1), 0) &&
        isEqual<T>(matrix(3,2), 0) &&
        isEqual<T>(matrix(3,3), 1))
    {
        m_is_affine = true;
    }
    else
    {
        m_is_affine = false;
    }
}


/** \tparam T scalar type
  *
  * \brief Constructs an instance of Transform3D.
  *
  * Sets the internal matrix to the given coefficients.
  *
  * \see reset()
  */

template <class T>
Transform3D<T>::Transform3D(T a11, T a12, T a13, T a14,
                            T a21, T a22, T a23, T a24,
                            T a31, T a32, T a33, T a34,
                            T a41, T a42, T a43, T a44)
    : m_matrix(new Matrix4T())
{
    using Tools::isEqual;

    *m_matrix << a11, a12, a13, a14,
                 a21, a22, a23, a24,
                 a31, a32, a33, a34,
                 a41, a42, a43, a44;

    if (isEqual<T>(a41, 0) &&
        isEqual<T>(a42, 0) &&
        isEqual<T>(a43, 0) &&
        isEqual<T>(a44, 1))
    {
        m_is_affine = true;
    }
    else
    {
        m_is_affine = false;
    }
}


/** \tparam T scalar type of the newly created instance
  * \tparam U scalar type of the copied instance
  *
  * \brief Copy constructor.
  *
  * \see reset()
  */

template <class T>
template <class U>
Transform3D<T>::Transform3D(const Transform3D<U> & transform3D)
    : m_matrix(new Matrix4T())
{
    const int rows = transform3D.getTransformationMatrix().rows();
    const int cols = transform3D.getTransformationMatrix().cols();

    m_matrix->resize(rows, cols);

    for (int m = 0; m < rows; ++m)
    {
        for (int n = 0; n < cols; ++n)
        {
            (*m_matrix)(m, n) = T(transform3D.getTransformationMatrix()(m, n));
        }
    }

    m_is_affine = transform3D.is_affine();
}


// Even though a templated copy constructor exist, we need to define
// this one because otherwise the compiler would generate a default
// copy constructor!?

/** \tparam T scalar type
  *
  * \brief Copy constructor.
  *
  * \see reset()
  */

template <class T>
Transform3D<T>::Transform3D(const Transform3D<T> & transform3D)
    : m_matrix( new Matrix4T() )
{
    const int rows = transform3D.getTransformationMatrix().rows();
    const int cols = transform3D.getTransformationMatrix().cols();

    m_matrix->resize(rows, cols);

    for (int m = 0; m < rows; ++m)
    {
        for (int n = 0; n < cols; ++n)
        {
            (*m_matrix)(m, n) = T(transform3D.getTransformationMatrix()(m, n));
        }
    }

    m_is_affine = transform3D.is_affine();
}


/** \tparam T scalar type
  *
  * \brief Copy assignment.
  *
  * \see reset()
  */

template <class T>
Transform3D<T> & Transform3D<T>::operator= (const Transform3D & transform3D)
{
    *m_matrix = *transform3D.m_matrix;
    m_is_affine = transform3D.m_is_affine;
    return *this;
}


/** \tparam T scalar type
  *
  * \brief Destroys the instance of Transform3D.
  */

template <class T>
Transform3D<T>::~Transform3D()
{
    delete m_matrix;
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{11} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a11()
{
    return (*m_matrix)(0, 0);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{12} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a12()
{
    return (*m_matrix)(0, 1);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{13} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a13()
{
    return (*m_matrix)(0, 2);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{14} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a14()
{
    return (*m_matrix)(0, 3);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{21} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a21()
{
    return (*m_matrix)(1, 0);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{22} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a22()
{
    return (*m_matrix)(1, 1);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{23} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a23()
{
    return (*m_matrix)(1, 2);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{24} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a24()
{
    return (*m_matrix)(1, 3);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{31} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a31()
{
    return (*m_matrix)(2, 0);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{32} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a32()
{
    return (*m_matrix)(2, 1);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{33} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a33()
{
    return (*m_matrix)(2, 2);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{34} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a34()
{
    return (*m_matrix)(2, 3);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{41} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a41()
{
    m_is_affine = false;
    return (*m_matrix)(3, 0);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{42} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a42()
{
    m_is_affine = false;
    return (*m_matrix)(3, 1);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{43} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a43()
{
    m_is_affine = false;
    return (*m_matrix)(3, 2);
}


/** \tparam T scalar type
  *
  * \brief Element access (readable and writable).
  *
  * \return Reference to internal matrix element \f$ a_{44} \f$.
  *
  * \see reset()
  */

template <class T>
T & Transform3D<T>::a44()
{
    m_is_affine = false;
    return (*m_matrix)(3, 3);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{11} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a11() const
{
    return (*m_matrix)(0, 0);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{12} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a12() const
{
    return (*m_matrix)(0, 1);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{13} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a13() const
{
    return (*m_matrix)(0, 2);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{14} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a14() const
{
    return (*m_matrix)(0, 3);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{21} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a21() const
{
    return (*m_matrix)(1, 0);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{22} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a22() const
{
    return (*m_matrix)(1, 1);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{23} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a23() const
{
    return (*m_matrix)(1, 2);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{24} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a24() const
{
    return (*m_matrix)(1, 3);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{31} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a31() const
{
    return (*m_matrix)(2, 0);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{32} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a32() const
{
    return (*m_matrix)(2, 1);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{33}\f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a33() const
{
    return (*m_matrix)(2, 2);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{34} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a34() const
{
    return (*m_matrix)(2, 3);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{41} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a41() const
{
    return (*m_matrix)(3, 0);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{42} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a42() const
{
    return (*m_matrix)(3, 1);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{43} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a43() const
{
    return (*m_matrix)(3, 2);
}


/** \tparam T scalar type
  *
  * \brief Read-only element access.
  *
  * \return Internal matrix element \f$ a_{44} \f$.
  *
  * \see reset()
  */

template <class T>
const T & Transform3D<T>::a44() const
{
    return (*m_matrix)(3, 3);
}


/** \tparam T scalar type
  *
  * \param [in] coordinate Coordinate to be transformed. \p coordinate may be
  *             \ref HomogeneousCoordinates "homogeneous".
  *
  * \return Transformed coordinate of the same size as the input coordinate.
  *
  * \brief Transforms a coordinate with the internally saved transformation.
  *
  * The given \p coordinate must be of size three or four. If \p coordinate has
  * a size of four (homogeneous coordinate), it is assumed that its fourth
  * component is equal to one.
  *
  * Example:
  *
  * \code
  * TRTK::Coordinate<double> coordinate(1, 4, 3);
  * TRTK::Transform3D<double> transform;
  *
  * transform.rotateZ(90).translate(2, 0, 0);
  *
  * cout << transform * coordinate << endl;
  * \endcode
  *
  * Output:
  *
  * \code
  * (-2, 1, 3)
  * \endcode
  */

template <class T>
const Coordinate<T> Transform3D<T>::operator* (const Coordinate<T> & coordinate) const
{
    // 'coordinate' must be of size three or four. If 'coordinate' has a size
    // of four, it is assumed that the fourth component is equal to one.

    const int size = coordinate.size();

    if (!(size == 3 || size == 4))
    {
        ErrorObj error;
        error.setClassName("Transform3D<T>");
        error.setFunctionName("operator*");
        error.setErrorMessage("'coordinate' must be of size three or four.");
        error.setErrorCode(WRONG_COORDINATE_SIZE);
        throw error;
    }

    assert(coordinate.size() == 3 || (coordinate.size() == 4 && coordinate.w() == 1));

    Vector4T vector(coordinate.x(), coordinate.y(), coordinate.z(), 1);
    Vector4T result = *m_matrix * vector;

    if (!is_affine())
    {
        normalize(result);
    }

    if (size == 3)
    {
        return Coordinate<T>(result(0), result(1), result(2));
    }
    else // size == 4
    {
        return Coordinate<T>(result(0), result(1), result(2), 1);
    }
}


/** \tparam T scalar type
  *
  * \param [in] vector 3D vector to be transformed.
  *
  * \return Transformed 3D vector.
  *
  * \brief Transforms a 3D vector with the internally saved transformation.
  *
  * Example:
  *
  * \code
  * TRTK::Transform3D<double> transform;
  * TRTK::Transform3D<double>::Vector3T vec(1, 4, 3);
  *
  * transform.rotateZ(90).translate(2, 0, 0);
  *
  * std::cout << transform * vec;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  * -2
  * 1
  * 3
  * \endcode
  */

template <class T>
const typename Transform3D<T>::Vector3T Transform3D<T>::operator* (const Vector3T & vector) const
{
    Vector4T vector4D(vector(0), vector(1), vector(2), 1);
    Vector4T result = *m_matrix * vector4D;

    if (!is_affine())
    {
        normalize(result);
    }

    return result.head(3);
}


/** \tparam T scalar type
  *
  * \param [in] vector 4D vector to be transformed.
  *
  * \return Transformed 4D vector.
  *
  * \brief Transforms a vector with the internally saved transformation.
  *
  * The vector is assumed to be a normalized \ref HomogeneousCoordinates
  * "homogeneous coordinate", i.e. its last component is assumed to be
  * equal to one.
  *
  * Example:
  *
  * \code
  * TRTK::Transform3D<double> transform;
  * TRTK::Transform3D<double>::Vector4T vec(1, 4, 3, 1);
  *
  * transform.rotateZ(90).translate(2, 0, 0);
  *
  * std::cout << transform * vec;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  * -2
  * 1
  * 3
  * 1
  * \endcode
  */

template <class T>
const typename Transform3D<T>::Vector4T Transform3D<T>::operator* (const Vector4T & vector) const
{
    assert(Tools::isEqual<T>(vector(3), 1));

    Vector4T result = *m_matrix * vector;

    if (!is_affine())
    {
        normalize(result);
    }

    return result;
}


/** \tparam T scalar type
  *
  * \param [in] matrix 4x4 transformation matrix.
  *
  * \return 4x4 transformation matrix.
  *
  * \brief Composition of two transformations.
  *
  * This function computes the composition of the currently internally saved
  * transformation with the transformation given by \p matrix. Here, the
  * composition is a simple matrix multiplication.
  *
  * Example:
  *
  * \code
  * typedef TRTK::Transform3D<double> Transform;
  *
  * Transform T1(1, 0, 0, 0,
  *              0, 1, 0, 1,
  *              0, 0, 2, 2,
  *              0, 0, 0, 1);
  *
  * Transform::Matrix4T A;
  *
  * A << 1, 0, 0, 0,
  *      0, 2, 0, 3,
  *      0, 0, 2, 2,
  *      0, 0, 0, 1;
  *
  * Transform T2 = T1 * A;
  *
  * std::cout << T2.getTransformationMatrix();
  * \endcode
  *
  * Output:
  *
  * \code
  * 1 0 0 0
  * 0 2 0 4
  * 0 0 4 6
  * 0 0 0 1
  * \endcode
  */

template <class T>
const typename Transform3D<T>::Matrix4T Transform3D<T>::operator* (const Matrix4T & matrix) const
{
    typename Transform3D<T>::Matrix4T result = *m_matrix * matrix;

    return result;
}


/** \tparam T scalar type
  *
  * \param [in] transform A transformation described by Transform3D.
  *
  * \return A Transform3D object describing the composition of the two
  *         transformations.
  *
  * \brief Composition of two transformations.
  *
  * This function computes the composition of the currently internally saved
  * transformation with the transformation given by \p transform. Here, the
  * composition is a simple matrix multiplication.
  *
  * Example:
  *
  * \code
  * typedef TRTK::Transform3D<double> Transform;
  *
  * Transform T1(1, 0, 0, 0,
  *              0, 1, 0, 1,
  *              0, 0, 2, 2,
  *              0, 0, 0, 1);
  *
  * Transform T2(1, 0, 0, 0,
  *              0, 2, 0, 3,
  *              0, 0, 2, 2,
  *              0, 0, 0, 1);
  *
  * Transform T3 = T1 * T2;
  *
  * std::cout << T3.getTransformationMatrix();
  * \endcode
  *
  * Output:
  *
  * \code
  * 1 0 0 0
  * 0 2 0 4
  * 0 0 4 6
  * 0 0 0 1
  * \endcode
  *
  * \see operator>>()
  */

template <class T>
const Transform3D<T> Transform3D<T>::operator* (const Transform3D & transform) const
{
    Transform3D<T> result = (*m_matrix * *transform.m_matrix).eval();
    result.m_is_affine = m_is_affine && transform.m_is_affine;

    return result;
}


/** \tparam T scalar type
  *
  * \param [in] transform A transformation described by Transform3D.
  *
  * \return A Transform3D object describing the composition of the two
  *         transformations.
  *
  * \brief Composition of two transformations.
  *
  * This function computes the composition of the currently internally saved
  * transformation with the transformation given by \p transform. Here, the
  * composition is a simple matrix multiplication.
  *
  * Example:
  *
  * \code
  * typedef TRTK::Transform3D<double> Transform;
  *
  * Transform T1(1, 0, 0, 0,
  *              0, 1, 0, 1,
  *              0, 0, 2, 2,
  *              0, 0, 0, 1);
  *
  * Transform T2(1, 0, 0, 0,
  *              0, 2, 0, 3,
  *              0, 0, 2, 2,
  *              0, 0, 0, 1);
  *
  * Transform T3 = T1 >> T2; // == T2 * T1
  *
  * std::cout << T3.getTransformationMatrix();
  * \endcode
  *
  * Output:
  *
  * \code
  * 1 0 0 0
  * 0 2 0 5
  * 0 0 4 6
  * 0 0 0 1
  * \endcode
  *
  * \see operator*()
  */

template <class T>
const Transform3D<T> Transform3D<T>::operator>> (const Transform3D & transform) const
{
    Transform3D<T> result = (*transform.m_matrix * *m_matrix).eval();
    result.m_is_affine = m_is_affine && transform.m_is_affine;

    return result;
}


/** \tparam T scalar type
  *
  * \return Returns the internal transformation as 4x4 matrix representation.
  *
  * \brief Returns the internal transformation matrix.
  */

template <class T>
typename Transform3D<T>::Matrix4T & Transform3D<T>::getTransformationMatrix()
{
    return *m_matrix;
}


/** \tparam T scalar type
  *
  * \return Returns the internal transformation as 4x4 matrix representation.
  *
  * \brief Returns the internal transformation matrix.
  */

template <class T>
const typename Transform3D<T>::Matrix4T & Transform3D<T>::getTransformationMatrix() const
{
    return *m_matrix;
}


/** \tparam T scalar type
  *
  * \brief Returns the inverse transformation.
  *
  * \return A new copy.
  */

template <class T>
const Transform3D<T> Transform3D<T>::inverse() const
{
    return Transform3D(m_matrix->inverse());
}


/** \tparam T scalar type
  *
  * \param [in] plane   \c XY_PLANE, \c XZ_PLANE, or \c YZ_PLANE
  *
  * \brief Constructs an orthographic projection matrix.
  *
  * Projects points to the x-y, x-z, or y-z plane. For example, in the case of
  * projecting to the x-y plane, the coordinte <tt>(3, 1, 4)</tt> would be
  * transformed to <tt>(3, 1, 0)</tt>.
  *
  * \warning WARNING: THIS METHOD MIGHT CHANGE, YET.
  *
  * \return Returns a projection matrix (and not \c *this!).
  *
  * \todo Check whether this method corresponds to Bloomenthal's paper.
  */

template <class T>
Transform3D<T> Transform3D<T>::orthographicProjection(const Plane plane) const
{
    Transform3D<T> transform3d;

    switch(plane)
    {
        case XY_PLANE:
            (*transform3d.m_matrix)(2, 2) = 0;
            break;

        case XZ_PLANE:
            (*transform3d.m_matrix)(1, 1) = 0;
            break;

        case YZ_PLANE:
            (*transform3d.m_matrix)(0, 0) = 0;
            break;

        default:
            ErrorObj error;
            error.setClassName("Transform3D<T>");
            error.setFunctionName("orthographicProjection");
            error.setErrorMessage("Unknown plane.");
            error.setErrorCode(INVALID_ARGUMENT);
    }

    return transform3d;
}


/** \tparam T scalar type
  *
  * \brief Constructs a perspective projection matrix.
  *
  * \param [in] plane   \c XY_PLANE, \c XZ_PLANE, or \c YZ_PLANE
  *
  * The point of view is at the point of origin (0, 0, 0). Every
  * point is projected on a plane with distance d to the point of
  * origin, that is, for instance the origin of the xy-plane is
  * (0, 0, d).
  *
  * \warning WARNING: THIS METHOD MIGHT CHANGE, YET.
  *
  * \return Returns a projection matrix (and not \c *this!).
  *
  * \todo Check whether this method corresponds to Bloomenthal's paper.
  */

template <class T>
Transform3D<T> Transform3D<T>::perspectiveProjection(const T distance, const Plane plane) const
{
    if (distance == 0)
    {
        ErrorObj error;
        error.setClassName("Transform3D<T>");
        error.setFunctionName("perspectiveProjection");
        error.setErrorMessage("Distance may not be zero.");
        error.setErrorCode(INVALID_ARGUMENT);
    }

    Transform3D<T> transform3d;

    switch(plane)
    {
        case XY_PLANE:
            (*transform3d.m_matrix)(2, 3) = distance;
            (*transform3d.m_matrix)(3, 2) = 1.0 / distance;
            break;

        case XZ_PLANE:
            (*transform3d.m_matrix)(1, 3) = distance;
            (*transform3d.m_matrix)(3, 1) = 1.0 / distance;
            break;

        case YZ_PLANE:
            (*transform3d.m_matrix)(0, 3) = distance;
            (*transform3d.m_matrix)(3, 0) = 1.0 / distance;
            break;

        default:
            ErrorObj error;
            error.setClassName("Transform3D<T>");
            error.setFunctionName("perspectiveProjection");
            error.setErrorMessage("Unknown plane.");
            error.setErrorCode(INVALID_ARGUMENT);
    }

    return transform3d;
}


/** \tparam T scalar type
  *
  * \brief Sets the internal matrix to the identity matrix.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::reset()
{
    m_matrix->setIdentity();
    m_is_affine = true;

    return *this;
}


/** \tparam T scalar type
  *
  * \brief Rotation around the x-, y-, or z-axis.
  *
  * \param [in] angle   Rotation angle.
  * \param [in] axis    The \ref Axis "axis".
  * \param [in] unit    Radian or degree.
  *
  * \return \c *this
  *
  * \see rotateX(), rotateY() and rotateZ()
  */

template <class T>
Transform3D<T> & Transform3D<T>::rotate(const double angle, const Axis axis, const Unit unit)
{
    switch (axis)
    {
        case X_AXIS:
            return rotateX(angle, unit);

        case Y_AXIS:
            return rotateY(angle, unit);

        case Z_AXIS:
            return rotateZ(angle, unit);

        default:
            ErrorObj error;
            error.setClassName("Transform3D<T>");
            error.setFunctionName("rotate");
            error.setErrorMessage("Invalid axis.");
            error.setErrorCode(INVALID_AXIS);
            throw error;
    }
}


/** \tparam T scalar type
  *
  * \brief Rotation around the x-axis.
  *
  * \param [in] angle   Angle in degrees or radian.
  * \param [in] unit    The unit of \p angle.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::rotateX(const double angle, const Unit unit)
{
    double angle_in_rad;

    switch (unit)
    {
        case RADIANS:
            angle_in_rad = angle;
            break;

        case DEGREES:
            angle_in_rad = angle / 180.0 * pi;
            break;

        default:
            ErrorObj error;
            error.setClassName("Transform3D<T>");
            error.setFunctionName("rotateX");
            error.setErrorMessage("Invalid unit.");
            error.setErrorCode(INVALID_UNIT);
            throw error;
    }

    Matrix4T rotation;

    const double c = std::cos(angle_in_rad);
    const double s = std::sin(angle_in_rad);

    rotation << 1, 0,  0, 0,
                0, c, -s, 0,
                0, s,  c, 0,
                0, 0,  0, 1;

    *m_matrix = rotation * *m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \brief Rotation around the y-axis.
  *
  * \param [in] angle   Angle in degrees or radian.
  * \param [in] unit    The unit of \p angle.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::rotateY(const double angle, const Unit unit)
{
    double angle_in_rad;

    switch (unit)
    {
        case RADIANS:
            angle_in_rad = angle;
            break;

        case DEGREES:
            angle_in_rad = angle / 180.0 * pi;
            break;

        default:
            ErrorObj error;
            error.setClassName("Transform3D<T>");
            error.setFunctionName("rotateX");
            error.setErrorMessage("Invalid unit.");
            error.setErrorCode(INVALID_UNIT);
            throw error;
    }

    Matrix4T rotation;

    const double c = std::cos(angle_in_rad);
    const double s = std::sin(angle_in_rad);

    rotation <<  c, 0, s, 0,
                 0, 1, 0, 0,
                -s, 0, c, 0,
                 0, 0, 0, 1;

    *m_matrix = rotation * *m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \brief Rotation around the z-axis.
  *
  * \param [in] angle   Angle in degrees or radian.
  * \param [in] unit    The unit of \p angle.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::rotateZ(const double angle, const Unit unit)
{
    double angle_in_rad;

    switch (unit)
    {
        case RADIANS:
            angle_in_rad = angle;
            break;

        case DEGREES:
            angle_in_rad = angle / 180.0 * pi;
            break;

        default:
            ErrorObj error;
            error.setClassName("Transform3D<T>");
            error.setFunctionName("rotateX");
            error.setErrorMessage("Invalid unit.");
            error.setErrorCode(INVALID_UNIT);
            throw error;
    }

    Matrix4T rotation;

    const double c = std::cos(angle_in_rad);
    const double s = std::sin(angle_in_rad);

    rotation << c, -s, 0, 0,
                s,  c, 0, 0,
                0,  0, 1, 0,
                0,  0, 0, 1;

    *m_matrix = rotation * *m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \brief Rotation around an arbitrary axis.
  *
  * \param [in] axis    Axis to rotate around (need not to be normalized).
  * \param [in] angle   Angle in degrees or radian.
  * \param [in] unit    The unit of \p angle.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::rotateAxis(const double angle, const Coordinate<T> & axis, const Unit unit)
{
    double angle_in_rad = 0.0;

    switch (unit)
    {
        case RADIANS:
            angle_in_rad = angle;
            break;

        case DEGREES:
            angle_in_rad = angle / 180.0 * pi;
            break;

        default:
            ErrorObj error;
            error.setClassName("Transform3D<T>");
            error.setFunctionName("rotateAxis");
            error.setErrorMessage("Invalid unit.");
            error.setErrorCode(INVALID_UNIT);
            throw error;
    }

    // The idea is to apply a change of basis such that the
    // axis of rotation becomes the z-axis (i.e. we must find
    // a mapping such that the given axis maps to [0, 0, 1]).
    // Then the rotation can be easily computed and eventually
    // a change back to the original basis can be applied.

    // Create the change-of-basis matrix.

    Coordinate<T> row1, row2, row3;

    row3 = axis.normalized();
    row2 = row3.orthogonal().normalize();
    row1 = row2.cross(row3);

    Matrix4T change_of_basis_matrix;

    change_of_basis_matrix << row1.x(), row1.y(), row1.z(), 0,
                              row2.x(), row2.y(), row2.z(), 0,
                              row3.x(), row3.y(), row3.z(), 0,
                                     0,        0,        0, 1;

    // Rotate around the z-axis in the changed basis.

    *m_matrix = change_of_basis_matrix * *m_matrix;
    rotateZ(angle_in_rad);
    *m_matrix = change_of_basis_matrix.inverse() * *m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \brief Rotation around an arbitrary axis.
  *
  * \param [in] axis    Axis to rotate around (need not to be normalized).
  * \param [in] angle   Angle in degrees or radian.
  * \param [in] unit    The unit of \p angle.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::rotateAxis(const double angle, const Vector3T & axis, const Unit unit)
{
    coordinate_type coord(axis);
    return this->rotateAxis(angle, coord, unit);
}


/** \tparam T scalar type
  *
  * \brief Rotation around an arbitrary axis.
  *
  * \param [in] axis    Axis to rotate around (need not to be normalized).
  * \param [in] angle   Angle in degrees or radian.
  * \param [in] unit    The unit of \p angle.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::rotateAxis(const double angle, const Vector4T & axis, const Unit unit)
{
    coordinate_type coord(axis);
    return this->rotateAxis(angle, coord, unit);
}


/** \tparam T scalar type
  *
  * \param [in] sx  Scaling factor in x direction.
  * \param [in] sy  Scaling factor in y direction.
  * \param [in] sz  Scaling factor in z direction.
  *
  * \brief Non-uniform scaling.
  *
  * This function scales the axes of the currently internally saved
  * transformation.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::scale(const T sx, const T sy, const T sz)
{
    Matrix4T scale;

    scale << sx,  0,  0,  0,
              0, sy,  0,  0,
              0,  0, sz,  0,
              0,  0,  0,  1;

    *m_matrix = scale * *m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] dx  Translation in x direction.
  * \param [in] dy  Translation in y direction.
  * \param [in] dz  Translation in z direction.
  *
  * \brief Translation.
  *
  * This function adds a translation to the currently stored transformation.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::translate(const T dx, const T dy, const T dz)
{
    Matrix4T translation;

    translation << 1, 0, 0, dx,
                   0, 1, 0, dy,
                   0, 0, 1, dz,
                   0, 0, 0,  1;

    *m_matrix = translation * *m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] position  Translation (must be a coordinate of size 3)
  *
  * \brief Translation.
  *
  * This function adds a translation to the currently stored transformation.
  *
  * \return \c *this
  */

template <class T>
Transform3D<T> & Transform3D<T>::translate(const Coordinate<T> & position)
{
    assert(position.size() == 3);

    Matrix4T translation;

    translation << 1, 0, 0, position.x(),
                   0, 1, 0, position.y(),
                   0, 0, 1, position.z(),
                   0, 0, 0, 1;

    *m_matrix = translation * *m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \brief Returns whether the internally saved transformation is affine or not.
  *
  * \return Returns true, if the internal matrix is affine.
  */

template <class T>
inline bool Transform3D<T>::is_affine() const
{
    return m_is_affine;
}


/** \tparam T scalar type
  *
  * \param [in] vec A homogeneous coordinate.
  *
  * \brief Internal function.
  *
  * This function normalizes a homogeneous coordinate, i.e. it divides all
  * components by its last component.
  *
  * \return A normalized homogeneous coordinate.
  *
  * \throw ErrorObj If the last component of \p vec is zero, an error object is
  *                 thrown and its error code set to DIVISION_BY_ZERO.
  */

template <class T>
inline void Transform3D<T>::normalize(Vector4T & vec) const
{
    if (Tools::isZero<T>(vec(3)))
    {
        ErrorObj error;
        error.setClassName("Transform3D");
        error.setFunctionName("normalize");
        error.setErrorMessage("Division by zero.");
        error.setErrorCode(DIVISION_BY_ZERO);
        throw error;
    }
    else
    {
        vec(0) /= vec(3);
        vec(1) /= vec(3);
        vec(2) /= vec(3);
        vec(3) = 1;
    }
}


} // namespace TRTK


#endif // TRANSFORM3D_HPP_0386836041
