/*
    Computes a 2D transform in homogeneous coordinates.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.2 (2011-09-24)
*/

/** \file Transform2D.hpp
  * \brief This file contains the \ref TRTK::Transform2D "Transform2D" class.
  */

/* Note: The concatenation of several affine matrices to one single
 *       matrix is possible without any problems. However, if a
 *       non-affine transform (e.g. a projective transform) is used,
 *       it may only be used once and at the very end (i.e. multiplied
 *       from left) due to the subsequent non-linear normalisation step.
 */


#ifndef TRANSFORM2D_HPP_9785873423
#define TRANSFORM2D_HPP_9785873423


#include <cassert>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "Tools.hpp"


namespace TRTK
{


/** \class Transform2D
  *
  * \brief Affine or projective transformation in 2D.
  *
  * This class provides means to transform 2D coordinates via an affine or a
  * projective transformation. The transformation can be composed from a
  * sequence of elementary transformations such as translation, scaling,
  * rotation etc., or by directly setting the entries of a \f$ 3 \times 3 \f$
  * transformation matrix. The coordinates may be given as 2D or as 3D
  * coordinates in case of \ref HomogeneousCoordinates "homogeneous coordinates".
  *
  * The transformation is computed by multiplying a \f$ 3 \times 3 \f$
  * matrix with a 3D homogeneous coordinate, followed by a potential normalization
  * step. That is, if \f$ x \f$ is transformed to \f$ y \f$, first an intermediate
  * vector \f$ w \f$ is computed
  *
  * \f[
  * \begin{pmatrix}
  *     w_1  \\  w_2  \\  w_3
  * \end{pmatrix}
  * =
  * \begin{pmatrix}
  *     a_{11}  &  a_{12}  &  a_{13}  \\
  *     a_{21}  &  a_{22}  &  a_{23}  \\
  *     a_{31}  &  a_{32}  &  a_{33}
  * \end{pmatrix}
  * \begin{pmatrix}
  *     x_1  \\  x_2  \\  1
  * \end{pmatrix}
  * \f]
  *
  * followed by a normalization step
  *
  * \f[
  * \begin{pmatrix}
  *     y_1  \\  y_2  \\  1
  * \end{pmatrix}
  * = \frac{1}{w_3}
  * \begin{pmatrix}
  *     w_1  \\  w_2  \\  w_3
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
  *     y_1  \\  y_2  \\  1
  * \end{pmatrix}
  * =
  * \begin{pmatrix}
  *     a_{11}  &  a_{12}  &  b_1  \\
  *     a_{21}  &  a_{22}  &  b_2  \\
  *     0       &  0       &  1
  * \end{pmatrix}
  * \begin{pmatrix}
  *     x_1  \\  x_2  \\  1
  * \end{pmatrix}
  * \f]
  *
  * the subsequent normalization step can be omitted, since the last entry of the
  * above intermediate vector \f$ w \f$ would be \f$ 1 \f$ in any event.
  *
  * Most functions check for certain assertions (e.g., if the coordinate size is
  * valid) and trigger an assertion failure, if the assumption does not hold.
  * This is meant for debugging purposes only, and can be disabled by defining
  * the macro \macro{NDEBUG}.
  *
  * Here are some examples of how to use the Transform2D class:
  *
  * Example 1:
  *
  * \code
  *
  * TRTK::Coordinate<double> coordinate(1, 4);
  * TRTK::Transform2D<double> transform;
  *
  * const double pi = transform::pi;
  * transform.rotate(pi/2).translate(2, 0);
  *
  * cout << transform * coordinate << endl;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * (-2, 1)
  *
  * \endcode
  *
  * Example 2:
  *
  * \code
  *
  * using namespace TRTK;
  *
  * Coordinate<double> coordinate(1, 4);
  *
  * Transform2D<double> transform1;
  * Transform2D<double> transform2;
  * Transform2D<double> transform3;
  * Transform2D<double> transform4;
  *
  * transform1.translate(1, 1);
  *
  * // rotation of 90 degrees
  * transform2.a11() =  0;
  * transform2.a12() =  1;
  * transform2.a21() = -1;
  * transform2.a22() =  0;
  *
  * transform3.translate(-1, 0);
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
  * (4, -2)
  * (4, -2)
  *
  *  0  1  0
  * -1  0 -1
  *  0  0  1
  *
  * \endcode
  *
  * \see Transform2D and Coordinate
  *
  * \author Christoph Haenisch
  * \version 0.1.2
  * \date last changed on 2011-09-24
  */

template <class T>
class Transform2D
{
public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Unit  {DEGREES, RADIANS};

    enum Error {DIVISION_BY_ZERO,
                INVALID_ARGUMENT,
                INVALID_UNIT,
                UNKNOWN_ERROR,
                WRONG_COORDINATE_SIZE};

    typedef T value_type;
    typedef Eigen::Matrix<T, 3, 3> Matrix3T;
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Matrix<T, 2, 1> Vector2T;
    typedef Coordinate<T> coordinate_type;

    Transform2D();

    Transform2D(const Matrix3T &);

    Transform2D(T a11, T a12, T a13,
                T a21, T a22, T a23,
                T a31, T a32, T a33);

    template <class U>
    Transform2D(const Transform2D<U> & transform2D);

    virtual ~Transform2D();

    T & a11();
    T & a12();
    T & a13();
    T & a21();
    T & a22();
    T & a23();
    T & a31();
    T & a32();
    T & a33();

    const T & a11() const;
    const T & a12() const;
    const T & a13() const;
    const T & a21() const;
    const T & a22() const;
    const T & a23() const;
    const T & a31() const;
    const T & a32() const;
    const T & a33() const;

    const Coordinate<T> operator* (const Coordinate<T> &) const;
    const Matrix3T operator* (const Matrix3T &) const;
    const Vector2T operator* (const Vector2T &) const;
    const Vector3T operator* (const Vector3T &) const;
    const Transform2D operator* (const Transform2D &) const;
    const Transform2D operator>> (const Transform2D &) const;

    Matrix3T & getTransformationMatrix();
    const Matrix3T & getTransformationMatrix() const;

    const Transform2D inverse() const;

    Transform2D & reset();

    Transform2D & rotate(const double angle, const Unit unit = RADIANS);

    Transform2D & scale(const T sx, const T sy);

    Transform2D & shear(const T sx, const T sy);

    Transform2D & translate(const T dx, const T dy);
    Transform2D & translate(const Coordinate<T> & position);

    bool is_affine() const;

    static const double pi;

private:

    void normalize(Vector3T &) const;

    Matrix3T m_matrix;
    bool m_is_affine;
};


template <class T>
const double Transform2D<T>::pi = 3.14159265358979323846264338327950288419716939937510;


/** \tparam T scalar type
  *
  * \brief Constructs an instance of Transform2D.
  *
  * The internal matrix is set to the identity matrix.
  *
  * \see reset()
  */

template <class T>
Transform2D<T>::Transform2D() : m_is_affine(true)
{
    m_matrix.setIdentity();
}


/** \tparam T scalar type
  *
  * \param [in] matrix  Tranformation matrix.
  *
  * \brief Constructs an instance of Transform2D.
  *
  * Sets the internal matrix to the given \p matrix.
  *
  * \see reset()
  */

template <class T>
Transform2D<T>::Transform2D(const Matrix3T & matrix)
{
    using Tools::isEqual;

    m_matrix = matrix;

    if (isEqual<T>(matrix(2,0), 0) &&
        isEqual<T>(matrix(2,1), 0) &&
        isEqual<T>(matrix(2,2), 1))
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
  * \brief Constructs an instance of Transform2D.
  *
  * Sets the internal matrix to the given coefficients.
  *
  * \see reset()
  */

template <class T>
Transform2D<T>::Transform2D(T a11, T a12, T a13,
                            T a21, T a22, T a23,
                            T a31, T a32, T a33)
{
    using Tools::isEqual;

    m_matrix << a11, a12, a13,
                a21, a22, a23,
                a31, a32, a33;

    if (isEqual<T>(a31, 0) &&
        isEqual<T>(a32, 0) &&
        isEqual<T>(a33, 1))
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
Transform2D<T>::Transform2D(const Transform2D<U> & transform2D)
{
    const int rows = transform2D.getTransformationMatrix().rows();
    const int cols = transform2D.getTransformationMatrix().cols();

    m_matrix.resize(rows, cols);

    for (int m = 0; m < rows; ++m)
    {
        for (int n = 0; n < cols; ++n)
        {
            m_matrix(m, n) = T(transform2D.getTransformationMatrix()(m, n));
        }
    }

    m_is_affine = transform2D.is_affine();
}


/** \tparam T scalar type
  *
  * \brief Destroys the instance of Transform2D.
  */

template <class T>
Transform2D<T>::~Transform2D()
{
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
T & Transform2D<T>::a11()
{
    return m_matrix(0, 0);
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
T & Transform2D<T>::a12()
{
    return m_matrix(0, 1);
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
T & Transform2D<T>::a13()
{
    return m_matrix(0, 2);
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
T & Transform2D<T>::a21()
{
    return m_matrix(1, 0);
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
T & Transform2D<T>::a22()
{
    return m_matrix(1, 1);
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
T & Transform2D<T>::a23()
{
    return m_matrix(1, 2);
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
T & Transform2D<T>::a31()
{
    m_is_affine = false;
    return m_matrix(2, 0);
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
T & Transform2D<T>::a32()
{
    m_is_affine = false;
    return m_matrix(2, 1);
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
T & Transform2D<T>::a33()
{
    m_is_affine = false;
    return m_matrix(2, 2);
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
const T & Transform2D<T>::a11() const
{
    return m_matrix(0, 0);
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
const T & Transform2D<T>::a12() const
{
    return m_matrix(0, 1);
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
const T & Transform2D<T>::a13() const
{
    return m_matrix(0, 2);
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
const T & Transform2D<T>::a21() const
{
    return m_matrix(1, 0);
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
const T & Transform2D<T>::a22() const
{
    return m_matrix(1, 1);
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
const T & Transform2D<T>::a23() const
{
    return m_matrix(1, 2);
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
const T & Transform2D<T>::a31() const
{
    return m_matrix(2, 0);
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
const T & Transform2D<T>::a32() const
{
    return m_matrix(2, 1);
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
const T & Transform2D<T>::a33() const
{
    return m_matrix(2, 2);
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
  * The given \p coordinate must be of size two or three. If \p coordinate has
  * a size of three (homogeneous coordinate), it is assumed that its last
  * component is equal to one.
  *
  * Example:
  *
  * \code
  * TRTK::Coordinate<double> coordinate(1, 4);
  * TRTK::Transform2D<double> transform;
  *
  * const double pi = TRTK::Transform2D<double>::pi;
  * transform.rotate(pi/2).translate(2, 0);
  *
  * cout << transform * coordinate << endl;
  * \endcode
  *
  * Output:
  *
  * \code
  * (-2, 1)
  * \endcode
  */

template <class T>
const Coordinate<T> Transform2D<T>::operator* (const Coordinate<T> & coordinate) const
{
    // 'coordinate' must be of size two or three. If 'coordinate' has a size
    // of three, it is assumed that the third component is equal to one.

    const int size = coordinate.size();

    if (!(size == 2 || size == 3))
    {
        ErrorObj error;
        error.setClassName("Transform2D<T>");
        error.setFunctionName("operator*");
        error.setErrorMessage("'coordinate' must be of size two or three.");
        error.setErrorCode(WRONG_COORDINATE_SIZE);
        throw error;
    }

    assert(coordinate.size() == 2 || (coordinate.size() == 3 && coordinate.w() == 1));

    Vector3T vector(coordinate.x(), coordinate.y(), 1);
    Vector3T result = m_matrix * vector;

    if (!is_affine())
    {
        normalize(result);
    }

    if (size == 2)
    {
        return Coordinate<T>(result(0), result(1));
    }
    else // size == 3
    {
        return Coordinate<T>(result(0), result(1), 1);
    }
}


/** \tparam T scalar type
  *
  * \param [in] vector  2D vector to be transformed.
  *
  * \return Transformed 2D vector.
  *
  * \brief Transforms a 2D vector with the internally saved transformation.
  *
  * Example:
  *
  * \code
  *
  * TRTK::Transform2D<double> transform;
  * TRTK::Transform2D<double>::Vector2T vec(1, 4);
  *
  * const double pi = TRTK::Transform2D<double>::pi;
  * transform.rotateZ(pi/2).translate(2, 0);
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
  * \endcode
  */

template <class T>
const typename Transform2D<T>::Vector2T Transform2D<T>::operator* (const Vector2T & vector) const
{
    Vector3T vector3D(vector(0), vector(1), 1);
    Vector3T result = m_matrix * vector3D;

    if (!is_affine())
    {
        normalize(result);
    }

    return result.head(2);
}


/** \tparam T scalar type
  *
  * \param [in] vector 3D vector to be transformed.
  *
  * \return Transformed 3D vector.
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
  * TRTK::Transform2D<double> transform;
  * TRTK::Transform2D<double>::Vector3T vec(1, 4, 1);
  *
  * const double pi = TRTK::Transform2D<double>::pi;
  * transform.rotateZ(pi/2).translate(2, 0);
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
  * 1
  * \endcode
  */

template <class T>
const typename Transform2D<T>::Vector3T Transform2D<T>::operator* (const Vector3T & vector) const
{
    assert(Tools::isEqual<T>(vector(2), 1));

    Vector3T result = m_matrix * vector;

    if (!is_affine())
    {
        normalize(result);
    }

    return result;
}


/** \tparam T scalar type
  *
  * \param [in] matrix 3x3 transformation matrix.
  *
  * \return 3x3 transformation matrix.
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
  * typedef TRTK::Transform2D<double> Transform;
  *
  * Transform T1(1,  1,  0,
  *              0,  2,  2,
  *              0,  0,  1);
  *
  * Transform::Matrix3T A;
  *
  * A << 1,  0,  3,
  *      0, -1,  0,
  *      0,  0,  1;
  *
  * Transform T2 = T1 * A;
  *
  * std::cout << T2.getTransformationMatrix();
  * \endcode
  *
  * Output:
  *
  * \code
  * 1 -1  3
  * 0 -2  2
  * 0  0  1
  * \endcode
  */

template <class T>
const typename Transform2D<T>::Matrix3T Transform2D<T>::operator* (const Matrix3T & matrix) const
{
    typename Transform2D<T>::Matrix3T result = m_matrix * matrix;

    return result;
}


/** \tparam T scalar type
  *
  * \param [in] transform A transformation described by Transform2D.
  *
  * \return A Transform2D object describing the composition of the two
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
  * typedef TRTK::Transform2D<double> Transform;
  *
  * Transform T1(1,  1,  0,
  *              0,  2,  2,
  *              0,  0,  1);
  *
  * Transform T2(1,  0,  3,
  *              0, -1,  0,
  *              0,  0,  1);
  *
  * Transform T3 = T1 * T2;
  *
  * std::cout << T3.getTransformationMatrix();
  * \endcode
  *
  * Output:
  *
  * \code
  * 1 -1  3
  * 0 -2  2
  * 0  0  1
  * \endcode
  *
  * \see operator>>()
  */

template <class T>
const Transform2D<T> Transform2D<T>::operator* (const Transform2D & transform) const
{
    Transform2D<T> result = (m_matrix * transform.m_matrix).eval();
    result.m_is_affine = m_is_affine && transform.m_is_affine;

    return result;
}


/** \tparam T scalar type
  *
  * \param [in] transform A transformation described by Transform2D.
  *
  * \return A Transform2D object describing the composition of the two
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
  * typedef TRTK::Transform2D<double> Transform;
  *
  * Transform T1(1,  1,  0,
  *              0,  2,  2,
  *              0,  0,  1);
  *
  * Transform T2(1,  0,  3,
  *              0, -1,  0,
  *              0,  0,  1);
  *
  * Transform T3 = T1 >> T2; // == T2 * T1
  *
  * std::cout << T3.getTransformationMatrix();
  * \endcode
  *
  * Output:
  *
  * \code
  * 1  1  3
  * 0 -2 -2
  * 0  0  1
  * \endcode
  *
  * \see operator*()
  */

template <class T>
const Transform2D<T> Transform2D<T>::operator>> (const Transform2D & transform) const
{
    Transform2D<T> result = (transform.m_matrix * m_matrix).eval();
    result.m_is_affine = m_is_affine && transform.m_is_affine;

    return result;
}


/** \tparam T scalar type
  *
  * \return Returns the internal transformation as 3x3 matrix representation.
  *
  * \brief Returns the internal transformation matrix.
  */

template <class T>
typename Transform2D<T>::Matrix3T & Transform2D<T>::getTransformationMatrix()
{
    return m_matrix;
}


/** \tparam T scalar type
  *
  * \return Returns the internal transformation as 3x3 matrix representation.
  *
  * \brief Returns the internal transformation matrix.
  */

template <class T>
const typename Transform2D<T>::Matrix3T & Transform2D<T>::getTransformationMatrix() const
{
    return m_matrix;
}


/** \tparam T scalar type
  *
  * \brief Returns the inverse transformation.
  *
  * \return A new copy.
  */

template <class T>
const Transform2D<T> Transform2D<T>::inverse() const
{
    return Transform2D(m_matrix.inverse());
}


/** \tparam T scalar type
  *
  * \brief Sets the internal matrix to the identity matrix.
  *
  * \return \c *this
  */

template <class T>
Transform2D<T> & Transform2D<T>::reset()
{
    m_matrix.setIdentity();
    m_is_affine = true;

    return *this;
}


/** \tparam T scalar type
  *
  * \brief Rotation.
  *
  * \param [in] angle   Angle in degrees or radian.
  * \param [in] unit    The unit of \p angle.
  *
  * \return \c *this
  */

template <class T>
Transform2D<T> & Transform2D<T>::rotate(const double angle, const Unit unit)
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
            error.setClassName("Transform2D<T>");
            error.setFunctionName("rotateX");
            error.setErrorMessage("Invalid unit.");
            error.setErrorCode(INVALID_UNIT);
            throw error;
    }

    Matrix3T rotation;

    const double c = std::cos(angle_in_rad);
    const double s = std::sin(angle_in_rad);

    rotation << c, -s, 0,
                s,  c, 0,
                0,  0, 1;

    m_matrix = rotation * m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] sx  Scaling factor in x direction.
  * \param [in] sy  Scaling factor in y direction.
  *
  * \brief Non-uniform scaling.
  *
  * This function scales the axes of the currently internally saved
  * transformation.
  *
  * \return \c *this
  */

template <class T>
Transform2D<T> & Transform2D<T>::scale(const T sx, const T sy)
{
    Matrix3T scale;

    scale << sx,  0,  0,
              0, sy,  0,
              0,  0,  1;

    m_matrix = scale * m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] sx  Horizontal shear (parallel to the x-axis).
  * \param [in] sy  Vertical shear (parallel to the y-axis).
  *
  * \brief Shear mapping.
  *
  * This function performs a shear mapping on the currently internally saved
  * transformation.
  *
  * \return \c *this
  */

template <class T>
Transform2D<T> & Transform2D<T>::shear(const T sx, const T sy)
{
    Matrix3T shear;

    shear << 1, sx,  0,
             sy, 1,  0,
             0,  0,  1;

    m_matrix = shear * m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] dx  Translation in x direction.
  * \param [in] dy  Translation in y direction.
  *
  * \brief Translation.
  *
  * This function adds a translation to the currently stored transformation.
  *
  * \return \c *this
  */

template <class T>
Transform2D<T> & Transform2D<T>::translate(const T dx, const T dy)
{
    Matrix3T translation;

    translation << 1, 0, dx,
                   0, 1, dy,
                   0, 0, 1;

    m_matrix = translation * m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] position  Translation (must be a coordinate of size 2)
  *
  * \brief Translation.
  *
  * This function adds a translation to the currently stored transformation.
  *
  * \return \c *this
  */

template <class T>
Transform2D<T> & Transform2D<T>::translate(const Coordinate<T> & position)
{
    assert(position.size() == 2);

    Matrix3T translation;

    translation << 1, 0, position.x(),
                   0, 1, position.y(),
                   0, 0, 1;

    m_matrix = translation * m_matrix;

    return *this;
}


/** \tparam T scalar type
  *
  * \brief Returns whether the internally saved transformation is affine or not.
  *
  * \return Returns true, if the internal matrix is affine.
  */

template <class T>
inline bool Transform2D<T>::is_affine() const
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
inline void Transform2D<T>::normalize(Vector3T & vec) const
{
    if (Tools::isZero<T>(vec(2)))
    {
        ErrorObj error;
        error.setClassName("Transform2D");
        error.setFunctionName("normalize");
        error.setErrorMessage("Division by zero.");
        error.setErrorCode(DIVISION_BY_ZERO);
        throw error;
    }
    else
    {
        vec(0) /= vec(2);
        vec(1) /= vec(2);
        vec(2) = 1;
    }
}


} // namespace TRTK


#endif // TRANSFORM2D_HPP_9785873423
