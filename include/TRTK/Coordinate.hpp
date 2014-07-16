/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.7.1 (2013-08-20)
*/

/** \file Coordinate.hpp
  * \brief This file contains the \ref TRTK::Coordinate "Coordinate" class
  *        and related functions.
  */

#ifndef COORDINATE_HPP_1023074891
#define COORDINATE_HPP_1023074891


#include <assert.h>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Tools.hpp"
#include "ErrorObj.hpp"

#ifdef TRTK_SUPPORT_CVECTOR
#include <Mathbib/Vector.h>
#endif // TRTK_SUPPORT_CVECTOR


namespace TRTK
{

// Forward declarations

namespace Tools
{
    template <class T>
    bool isZero(const T value);

    template <class T>
    T rand();

    template <class T>
    T rand(T a, T b);

    template <class T>
    T randn();

    template <class T>
    T randn(T mu, T sigma);
}

/** \tparam T scalar type
  *
  * \brief A generic coordinate class.
  *
  * Coordinate represents an ordinary coordinate vector of arbitrary size.
  * Its elements (i.e. the scalars) are of type \c T and can be freely chosen,
  * thus also allowing the use of classes with arbitrary precision (like CLN or
  * GMP), for instance. Conversions between distinctive specializations like
  * \c Coordinate<int> and \c Coordinate<double> are done fully automatically.
  * The size of a coordinate can be changed at any time; Coordinate is a column
  * vector.
  *
  * Most functions check for certain assertions (e.g. if an index is in a valid
  * range) and trigger an assertion failure if the assumption does not hold.
  * This is meant for debugging purposes only and can be disabled by defining
  * the macro \macro{NDEBUG}.
  *
  * Since Coordinate builds on top of <a href=http://eigen.tuxfamily.org>Eigen</a>,
  * it can be seamlessly converted to \c Eigen::Array using the toArray() method.
  *
  * Here are some examples:
  *
  * \code
  *
  * using namespace TRTK;
  *
  * Coordinate<double> c1(1, 0);
  * Coordinate<double> c2(0, 1);
  *
  * Coordinate<double> c3 = c1 + c2 * 2 + 2;
  *
  * cout << c3        << endl
  *      << c3.norm() << endl
  *      << (c3, 1)   << endl;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * (3, 4)
  * 5
  * (3, 4, 1)
  *
  * \endcode
  *
  * \warning Be careful when performing computations with \e two \e coordinates
  *          having different scalar types! The type of the result is the same
  *          as the one of the first operand:
  *          \code
  * // use typedefs due to the limited space in this box...
  *
  * typedef Coordinate<double> CoordinateD;
  * typedef Coordinate<int> CoordinateI;
  *
  * CoordinateD(1, 1) * CoordinateI(1, 1);       // result is of type CoordinateD
  * CoordinateI(1, 1) * CoordinateD(1, 1));      // result is of type CoordinateI
  *
  *          \endcode
  *          In all \b other \b cases automatic type conversion works \b as
  *          \b expected:
  *          \code
  *
  * CoordinateI(1, 1) * 1;                       // result is of type CoordinateI
  * CoordinateI(1, 1) * 1.0;                     // result is of type CoordinateI
  *
  * CoordinateD(1, 1) * 1;                       // result is of type CoordinateD
  * CoordinateD(1, 1) * 1.0;                     // result is of type CoordinateD
  *
  *   1 * CoordinateI(1, 1);                     // result is of type CoordinateI
  * 1.0 * CoordinateI(1, 1);                     // result is of type CoordinateI
  *
  *   1 * CoordinateD(1, 1);                     // result is of type CoordinateD
  * 1.0 * CoordinateD(1, 1);                     // result is of type CoordinateD
  *
  * CoordinateD(1, 1) * CoordinateD(1, 1);       // result is of type CoordinateD
  *
  *          \endcode
  *
  * \macros If \macro{TRTK_SUPPORT_CVECTOR} is defined, Coordinate will
  *         support the mediTEC \c CVector class. That is, an constructor as
  *         well as a conversion operator is defined.
  *
  * \see Transform2D and Transform3D
  *
  * \author Christoph Haenisch
  * \version 0.7.1
  * \date last changed on 2013-08-20
  */

template <class T>
class Coordinate
{
public:
    typedef T value_type;
    typedef Eigen::Array<T, Eigen::Dynamic, 1> data_type;                   ///< See also http://eigen.tuxfamily.org.
    typedef const Eigen::Array<T, Eigen::Dynamic, 1> const_data_type;       ///< See also http://eigen.tuxfamily.org.
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> matrix_type;                ///< See also http://eigen.tuxfamily.org.

    Coordinate();               ///< Constructs a zero-dimensional coordinate.
    Coordinate(T);              ///< Constructs a one-dimensional coordinate.
    Coordinate(T, T);           ///< Constructs a two-dimensional coordinate.
    Coordinate(T, T, T);        ///< Constructs a three-dimensional coordinate.
    Coordinate(T, T, T, T);     ///< Constructs a four-dimensional coordinate.

    Coordinate(const Coordinate &);

    template <class U>
    Coordinate(const Coordinate<U> & coordinate);

    #ifdef TRTK_SUPPORT_CVECTOR
    Coordinate(const CVector &);
    #endif // TRTK_SUPPORT_CVECTOR

    template <typename Derived>
    explicit Coordinate(const Eigen::DenseBase<Derived> & vector); ///< Constructs a coordinate from an Eigen Array or Eigen Matrix.

    virtual ~Coordinate();

    T & x();                ///< Returns the first component.
    T & y();                ///< Returns the second component.
    T & z();                ///< Returns the third component.
    T & w();                ///< Returns the fourth component.

    const T & x() const;    ///< Returns the first component.
    const T & y() const;    ///< Returns the second component.
    const T & z() const;    ///< Returns the third component.
    const T & w() const;    ///< Returns the fourth component.

    T & operator[](int);
    const T & operator[](int) const;

    bool operator==(const Coordinate &) const;

    Coordinate & operator+=(T);
    Coordinate & operator-=(T);
    Coordinate & operator*=(T);
    Coordinate & operator/=(T);

    Coordinate & operator+=(const Coordinate &);        ///< Component-wise addition.
    Coordinate & operator-=(const Coordinate &);        ///< Component-wise subtraction.
    Coordinate & operator*=(const Coordinate &);        ///< Component-wise multiplication.
    Coordinate & operator/=(const Coordinate &);        ///< Component-wise division.

    Coordinate operator+(T) const;
    Coordinate operator-(T) const;
    Coordinate operator*(T) const;
    Coordinate operator/(T) const;

    Coordinate operator+(const Coordinate &) const;     ///< Component-wise addition.
    Coordinate operator-(const Coordinate &) const;     ///< Component-wise subtraction.
    Coordinate operator*(const Coordinate &) const;     ///< Component-wise multiplication.
    Coordinate operator/(const Coordinate &) const;     ///< Component-wise division.

    Coordinate operator,(T) const;                      ///< Returns a copy of the coordinate, enlarged by one component.

    operator data_type();                               ///< Conversion operator; yields an Eigen 3 array.
    operator const_data_type() const;                   ///< Conversion operator; yields an Eigen 3 array.

    #ifdef TRTK_SUPPORT_CVECTOR
    operator CVector() const;                           ///< Conversion operator; yields an CVector.
    #endif // TRTK_SUPPORT_CVECTOR

    Coordinate cross(const Coordinate &) const;         ///< Computes a cross product.
    double dot(const Coordinate &) const;               ///< Computes a dot product.
    Coordinate & fill(T);
    double norm() const;                                ///< Returns the Euclidean norm.
    Coordinate & normalize();                           ///< Normalizes \c *this.
    Coordinate normalized() const;                      ///< Returns a normalized copy of \c *this.
    Coordinate orthogonal() const;                      ///< Returns a vector that is orthogonal to \c *this.
    Coordinate orthonormal() const;                     ///< Returns a normalized vector that is orthogonal to \c *this.
    Coordinate & reserve(unsigned int size);            ///< Reserves memory to store \c size elements.
    Coordinate & resize(unsigned int size, T value = T());
    unsigned int size() const;
    double squaredNorm() const;                         ///< Returns the squared Euclidean norm.

    data_type & toArray();                              ///< Returns an Eigen 3 array.
    const data_type & toArray() const;                  ///< Returns an Eigen 3 array.

    matrix_type toMatrix() const;                       ///< Returns an Eigen 3 matrix.

    /** \enum Error Error codes that might be set when ErrorObj is thrown. */

    enum Error {DIVISION_BY_ZERO,                       ///< a division by zero occurred
                UNKNOWN_ERROR                           ///< an unknown error occurred
    };

    static Coordinate rand(unsigned int size, T a = T(0), T b = T(1));              ///< Returns a vector filled with uniformly distributed random samples.
    static Coordinate randn(unsigned int size, T mu = T(0), T sigma =  T(1));       ///< Returns a vector filled with normally distributed random samples.

private:
    data_type m_data;
};


// The subsequent functions allow expressions, where the scalar is in front
// of the coordinate. See Corrdinate.cpp for their implementations

Coordinate<double> operator+(int, const Coordinate<double> &);


Coordinate<double> operator-(int, const Coordinate<double> &);


Coordinate<double> operator*(int, const Coordinate<double> &);


Coordinate<double> operator/(int, const Coordinate<double> &);


Coordinate<int> operator+(double, const Coordinate<int> &);


Coordinate<int> operator-(double, const Coordinate<int> &);


Coordinate<int> operator*(double, const Coordinate<int> &);


Coordinate<int> operator/(double, const Coordinate<int> &);


template <class T>
Coordinate<T> operator+(T, const Coordinate<T> & coordinate);


template <class T>
Coordinate<T> operator-(T, const Coordinate<T> & coordinate);


template <class T>
Coordinate<T> operator*(T, const Coordinate<T> & coordinate);


template <class T>
Coordinate<T> operator/(T, const Coordinate<T> & coordinate);


// Stream operators

template <class T>
std::istream & operator >> (std::istream &, Coordinate<T> &);           ///< Reads Coordinate from input stream.


template <class T>
std::ostream & operator << (std::ostream &, const Coordinate<T> &);     ///< Writes Coordinate to output stream.



// Arithmetical functions

template <class T>
TRTK::Coordinate<T> sqrt(const TRTK::Coordinate<T> & coordinate);


/////////////////////////////////////////////////////////////////////////////////////


template <class T>
Coordinate<T>::Coordinate()
{
}


/** \tparam T scalar type
  *
  * \param [in] x first element
  */

template <class T>
Coordinate<T>::Coordinate(T x) : m_data(1)
{
    m_data(0) = x;
}


/** \tparam T scalar type
  *
  * \param [in] x first element
  * \param [in] y second element
  */

template <class T>
Coordinate<T>::Coordinate(T x, T y) : m_data(2)
{
    m_data(0) = x;
    m_data(1) = y;
}


/** \tparam T scalar type
  *
  * \param [in] x first element
  * \param [in] y second element
  * \param [in] z third element
  */

template <class T>
Coordinate<T>::Coordinate(T x, T y, T z) : m_data(3)
{
    m_data(0) = x;
    m_data(1) = y;
    m_data(2) = z;
}


/** \tparam T scalar type
  *
  * \param [in] x first element
  * \param [in] y second element
  * \param [in] z third element
  * \param [in] w fourth element
  */

template <class T>
Coordinate<T>::Coordinate(T x, T y, T z, T w) : m_data(4)
{
    m_data(0) = x;
    m_data(1) = y;
    m_data(2) = z;
    m_data(3) = w;
}


/** Copy constructor.
  *
  * \tparam T scalar type
  *
  * \param [in] coordinate coordinate of the same type
  */

template <class T>
Coordinate<T>::Coordinate(const Coordinate & coordinate)
{
    m_data = coordinate.m_data;
}


/** Copy constructor.
  *
  * \tparam T scalar type of the newly created coordinate
  * \tparam U scalar type of the copied coordinate
  *
  * \param [in] coordinate Coordinate with scalar type \c U
  */

template <class T>
template <class U>
Coordinate<T>::Coordinate(const Coordinate<U> & coordinate)
{
    const unsigned int size = coordinate.size();

    m_data.resize(size);

    for (unsigned i = 0; i < size; ++i)
    {
        m_data(i) = static_cast<T>(coordinate[i]);
    }
}


#ifdef TRTK_SUPPORT_CVECTOR

/** Constructs a Coordinate.
  *
  * \tparam T scalar type
  *
  * \param [in] cVector instance of the \c CVector class
  *
  * This constructor converts a \c CVector into a Coordinate.
  *
  * \note \macro{TRTK_SUPPORT_CVECTOR} must be defined to enable this
  *       function.
  */

template <class T>
Coordinate<T>::Coordinate(const CVector & cVector)
{
    const unsigned int size = cVector.GetSize();

    this->resize(size);

    for (int i = 0; i < size; ++i)
    {
        m_data[i] = cVector(i);
    }
}

#endif // TRTK_SUPPORT_CVECTOR


/** \tparam T           scalar type of the newly created coordinate
  * \tparam Derived     actual type of vector
  *
  * \param [in] vector  Eigen Array or Eigen Matrix
  *
  * The type of \p vector must be \c T, otherwise an assertion is thrown.
  *
  * Example:
  *
  * \code
  * Eigen::Array3d array(1, 2, 3);
  * TRTK::Coordinate<double> coordinate(array);
  * \endcode
  *
  * \note The constructor is defined \e explicit to avoid automatic conversion
  *       clashes with the \e Eigen libraray.
  *
  * \see Eigen (http://eigen.tuxfamily.org)
  */

template <class T>
template <typename Derived>
Coordinate<T>::Coordinate(const Eigen::DenseBase<Derived> & vector)
{
    assert(vector.rows() == 1 || vector.cols() == 1);
    m_data = vector; // no conversion from row vector to column vector necessary
}


/** Destructs the coordinate.
  *
  * \tparam T scalar type
  */

template <class T>
Coordinate<T>::~Coordinate()
{
}


/** \tparam T scalar type */

template <class T>
inline T & Coordinate<T>::x()
{
    assert(size() >= 1);

    return m_data(0);
}


/** \tparam T scalar type */

template <class T>
inline T & Coordinate<T>::y()
{
    assert(size() >= 2);

    return m_data(1);
}


/** \tparam T scalar type */

template <class T>
inline T & Coordinate<T>::z()
{
    assert(size() >= 3);

    return m_data(2);
}


/** \tparam T scalar type */

template <class T>
inline T & Coordinate<T>::w()
{
    assert(size() >= 4);

    return m_data(3);
}


/** \tparam T scalar type */

template <class T>
inline const T & Coordinate<T>::x() const
{
    assert(size() >= 1);

    return m_data(0);
}


/** \tparam T scalar type */

template <class T>
inline const T & Coordinate<T>::y() const
{
    assert(size() >= 2);

    return m_data(1);
}


/** \tparam T scalar type */

template <class T>
inline const T & Coordinate<T>::z() const
{
   assert(size() >= 3);

   return m_data(2);
}


/** \tparam T scalar type */

template <class T>
inline const T & Coordinate<T>::w() const
{
    assert(size() >= 4);

    return m_data(3);
}


/** \tparam T scalar type
  *
  * \param [in] index position of the element to be returned
  *
  * \return scalar
  *
  * Returns the element at index position \c index. The first element has
  * index 0.
  *
  * Example:
  *
  * \code
  * Coordinate<double> c(1, 2, 3);
  * double a = c[1]; // a == 2
  * \endcode
  */

template <class T>
inline T & Coordinate<T>::operator[](int index)
{
    assert(0 <= index && index < m_data.size());

    return m_data(index);
}


/** \tparam T scalar type
  *
  * \param [in] index position of the element to be returned
  *
  * \return scalar
  *
  * Returns the element at index position \c index. The first element has
  * index 0.
  *
  * Example:
  *
  * \code
  * Coordinate<double> c(1, 2, 3);
  * double a = c[1]; // a == 2
  * \endcode
  */

template <class T>
inline const T & Coordinate<T>::operator[](int index) const
{
    assert(0 <= index && index < m_data.size());

    return m_data(index);
}


/** \tparam T scalar type
  *
  * \param [in] coordinate coordinate to be compared with \c *this
  *
  * \return true if all elements of \c *this and \c coordinate are equal to
  *         each other
  */

template <class T>
inline bool Coordinate<T>::operator==(const Coordinate & coordinate) const
{
    assert(this->size() == coordinate.size());

    return (m_data == coordinate.m_data).all();
}


/** \tparam T scalar type
  *
  * \param [in] value scalar to be added
  *
  * \return *this
  *
  * Adds \c value to each component of \c *this.
  */

template <class T>
inline Coordinate<T> & Coordinate<T>::operator+=(T value)
{
    m_data += value;
    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] value scalar to be subtracted
  *
  * \return *this
  *
  * Subtracts \c value from each component of \c *this.
  */

template <class T>
inline Coordinate<T> & Coordinate<T>::operator-=(T value)
{
    m_data -= value;
    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] value scalar to be multiplied with
  *
  * \return *this
  *
  * Multiplies each component of \c *this with \c value.
  */

template <class T>
inline Coordinate<T> & Coordinate<T>::operator*=(T value)
{
    m_data *= value;
    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] value divisor
  *
  * \return *this
  *
  * \throw ErrorObj If \c value is zero, an error object is thrown and its error
  *        code set to \c DIVISION_BY_ZERO.
  *
  * Divides each component of \c *this by \c value.
  *
  * \see Enumerator Coordinate::Error
  */

template <class T>
Coordinate<T> & Coordinate<T>::operator/=(T value)
{
    if (value == 0)
    {
        ErrorObj error;
        error.setClassName("Coordinate");
        error.setFunctionName("operator/=");
        error.setErrorMessage("Division by zero.");
        error.setErrorCode(DIVISION_BY_ZERO);
        throw error;
    }

    m_data /= value;
    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] coordinate \c other
  *
  * \return *this
  *
  * Component-wise addition of \c other and \c *this.
  */

template <class T>
inline Coordinate<T> & Coordinate<T>::operator+=(const Coordinate & coordinate)
{
    assert(this->size() == coordinate.size());

    m_data += coordinate.m_data;
    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] coordinate \c other
  *
  * \return *this
  *
  * Component-wise subtraction of \c other and \c *this.
  */

template <class T>
inline Coordinate<T> & Coordinate<T>::operator-=(const Coordinate & coordinate)
{
    assert(this->size() == coordinate.size());

    m_data -= coordinate.m_data;
    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] coordinate \c other
  *
  * \return *this
  *
  * Component-wise multiplication of \c other and \c *this.
  */

template <class T>
inline Coordinate<T> & Coordinate<T>::operator*=(const Coordinate & coordinate)
{
    assert(this->size() == coordinate.size());

    m_data *= coordinate.m_data;
    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] coordinate coordinate containing the divisors
  *
  * \return *this
  *
  * \throw ErrorObj If one or more components of \c coordinate are zero, an
  *        error object is thrown and its error code set to \c DIVISION_BY_ZERO.
  *
  * Component-wise division of \c *this by \c other.
  *
  * \see Enumerator Coordinate::Error
  */

template <class T>
Coordinate<T> & Coordinate<T>::operator/=(const Coordinate & coordinate)
{
    assert(this->size() == coordinate.size());

    if (!coordinate.m_data.all())
    {
        ErrorObj error;
        error.setClassName("Coordinate");
        error.setFunctionName("operator/=");
        error.setErrorMessage("Division by zero.");
        error.setErrorCode(DIVISION_BY_ZERO);
        throw error;
    }

    m_data /= coordinate.m_data;

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] value scalar to be added
  *
  * \return Coordinate
  *
  * Adds \c value to each component of \c *this.
  */

template <class T>
Coordinate<T> Coordinate<T>::operator+(T value) const
{
    Coordinate<T> coordinate;
    coordinate.m_data = m_data + value;

    return coordinate;
}


/** \tparam T scalar type
  *
  * \param [in] value scalar to be subtracted
  *
  * \return Coordinate
  *
  * Subtracts \c value from each component of \c *this.
  */

template <class T>
Coordinate<T> Coordinate<T>::operator-(T value) const
{
    Coordinate<T> coordinate;
    coordinate.m_data = m_data - value;

    return coordinate;
}


/** \tparam T scalar type
  *
  * \param [in] value scalar to be multiplied with
  *
  * \return Coordinate
  *
  * Multiplies \c value with each component of \c *this.
  */

template <class T>
Coordinate<T> Coordinate<T>::operator*(T value) const
{
    Coordinate<T> coordinate;
    coordinate.m_data = m_data * value;

    return coordinate;
}


/** \tparam T scalar type
  *
  * \param [in] value divisor
  *
  * \return Coordinate
  *
  * \throw ErrorObj If \c value is zero, an error object is thrown and its error
  *        code set to \c DIVISION_BY_ZERO.
  *
  * Divides each component of \c *this by \c value.
  *
  * \see Enumerator Coordinate::Error
  */

template <class T>
Coordinate<T> Coordinate<T>::operator/(T value) const
{
    if (value == 0)
    {
        ErrorObj error;
        error.setClassName("Coordinate");
        error.setFunctionName("operator/");
        error.setErrorMessage("Division by zero.");
        error.setErrorCode(DIVISION_BY_ZERO);
        throw error;
    }

    Coordinate<T> coordinate;
    coordinate.m_data = m_data / value;

    return coordinate;
}


/** \tparam T scalar type
  *
  * \param [in] coordinate
  *
  * \return Coordinate
  */

template <class T>
Coordinate<T> Coordinate<T>::operator+(const Coordinate & coordinate) const
{
    assert(this->size() == coordinate.size());

    Coordinate<T> tmp;
    tmp.m_data = m_data + coordinate.m_data;

    return tmp;
}


/** \tparam T scalar type
  *
  * \param [in] coordinate
  *
  * \return Coordinate
  */

template <class T>
Coordinate<T> Coordinate<T>::operator-(const Coordinate & coordinate) const
{
    assert(this->size() == coordinate.size());

    Coordinate<T> tmp;
    tmp.m_data = m_data - coordinate.m_data;

    return tmp;
}


/** \tparam T scalar type
  *
  * \param [in] coordinate
  *
  * \return Coordinate
  */

template <class T>
Coordinate<T> Coordinate<T>::operator*(const Coordinate & coordinate) const
{
    assert(this->size() == coordinate.size());

    Coordinate<T> tmp;
    tmp.m_data = m_data * coordinate.m_data;

    return tmp;
}


/** \tparam T scalar type
  *
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * \throw ErrorObj If one or more components of \c coordinate are zero, an
  *        error object is thrown and its error code set to \c DIVISION_BY_ZERO.
  *
  * \see Enumerator Coordinate::Error
  */

template <class T>
Coordinate<T> Coordinate<T>::operator/(const Coordinate & coordinate) const
{
    assert(this->size() == coordinate.size());

    if (!coordinate.m_data.all())
    {
        ErrorObj error;
        error.setClassName("Coordinate");
        error.setFunctionName("operator/");
        error.setErrorMessage("Division by zero.");
        error.setErrorCode(DIVISION_BY_ZERO);
        throw error;
    }

    Coordinate<T> tmp;
    tmp.m_data = m_data / coordinate.m_data;

    return tmp;
}


/** \tparam T scalar type
  *
  * \param [in] value component added to the coordinate
  *
  * \return enlarged copy of the coordinate
  *
  * With the comma operator it is possible to enlarge a coordinate by one or
  * more scalars. This can be usefull, if you need to convert an ordinary
  * coordinate into a homogeneous coordinate, as shown in the example below.
  *
  * Example:
  *
  * \code
  *
  * Coordinate<double> a(1, 2, 3);
  *
  * cout << a << endl
  *      << (a, 1) << endl
  *      << (a, 4, 4) << endl;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * (1, 2, 3)
  * (1, 2, 3, 1)
  * (1, 2, 3, 4, 4)
  *
  * \endcode
  *
  * \note You need to embrace the expression, otherwise the comma is not
  *       recognized as an operator!
  */

template <class T>
Coordinate<T> Coordinate<T>::operator,(T value) const
{
    const unsigned int size = m_data.size();
    data_type data(size + 1);
    data.head(size) = m_data;
    data(size) = value;

    Coordinate coordinate;
    coordinate.m_data = data;

    return coordinate;
}


/** \tparam T scalar type
  *
  * \return \c *this (converted)
  *
  * This operator performs an automatic conversion of \c *this to
  * Coordinate::data_type, i.e. an \c Eigen::Array
  *
  * \see http://eigen.tuxfamily.org
  */

template <class T>
Coordinate<T>::operator data_type()
{
    return m_data;
}


/** \tparam T scalar type
  *
  * \return \c *this (converted)
  *
  * This operator performs an automatic conversion of \c *this to
  * Coordinate::const_data_type, i.e. an \c Eigen::Array
  *
  * \see http://eigen.tuxfamily.org
  */

template <class T>
Coordinate<T>::operator const_data_type() const
{
    return m_data;
}


#ifdef TRTK_SUPPORT_CVECTOR

/** \tparam T scalar type
  *
  * \return \c *this (converted)
  *
  * This operator performs an automatic type conversion from Coordinate to
  * \c CVector.
  *
  * \note \macro{TRTK_SUPPORT_CVECTOR} must be defined to enable this
  *       function.
  */

template <class T>
Coordinate<T>::operator CVector() const
{
    const unsigned int size = m_data.size();

    CVector cVector(size);

    for (unsigned int i = 0; i < size; ++i)
    {
        cVector(i) = m_data(i);
    }

    return cVector;
}

#endif // TRTK_SUPPORT_CVECTOR


/** \tparam T scalar type
  *
  * \param [in] other coordinate
  *
  * \return Returns the cross product of \c *this and \c other.
  */

template <class T>
Coordinate<T> Coordinate<T>::cross(const Coordinate & other) const
{
    assert((this->size() == 3) && (other.size() == 3));

    const T x1 = m_data(0);
    const T x2 = m_data(1);
    const T x3 = m_data(2);

    const T y1 = other.m_data(0);
    const T y2 = other.m_data(1);
    const T y3 = other.m_data(2);

    return Coordinate<T>(x2 * y3 - x3 * y2,
                         x3 * y1 - x1 * y3,
                         x1 * y2 - x2 * y1);
}


/** \tparam T scalar type
  *
  * \param [in] other coordinate
  *
  * \return Returns the dot product of \c *this and \c other.
  */

template <class T>
double Coordinate<T>::dot(const Coordinate & other) const
{
    assert(this->size() == other.size());

    return m_data.matrix().dot(other.m_data.matrix());
}


/** \tparam T scalar type
  *
  * \param [in] value \n
  *
  * Sets each component of \c *this to \c value.
  *
  * \return Returns \c *this.
  */

template <class T>
inline Coordinate<T> & Coordinate<T>::fill(T value)
{
    m_data.setConstant(value);
    return *this;
}


/** \tparam T scalar type */

template <class T>
inline double Coordinate<T>::norm() const
{
    return m_data.matrix().norm();
}


/** \tparam T scalar type
  *
  * If this function is called, \c *this is divided by its Euclidean norm.
  *
  * \return Returns \c *this.
  */

template <class T>
Coordinate<T> & Coordinate<T>::normalize()
{
    m_data.matrix().normalize();
    return *this;
}


/** \tparam T scalar type */

template <class T>
Coordinate<T> Coordinate<T>::normalized() const
{
    return Coordinate(m_data.matrix().normalized());
}


/** \tparam T scalar type
  *
  * The returned vector is \b not normalized!
  */

template <class T>
Coordinate<T> Coordinate<T>::orthogonal() const
{
    assert(this->size() >= 2);
    assert(!Tools::isZero(this->norm()));


    // Find largest and second largest element of this coordinate
    // and their according positions.

    T max[2] = {0};
    int pos[2] = {0};

    for (unsigned int i = 0; i < this->size(); ++i)
    {
        using std::abs;

        if (abs(m_data(i)) > abs(max[0]))
        {
            max[1] = max[0];
            max[0] = m_data(i);
            pos[1] = pos[0];
            pos[0] = i;
        }
        else if (abs(m_data(i)) >= abs(max[1]))
        {
            max[1] = m_data(i);
            pos[1] = i;
        }
    }

    // Construct the orthogonal vector by switching the positions of
    // the two largest elements and negating the second largest element.
    // All other components are set to zero.
    // The dot product of this constructed vector with *this always equals zero.

    Coordinate<T> result;
    result.resize(this->size(), 0);
    result[pos[0]] = - max[1];
    result[pos[1]] =   max[0];

    return result;
}


/** \tparam T scalar type
  *
  * The returned vector is normalized!
  */

template <class T>
Coordinate<T> Coordinate<T>::orthonormal() const
{
    assert(this->size() >= 2);
    assert(!Tools::isZero(this->norm()));

    // Find largest and second largest element of this coordinate
    // and their according positions.

    T max[2] = {0};
    int pos[2] = {0};

    for (unsigned int i = 0; i < this->size(); ++i)
    {
        using std::abs;

        if (abs(m_data(i)) > abs(max[0]))
        {
            max[1] = max[0];
            max[0] = m_data(i);
            pos[1] = pos[0];
            pos[0] = i;
        }
        else if (abs(m_data(i)) >= abs(max[1]))
        {
            max[1] = m_data(i);
            pos[1] = i;
        }
    }

    // Construct the orthonormal vector by switching the positions of
    // the two largest elements and negating the second largest element.
    // All other components are set to zero.

    using std::sqrt;
    T norm = sqrt(max[0] * max[0] + max[1] * max[1]);

    Coordinate<T> result;
    result.resize(this->size(), 0);
    result[pos[0]] = - max[1] / norm;
    result[pos[1]] =   max[0] / norm;

    return result;
}


/** \tparam T scalar type
  *
  * \param [in] size    coordinate size
  * \param [in] a       minimum value of the distribution
  * \param [in] b       maximum value of the distribution
  *
  * Properties of the uniform distribution \f$ \mathcal{U}(a, b) \f$:
  * - pdf: \f$ f(x) = \begin{cases}
  *                      \frac{1}{b - a}    & \text{for } a \leq x \leq b \\
  *                      0                  & \text{otherwise}
  *                   \end{cases} \f$
  * - support: \f$ x \in [a; b] \f$
  * - mean value: \f$ 0.5 (a + b) \f$
  * - variance: \f$ \frac{1}{12} (b - a)^2 \f$
  *
  * \return Returns a \c Coordinate<T>.
  *
  * \see size()
  */

template <class T>
Coordinate<T> Coordinate<T>::rand(unsigned int size, T a, T b)
{
    Coordinate<T> coordinate;
    coordinate.reserve(size);

    for (unsigned i = 0; i < size; ++i)
    {
        coordinate[i] = Tools::rand(a, b);
    }

    return coordinate;
}


/** \tparam T scalar type
  *
  * \param [in] size    coordinate size
  * \param [in] mu      mean of the distribution
  * \param [in] sigma   standard deviation of the distribution
  *
  * Properties of the normal distribution \f$ \mathcal{N}(\mu, \sigma^2) \f$:
  * - pdf: \f$ f(x) = \frac{1}{\sqrt{2\pi\sigma^2}} e^{-\frac{(x-\mu)^2}{2\sigma^2}} \f$
  * - support: \f$ x \in R \f$
  * - mean value: \f$ \mu \f$
  * - variance: \f$ \sigma^2 \f$
  *
  * \return Returns a \c Coordinate<T>.
  *
  * \see size()
  */

template <class T>
Coordinate<T> Coordinate<T>::randn(unsigned int size, T mu, T sigma)
{
    Coordinate<T> coordinate;
    coordinate.reserve(size);

    for (unsigned i = 0; i < size; ++i)
    {
        coordinate[i] = Tools::randn(mu, sigma);
    }

    return coordinate;
}


/** \tparam T scalar type
  *
  * \param [in] size new coordinate size
  *
  * \note This call invalidates the data currently hold!
  *
  * \return Returns \c *this.
  *
  * \see size()
  */

template <class T>
Coordinate<T> & Coordinate<T>::reserve(unsigned int size)
{
    m_data.resize(size);
    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] size new coordinate size
  * \param [in] value values of appended elements in case \c *this is enlarged
  *
  * Changes the coordinate size to \p size.
  *
  * \return Returns \c *this.
  *
  * \see size()
  */

template <class T>
Coordinate<T> & Coordinate<T>::resize(unsigned int size, T value)
{
    const int former_size = signed(this->size());
    const int difference_in_size = signed(size) - signed(former_size);

    // resizing allocates new memory and thus all data are lost
    // --> save data to tempory and restore it afterwards

    data_type data = m_data;

    m_data.resize(size);

    if (difference_in_size <= 0)
    {
        m_data = data.head(size);
    }
    else
    {
        m_data.head(former_size) = data;
        m_data.tail(difference_in_size).setConstant(value);
    }

    return *this;
}


/** \tparam T scalar type
  *
  * \return size of \c *this
  *
  * \see resize(const int size, const T value)
  */

template <class T>
inline unsigned int Coordinate<T>::size() const
{
    return m_data.size();
}


/** \tparam T scalar type */

template <class T>
inline double Coordinate<T>::squaredNorm() const
{
    return m_data.matrix().squaredNorm();
}


/** \tparam T scalar type
  *
  * \return returns an \e Eigen \e 3 column vector
  *
  * \see Eigen (http://eigen.tuxfamily.org)
  */

template <class T>
inline typename Coordinate<T>::data_type & Coordinate<T>::toArray()
{
    return m_data;
}


/** \tparam T scalar type
  *
  * \return returns an \e Eigen \e 3 column vector
  *
  * \see Eigen (http://eigen.tuxfamily.org)
  */

template <class T>
inline const typename Coordinate<T>::data_type & Coordinate<T>::toArray() const
{
    return m_data;
}


/** \tparam T scalar type
  *
  * \return returns an \e Eigen \e 3 column vector
  *
  * \note
  * This functions copies the actual coordinate. Alternatively, you can use
  * \code
  * coordinate.toArray().matrix()
  * \endcode
  * which is more efficient (since no copying is done).
  *
  * \see Eigen (http://eigen.tuxfamily.org)
  */

template <class T>
inline typename Coordinate<T>::matrix_type Coordinate<T>::toMatrix() const
{
    return m_data;
}


/////////////////////////////////////////////////////////////////////////////////////


// Operators of the form T op Coordinate<class T>

/** \relates Coordinate
  *
  * \tparam T scalar type of coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate with scalar type T
  *
  * Adds \c value to each component of \c coordinate.
  */

template <class T>
Coordinate<T> operator+(T value, const Coordinate<T> & coordinate)
{
    return coordinate + value;
}


/** \relates Coordinate
  *
  * \tparam T scalar type of coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate with scalar type T
  *
  * Element-wise subtraction where \c coordinate is subtracted from
  * \f$ (value, value, ..., value)^T \f$.
  */

template <class T>
Coordinate<T> operator-(T value, const Coordinate<T> & coordinate)
{
    return coordinate * (-1) + value;
}


/** \relates Coordinate
  *
  * \tparam T scalar type of coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate with scalar type T
  *
  * Multiplies each component of \c coordinate with \c value.
  */

template <class T>
Coordinate<T> operator*(T value, const Coordinate<T> & coordinate)
{
    return coordinate * value;
}


/** \relates Coordinate
  *
  * \tparam T scalar type of coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate with scalar type T
  *
  * Element-wise division of \f$ (value, value, \dots, value)^T \f$ by
  * \c coordinate.
*/

template <class T>
Coordinate<T> operator/(T value, const Coordinate<T> & coordinate)
{
    Coordinate<T> coordinate2;
    coordinate2.resize(coordinate.size(), value);

    return coordinate2.operator/(coordinate);
}


// Stream operators

/** \relates Coordinate
  *
  * \tparam T scalar type
  *
  * \param [in]  input input stream (e.g. \c std::cin)
  * \param [out] coordinate target coordinate
  *
  * \return Returns a reference to the input stream to allow constructs like
  *         \code
  * Coordinate<double> c1(0, 0), c2(0, 0);
  * std::cin >> c1 >> c2;
  *         \endcode
  *
  * Reads a Coordinate from \c input and saves it to \c coordinate. The input
  * format is a comma separated list enclosed by round brackets. Anything
  * diverging from it, leads to an exception. The dimension of the input
  * coordinate must coincide with the dimension of the target coordinate,
  * otherwise an exception is thrown.
  *
  * Here is an more elaborate example:
  *
  * \code
  *
  * #include <Coordinate.hpp>
  * #include <iostream>
  * #include <sstream>
  *
  * int main()
  * {
  *     Coordinate<double> c(1, 2, 3);
  *
  *     std::cout << c << std::endl;
  *
  *     std::stringstream ss;
  *     ss << "(4, 2, 1)";
  *     ss >> c;
  *
  *     std::cout << c << std::endl;
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
  * (1, 2, 3)
  * (4, 2, 1)
  *
  * \endcode
  *
  * \throw std::exception (see text above)
  *
  * \see operator >> (std::istream & input, Coordinate<T> & coordinate)
  */

template <class T>
std::istream & operator >> (std::istream & input, Coordinate<T> & coordinate)
{
    // The input format is a comma separated list enclosed by round brackets.
    // Anything diverging from it, leads to an exception.
    //
    // Example: (1, 2, 3)

    char input_character;
    const std::string error_msg("Wrong input format or coordinate size.");

    // throw an exception if the input character differs from what was expected

    std::ios_base::iostate old_iostate = input.exceptions();

    input.exceptions(std::ios_base::badbit | std::ios_base::failbit);

    // read in coordinate

    input >> input_character;
    if (input_character != '(') throw std::invalid_argument(error_msg);

    int i = 0;

    while (i < signed(coordinate.size()) - 1)
    {
        input >> coordinate[i++];

        input >> input_character;
        if (input_character != ',') throw std::invalid_argument(error_msg);
    }

    if (coordinate.size() != 0) // allow empty coordinates like '( )'
    {
        input >> coordinate[i];
    }

    input >> input_character;
    if (input_character != ')') throw std::invalid_argument(error_msg);

    // restore old io_state

    input.exceptions(old_iostate);

    return input;
}


/** \relates Coordinate
  *
  * \tparam T scalar type
  *
  * \param [out] ouput output stream (e.g. \c std::out)
  * \param [in]  coordinate coordinate
  *
  * \return Returns a reference to the output stream to allow constructs like
  *         \code
  * Coordinate<double> c(1, 2);
  * std::cout << c << std::endl;
  *         \endcode
  *
  * Writes a Coordinate as formatted text to an output stream. The output
  * format is a comma separated list enclosed by round brackets.
  *
  * Example:
  *
  * \code
  *
  * Coordinate<double> c(1, 2, 3);
  * std::cout << c << std::endl;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * (1, 2, 3)
  *
  * \endcode
  *
  * \see operator >> (std::istream & input, Coordinate<T> & coordinate)
  */

template <class T>
std::ostream & operator << (std::ostream & output, const Coordinate<T> & coordinate)
{
    // save stream state

    std::ios_base::fmtflags saved_flags = output.flags();
    // std::streamsize saved_precision = output.precision();

    // throw an exception if the output operation fails for any reason

    output.exceptions(std::ios_base::badbit | std::ios_base::failbit | std::ios_base::eofbit);

    // put out formatted coordinate

    output.clear();

    output // << std::fixed // or std::scientific
           // << std::setprecision(4)
           // << std::showpos
           // << std::setfill('0')
           // << std::internal
           << "(";

    int i = 0;

    while (i < signed(coordinate.size()) - 1)
    {
        output // << std::setw(10)
               << coordinate[i++]
               << ", ";
    }

    if (coordinate.size() != 0) // allow empty coordinates like '( )'
    {
        output // << std::setw(10)
               << coordinate[i];
    }

    output <<  ")";

    // restore old stream state

    // output.precision(saved_precision);
    output.flags(saved_flags);

    return output;
}


/** \brief Computes the square root of each element.
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2013-08-19
  */

template <class T>
TRTK::Coordinate<T> sqrt(const TRTK::Coordinate<T> & coordinate)
{
    TRTK::Coordinate<T> result;
    result.resize(coordinate.size());

    for (unsigned i = 0; i < coordinate.size(); ++i)
    {
        using std::sqrt;
        result[i] = ::sqrt(coordinate[i]);
    }

    return result;
}


} // end of namespace TRTK

#endif // COORDINATE_HPP_1023074891
