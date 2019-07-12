/*
    Copyright (C) 2010 - 2019 Christoph Hänisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.4.0 (2019-07-02)
*/

/** \file Tools.hpp
  * \brief This file contains various helper functions.
  */

#ifndef TOOLS_H_9034987422
#define TOOLS_H_9034987422


#include <cassert>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <list>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "Coordinate.hpp"

#ifdef QT_VERSION
    #include <QTransform>
#endif // QT_VERSION


namespace TRTK
{


template <class T> class Coordinate;


namespace Tools
{


/** \namespace TRTK::Tools
  * \brief This namespace contains various helper functions.
  *
  * Helper functions sorted by topic:
  *
  * - Classes and Inheritance
  *   - \ref isClass()
  *   - \ref isDerivedFrom()
  * - Containers
  *   - \ref listToVector()
  *   - \ref mean()
  *   - \rev standardDeviation()
  *   - \ref variance()
  *   - \ref vectorToList()
  *   - \ref zip()
  * - Date and Time
  *   - \ref getCurrentDate()
  *   - \ref getCurrentTime()
  * - Files
  *   - \ref fileExists()
  *   - \ref fileLength()
  * - Numbers (Comparisons, Conversions, Random Numbers, etc.)
  *   - \ref isEqual()
  *   - \ref isZero()
  *   - \ref rand()
  *   - \ref rand(T, T) "rand(a, b)"
  *   - \ref randn()
  *   - \ref randn(T, T) "randn(mu, sigma)"
  *   - \ref sign()
  *   - \ref toString()
  * - Rotation and Coordinate Systems
  *   - \ref cartesian2Spherical()
  *   - \ref axisAngleFromRotationMatrix()
  *   - \ref Eigen3x3_to_QTransfom
  *   - \ref orthogonalMatrixToQuaternion()
  *   - \ref quaternionToOrthogonalMatrix()
  *   - \ref rotationMatrix()
  *   - \ref spherical2Cartesian()
  */


#ifdef EIGEN_MACROS_H
#ifdef QT_VERSION

/** \brief Converts an Eigen 3x3 matrix into a QTransform.
  *
  * \note This function is only available, if Eigen and Qt were included before.
  *
  * \author Christoph Hänisch
  * \version 0.1.1
  * \date last changed on 2011-10-10
  */

template <class EigenMatrix>
QTransform Eigen3x3_to_QTransfom(const EigenMatrix & eigenMatrix)
{
    QTransform qTransform(eigenMatrix(0, 0), eigenMatrix(0, 1), eigenMatrix(0, 2),
                          eigenMatrix(1, 0), eigenMatrix(1, 1), eigenMatrix(1, 2),
                          eigenMatrix(2, 0), eigenMatrix(2, 1), eigenMatrix(2, 2));

    return qTransform;
}

#endif // QT_VERSION
#endif // EIGEN_MACROS_H


 /**
  * \tparam T scalar (floating point) type
  *
  * This function decomposes a given 3-by-3 rotation matrix into the normalized rotation
  * axis and the positive rotation angle where the rotation angle \f$ \alpha \in [0, \pi] \f$.
  * The decomposition is unique. The rotation matrix does not need to describe a pure rotation;
  * an additional scaling may be implied but is neglected. The matrix is assumed to be left
  * multiplied to column vectors.
  *
  * Example:
  *
  * \code
  *
  * Matrix3d R = rotationMatrix(Vector3d(1, 1, 1), 0.3);
  * Vector3d axis;
  * double angle;
  * tie(axis, angle) = axisAngleFromRotationMatrix(R);
  *
  * \endcode
  *
  * Then, \p axis = (0.5774, 0.5774, 0.5774) and \p angle = 0.3.
  *
  * @returns (axis, angle) where axis is a 3d vector
  *
  * \see rotationMatrix
  *
  * \author Christoph Hänisch
  * \version 0.2.0
  * \date last changed on 2019-07-01
  */

template <class T>
std::pair<Eigen::Matrix<T, 3, 1>, T> axisAngleFromRotationMatrix(const Eigen::Matrix<T, 3, 3> & matrix)
{
    // The implementation is based on the lecture notes of Carlo Tomasi. See [1] for more information.
    // [1] https://www2.cs.duke.edu/courses/fall13/compsci527/notes/rodrigues.pdf

    using namespace Eigen;
    using Matrix3T = Matrix<T, 3, 3>;
    using Vector3T = Matrix<T, 3, 1>;

    const T eps = 1000 * std::numeric_limits<T>::epsilon();
    const T pi = 3.14159265359;

    Matrix3T A = (matrix - matrix.transpose()) / 2;
    Vector3T rho = Vector3T(A(2, 1), A(0, 2), A(1, 0));
    T s = rho.norm();
    T c = (matrix.trace() - 1) / 2;

    Vector3T axis;
    T angle;

    if (abs(s - 0) < eps && abs(c - 1) < eps)
    {
        axis = Vector3T(1, 0, 0);
        angle = 0;
    }
    else if (abs(s - 0) < eps && abs(c + 1) < eps)
    {
        angle = pi;
        Matrix3T B = matrix + Matrix3T::Identity();
        for (int i = 0; i < 3; ++i)
        {
            axis = B.col(i).normalized();
            if (axis.norm() > 0) break;
        }
        // Assure uniqueness by forcing the axis onto the half-hemisphere.
        if (abs(axis.x()) < eps && abs(axis.y()) < eps && axis.z() < 0 ||
            abs(axis.x()) < eps && axis.y() < 0 ||
            axis.x() < 0)
        {
            angle = -angle;
            axis = -axis;
        }
    }
    else
    {
        using ::atan2;
        axis = rho / s;
        angle = atan2(s, c);
    }

    return std::make_pair(axis, angle);
}


/**
 * \tparam T scalar (floating point) type
 *
 * The Cartesian coordinate is converted to spherical coordinates using the
 * ISO 80000-2 convention (i.e. as in physics: radius r, inclination theta,
 * azimuth phi).
 *
 * @returns (r, theta, phi)
 *
 * \see spherical2Cartesian
 *
 * \author Christoph Hänisch
 * \version 0.1.0
 * \date last changed on 2019-06-17
 */

template <class T>
std::tuple<T, T, T> cartesian2Spherical(const T & x, const T & y, const T & z)
{
    const T pi = 3.1415926535897932384626433;
    using ::sqrt;
    auto r = sqrt(x * x + y * y + z * z);
    T theta;
    if (r == 0)
        theta = 0;
    else
        theta = acos(z / r);
    auto phi = atan2(y, x);

    assert(0 <= r);
    assert(0 <= theta && theta <= pi);
    assert(-pi <= phi && phi <= pi);

    return std::make_tuple(r, theta, phi);
}


/**
 * \tparam T scalar (floating point) type
 *
 * The Cartesian coordinate is converted to spherical coordinates using the
 * ISO 80000-2 convention (i.e. as in physics: radius r, inclination theta,
 * azimuth phi).
 *
 * @returns 3d vector (r, theta, phi)
 *
 * \see spherical2Cartesian
 *
 * \author Christoph Hänisch
 * \version 0.1.0
 * \date last changed on 2019-06-17
 */

template <class T>
Eigen::Matrix<T, 3, 1> cartesian2Spherical(const Eigen::Matrix<T, 3, 1> & point)
{
    Eigen::Matrix<T, 3, 1> result;
    std::tie(result.x(), result.y(), result.z()) = cartesian2Spherical(point.x(), point.y(), point.z());
    return result;
}


/**
 * \tparam T scalar (floating point) type
 *
 * The Cartesian coordinate is converted to spherical coordinates using the
 * ISO 80000-2 convention (i.e. as in physics: radius r, inclination theta,
 * azimuth phi).
 *
 * \see spherical2Cartesian
 *
 * @returns 3d coordinate (r, theta, phi)
 *
 * \author Christoph Hänisch
 * \version 0.1.0
 * \date last changed on 2019-06-17
 */

template <class T>
Coordinate<T> cartesian2Spherical(const Coordinate<T> & point)
{
    Coordinate<T> result(0, 0, 0);
    std::tie(result.x(), result.y(), result.z()) = cartesian2Spherical(point.x(), point.y(), point.z());
    return result;
}


/** \brief Checks whether a certain file exists. */

bool fileExists(const char * file_name);


/** \brief Returns the size of a certain file. */

unsigned long long fileLength(const char * file_name);


/** \brief Returns the size of a certain file stream. */

unsigned long long fileLength(std::ifstream & file_stream);


/** \brief Returns the current date in ISO 8601 format (YYYY-MM-DD). */

std::string getCurrentDate();


/** \brief Returns the current time in format hh:mm:ss. */

std::string getCurrentTime();


/** \brief Checks whether an object is of a certain class.
  *
  * An object is also classified as being of type \p Derived, if \p Derived is
  * one of its base classes.
  *
  * Here are some examples also incorporating isClass(Base & base):
  *
  * \code
  * class A
  * {
  * public:
  *     virtual ~A() {};
  * };
  *
  * class B : public A
  * {
  * public:
  *     virtual ~B() {};
  * };
  *
  * class C
  * {
  * public:
  *     virtual ~C() {};
  * };
  *
  * int main()
  * {
  *     A a1;
  *     A * a2 = new A;
  *     A * a3 = new B;
  *     B b1;
  *     B * b2 = new B;
  *     C c1;
  *     C * c2 = new C;
  *
  *     cout << isClass<B>(a1) << endl;    // output: 0
  *     cout << isClass<B>(a2) << endl;    // output: 0
  *     cout << isClass<B>(a3) << endl;    // output: 1
  *     cout << isClass<B>(b1) << endl;    // output: 1
  *     cout << isClass<B>(b2) << endl;    // output: 1
  *     cout << isClass<B>(c1) << endl;    // output: 0
  *     cout << isClass<B>(c2) << endl;    // output: 0
  *
  *     return 0;
  * }
  * \endcode
  *
  * \note
  *   - The involved classes must be polymorphic, that is, they must at least
  *     contain one virtual function (e.g. a virtual destructor)
  *   - The compiler must support RTTI.
  *
  * \author Christoph Hänisch
  * \version 0.1.1
  * \date last changed on 2011-08-18
  */

template <class Derived, class Base>
inline bool isClass(Base * base_ptr)
{
    const Derived * derived_ptr = dynamic_cast<const Derived *>(base_ptr);
    return derived_ptr == NULL ? 0 : 1;
}


/** \brief Checks whether an object is of a certain class.
  *
  * An object is also classified as being of type \p Derived, if \p Derived is
  * one of its base classes.
  *
  * Please have a look at \ref isClass(Base * base_ptr) to see some example
  * code.
  *
  * \note
  *   - The involved classes must be polymorphic, that is, they must at least
  *     contain one virtual function (e.g. a virtual destructor)
  *   - The compiler must support RTTI.
  *
  * \author Christoph Hänisch
  * \version 0.1.1
  * \date last changed on 2011-08-18
  */

template <class Derived, class Base>
inline bool isClass(Base & base)
{
    return isClass<Derived>(&base);

    /*
    // This is an alternative implementation. However, the above one might be
    // better to be optimized by the compiler.

    // it might not be optimized

    try
    {
        Derived & derived = dynamic_cast<Derived &>(base);
        return 1;
    }
    catch (bad_cast)
    {
        return 0;
    }
    */
}


/** \brief Checks whether an object is derived from a certain class.
  *
  * Example:
  *
  * \code
  * class A
  * {
  * public:
  *     virtual ~A() {};
  * };
  *
  * class B : public A
  * {
  * public:
  *     virtual ~B() {};
  * };
  *
  * class C
  * {
  * public:
  *     virtual ~C() {};
  * };
  *
  * int main()
  * {
  *     A * a = new A;
  *     B * b = new B;
  *     C * c = new C;
  *
  *     cout << isDerivedFrom<A>(a) << endl;    // output: 1
  *     cout << isDerivedFrom<A>(b) << endl;    // output: 1
  *     cout << isDerivedFrom<A>(c) << endl;    // output: 0
  *
  *     return 0;
  * }
  * \endcode
  *
  * \author Christoph Hänisch
  * \version 0.1.1
  * \date last changed on 2011-08-18
  */

template <typename Base>
inline bool isDerivedFrom(const Base * base_ptr)
{
    return true;
}


/** \brief Checks whether an object is derived from a certain class.
  *
  * Please have a look at \ref isDerivedFrom(Base * base_ptr) to see some example
  * code.
  *
  * \author Christoph Hänisch
  * \version 0.1.1
  * \date last changed on 2011-08-18
  */

template <typename Base>
inline bool isDerivedFrom(const void * base_ptr)
{
    return false;
}


/** \tparam T scalar type
  *
  * \brief Tests for equality of two scalars.
  *
  * This function tests whether the absolute difference between the two input
  * arguments is below a certain threshold. The default threshold is
  * \f$ 10^{-14} \f$ or \c numeric_limits<T>::min() in case the numeric limit
  * of \c T is greater than the default value.
  *
  * Example:
  *
  * \code
  * double value = 1.0;
  *
  * isEqual(value, 1.0);        // twice the same argument type
  * isEqual<double>(value, 1);  // different argument types
  * \endcode
  *
  * \note The functions \c abs() as well as \c numeric_limits<T>::min() must be
  *       defined for the given scalar type \c T. (A template function
  *       specialization for integers is implemented.)
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-04-21
  */

template <class T>
inline bool isEqual(const T x, const T y)
{
    using std::abs;
    using std::numeric_limits;

    const T epsilon = numeric_limits<T>::min() > 10e-14 ? numeric_limits<T>::min() : 10e-14;
    return abs(x - y) < epsilon; // needs "cmath" and "limits" to be included
}

template <>
inline bool isEqual<int>(const int x, const int y)
{
    return x == y;
}



/** \tparam T scalar type
  *
  * \brief Tests for equality with zero.
  *
  * Actually, this function is only a short form of
  * <tt>isEqual<T>(value, T(0))</tt>, hence testing the absolute value
  * against a certain threshold. See \ref isEqual "isEqual(...)" for more
  * details.
  *
  * \note The template parameter can be omitted since it can be deduced from
  *       the argument type.
  *
  * \note The type \c T must provide a single parameter constructor allowing
  *       an initialization of the form T(0).
  *
  * Example:
  *
  * \code
  * double value = 1.0;
  * isZero(value); // returns false
  * \endcode
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-04-21
  */

template <class T>
inline bool isZero(const T value)
{
    return isEqual<T>(value, T(0));
}


/** \brief Converts an STL list into an STL vector.
  *
  * The vector's class type is the same as the one of the list.
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-04-21
  */

template <class T>
std::vector<T> listToVector(const std::list<T> & lst)
{
    int i = 0;
    std::vector<T> vec(lst.size());
    typename std::list<T>::const_iterator it;

    for (it = lst.begin(); it != lst.end(); ++it)
    {
        vec[i++] = *it;
    }

    return vec;
}


/** \brief Returns the mean of all container elements.
  *
  * \note The container must provide an STL interface.
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2013-07-30
  */

template <class Container, class ValueType> // TODO: C++11 --> class ValueType = Container::value_type
inline ValueType mean(const Container & container, ValueType null_value = ValueType())
{
	ValueType mean = null_value;

	for (typename Container::const_iterator it = container.begin(); it != container.end(); ++it)
	{
		mean = mean + *it;
	}

    return mean / container.size();
}


/** \brief Converts an orthogonal matrix to a unit quaternion.
  *
  * \note This function needs the following header to be included!
  *       \code #include <Eigen/Core> \endcode
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2013-08-12
  */

template <class T>
TRTK::Coordinate<T> orthogonalMatrixToQuaternion(const Eigen::Matrix<T, 3, 3> & matrix)
{
    // References:
    //
    // [1] http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
    // [2] http://en.wikipedia.org/wiki/Rotation_matrix

    // Note: Coordinate<T> is of the form (x, y, z, w) whereas quaternions are often notated (w, x, y, z)!

    using std::sqrt;

    TRTK::Coordinate<T> quaternion(0, 0, 0, 0);

    T trace = matrix.trace();

    if (trace > -1) // check because of the computation of the root
    {
        double s = 0.5 / sqrt(trace + 1);
        quaternion.x() = 0.25 / s;
        quaternion.y() = (matrix(2,1) - matrix(1,2)) * s;
        quaternion.z() = (matrix(0,2) - matrix(2,0)) * s;
        quaternion.w() = (matrix(1,0) - matrix(0,1)) * s;
    }
    else if (matrix(0,0) > matrix(1,1) && matrix(0,0) > matrix(2,2))
    {
        double s = 2.0 * sqrt(1 + matrix(0,0) - matrix(1,1) - matrix(2,2));
        quaternion.x() = (matrix(2,1) - matrix(1,2)) / s;
        quaternion.y() = 0.25 * s;
        quaternion.z() = (matrix(0,1) + matrix(1,0)) / s;
        quaternion.w() = (matrix(0,2) + matrix(2,0)) / s;
    }
    else if (matrix(1,1) > matrix(2,2))
    {
        double s = 2 * sqrt(1 + matrix(1,1) - matrix(0,0) - matrix(2,2));
        quaternion.x() = (matrix(0,2) - matrix(2,0)) / s;
        quaternion.y() = (matrix(0,1) + matrix(1,0)) / s;
        quaternion.z() = 0.25 * s;
        quaternion.w() = (matrix(1,2) + matrix(2,1)) / s;
    }
    else
    {
        double s = 2 * sqrt(1 + matrix(2,2) - matrix(0,0) - matrix(1,1));
        quaternion.x() = (matrix(1,0) - matrix(0,1)) / s;
        quaternion.y() = (matrix(0,2) + matrix(2,0)) / s;
        quaternion.z() = (matrix(1,2) + matrix(2,1)) / s;
        quaternion.w() = 0.25 * s;
    }

    return quaternion.normalize();
}


/** \brief Converts an orthogonal matrix to a unit quaternion.
  *
  * The given matrix can be nonorthogonal. Then this algorithm computes a
  * quaternion that yields the closest orthogonal matrix to the given matrix.
  *
  * \note This function needs the following headers to be included!
  * \code
  * #include <Eigen/Core>
  * #include <Eigen/Eigenvalues>
  * \endcode
  *
  * \references
  *
  * [1] Bar-Itzhack, "New Method for Extracting the Quaternion from a Rotation
  *     Matrix", J. Guidance, Vol. 23, No. 6: Engineering Notes, 2000
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2013-08-12
  */

template <class T>
TRTK::Coordinate<T> orthogonalMatrixToQuaternion2(const Eigen::Matrix<T, 3, 3> & matrix)
{
    // See [1] for more details about the implementation.

    // The matrix K3 is selfadjoint because it is real and symmetric. Thus we can use an
    // appropriate solver. The solver employed below only uses the lower triangular part
    // of the matrix hence we can leave the upper part uninitialized.

    typedef Eigen::Matrix<T, 4, 4> Matrix4T;
    typedef Eigen::Matrix<T, 4, 1> Vector4T;

    Matrix4T K3;

    K3(0,0) = matrix(0,0) - matrix(1,1) - matrix(2,2);
    K3(1,0) = matrix(1,0) + matrix(0,1);
    K3(2,0) = matrix(2,0) + matrix(0,2);
    K3(3,0) = matrix(1,2) - matrix(2,1);
    K3(1,1) = matrix(1,1) - matrix(0,0) - matrix(2,2);
    K3(2,1) = matrix(2,1) + matrix(1,2);
    K3(3,1) = matrix(2,0) - matrix(0,2);
    K3(2,2) = matrix(2,2) - matrix(0,0) - matrix(1,1);
    K3(3,2) = matrix(0,1) - matrix(1,0);
    K3(3,3) = matrix(0,0) + matrix(1,1) + matrix(2,2);

    K3 /= 3;

    Eigen::SelfAdjointEigenSolver<Matrix4T> solver(K3);

    Vector4T quaternion = solver.eigenvectors().col(3).normalized();

    return Coordinate<T>(quaternion);
}


/** \brief Converts a unit quaternion to an orthogonal matrix.
  *
  * \note This function needs the following header to be included!
  *       \code #include <Eigen/Core> \endcode
  *
  * \author Christoph Hänisch
  * \version 0.1.1
  * \date last changed on 2016-04-29
  */

template <class T>
Eigen::Matrix<T, 3, 3> quaternionToOrthogonalMatrix(T q0, T q1, T q2, T q3)
{
    // Revised code.

    // Identical to what you can find in the Wikipedia article
    // https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    // This matrix is a left-handed (post-multiplied) rotation matrix.

    assert(abs(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3 - 1) < 1e-5); // check for unit quaternion

    Eigen::Matrix<T, 3, 3> R;

    R(0, 0) = 1 - 2 * (q2 * q2 + q3 * q3);
    R(0, 1) = 2 * q1 * q2 - 2 * q0 * q3;
    R(0, 2) = 2 * q1 * q3 + 2 * q0 * q2;

    R(1, 0) = 2 * q1 * q2 + 2 * q0 * q3;
    R(1, 1) = 1 - 2 * (q1 * q1  + q3 * q3);
    R(1, 2) = 2 * q2 * q3 - 2 * q0 * q1;

    R(2, 0) = 2 * q1 * q3 - 2 * q0 * q2;
    R(2, 1) = 2 * q2 * q3 + 2 * q0 * q1;
    R(2, 2) = 1 - 2 * (q1 * q1 + q2 * q2);

    return R;
}


/** \brief Converts a unit quaternion to an orthogonal matrix.
  *
  * \note This function needs the following header to be included!
  *       \code #include <Eigen/Core> \endcode
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2013-08-06
  */

template <class T>
inline Eigen::Matrix<T, 3, 3> quaternionToOrthogonalMatrix(const TRTK::Coordinate<T> & quaternion)
{
    assert(quaternion.size() == 4);
    return quaternionToOrthogonalMatrix<T>(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
}


/** \tparam T floating point type
  *
  * \brief Random sample of the standard uniform distribution.
  *
  * Properties of the standard uniform distribution \f$ \mathcal{U}(0, 1) \f$:
  * - pdf: \f$ f(x) = \begin{cases}
  *                      1 & \text{for } 0 \leq x \leq 1 \\
  *                      0 & \text{otherwise}
  *                   \end{cases} \f$
  * - support: \f$ x \in [0; 1] \f$
  * - mean value: \f$ 0.5 \f$
  * - variance: \f$ \frac{1}{12} \f$
  *
  * \returns Returns a pseudo-random sample of the standard uniform distribution,
  *          i.e. a number in the range \f$ [0; 1] \f$.
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-11-07
  */

template <class T>
inline T rand()
{
    return static_cast<T>(std::rand()) / static_cast<T>(RAND_MAX);
}


/** \tparam T floating point type
  *
  * \param [in] a   minimum value of the distribution
  * \param [in] b   maximum value of the distribution
  *
  * \brief Random sample of a uniform distribution.
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
  * \returns Returns a pseudo-random number in the range \f$ [a; b] \f$.
  *
  * \author Christoph Hänisch
  * \version 0.1.2
  * \date last changed on 2016-07-13
  */

template <class T>
T rand(T a, T b)
{
    assert(a <= b);
    return static_cast<T>(std::rand()) / static_cast<T>(RAND_MAX) * (b - a) + a;
}

template <>
inline int rand<int>(int a, int b)
{
    assert(a <= b);
    return int(rand<double>(a, b));
}

template <>
inline unsigned rand<unsigned>(unsigned a, unsigned b)
{
    assert(0 <= a && a <= b);
    return unsigned(rand<double>(a, b));
}


/** \tparam T floating point type
  *
  * \brief Random sample of the standard normal distribution.
  *
  * Properties of the standard normal distribution \f$ \mathcal{N}(0, 1) \f$:
  * - pdf: \f$ f(x) = \frac{1}{\sqrt{2\pi}} e^{-\frac{1}{2}x^2} \f$
  * - support: \f$ x \in R \f$
  * - mean value: \f$ 0 \f$
  * - variance: \f$ 1 \f$
  *
  * \returns Returns a pseudo-random number.
  *
  * \note For reasons of convenience, there is also a non-templated \c randn()
  *       function which uses the \c double type.
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-11-08
  */

template <class T>
T randn()
{
    /*
    This algorithm uses the Marsaglia polar method. For more details, please
    have a look at "A convenient method for generating normal variables" by
    G. Marsaglia and T. A. Bray (SIAM Rev. 6, 260-264, 1964).
    */

    // The polar method yields two random variables per computation. So one
    // random variable is cached each time and returned on the next call.

    using std::log;
    using std::sqrt;

    static bool cache_is_valid = false;
    static T cached_random_variable = 0;

    if (!cache_is_valid)
    {
        T v1, v2, s;

        do
        {
            v1 = rand<T>(-1, 1);
            v2 = rand<T>(-1, 1);

            s = v1 * v1 + v2 * v2;
        }
        while (s >= 1 || s == 0);

        const T t = sqrt(-2 * log(s) / s);

        T random_variable = v1 * t;

        cached_random_variable = v2 * t;
        cache_is_valid = true;

        return random_variable;
    }
    else
    {
        cache_is_valid = false;

        return cached_random_variable;
    }
}

double randn(); // Convenience function for randn<double>().


/** \tparam T floating point type
  *
  * \param [in] mu      mean of the distribution
  * \param [in] sigma   standard deviation of the distribution
  *
  * \brief Random sample of the normal distribution.
  *
  * Properties of the normal distribution \f$ \mathcal{N}(\mu, \sigma^2) \f$:
  * - pdf: \f$ f(x) = \frac{1}{\sqrt{2\pi\sigma^2}} e^{-\frac{(x-\mu)^2}{2\sigma^2}} \f$
  * - support: \f$ x \in R \f$
  * - mean value: \f$ \mu \f$
  * - variance: \f$ \sigma^2 \f$
  *
  * \returns Returns a pseudo-random number.
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-11-07
  */

template <class T>
inline T randn(T mu, T sigma)
{
    return sigma * randn<T>() + mu;
}

template <>
inline int randn(int mu, int sigma)
{
    return int(sigma * randn<double>() + mu);
}


/** \brief Extracts the sign of a floating point number.
  *
  * It returns
  * \f[
  *     sgn(x) =
  *         \begin{cases}
  *         -1 & \text{if } x < 0 \\
  *          0 & \text{if } x = 0 \\
  *          1 & \text{if } x > 0
  *         \end{cases}
  * \f]
  *
  * \note
  * The type \c T must be constructable from \c 0 as well as provide some
  * comparison operators.
  */

template <class T> int sign(T value)
{
    return (T(0) < value) - (value < T(0));
}


/**
 * \tparam T scalar (floating point) type
 *
 * Constructs a 3d rotation matrix from a rotation vector and a rotation angle.
 *
 * Note, negating the rotation vector or the rotation angle yields the inverse of
 * the rotation. Consequently, if the rotation vector as well as the rotation angle
 * are negated the same rotation is obtained.
 *
 * @returns 3-by-3 matrix
 *
 * \see axisAngleFromRotationMatrix
 *
 * \author Christoph Hänisch
 * \version 0.1.0
 * \date last changed on 2019-06-24
 */

template <class T>
Eigen::Matrix<T, 3, 3> rotationMatrix(const Eigen::Matrix<T, 3, 1> & axis_, double angle)
{
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using Matrix3T = Eigen::Matrix<T, 3, 3>;

    T axis_magnitude = axis_.norm();
    assert(axis_magnitude > 0); // Axis may not be a null vector.
    Vector3T axis = axis_ / axis_magnitude;

    // Find an orthogonal vector.

    const T eps = std::numeric_limits<T>::epsilon();

    auto isNullvector = [&] (Vector3T & vec) -> bool
    {
        return vec.norm() < 10 * eps;
    };

    auto isCollinear = [&] (Vector3T a, Vector3T b) -> bool
    {
        a.normalize();
        b.normalize();
        return abs(abs(a.dot(b)) - 1) < 10 * eps;
    };

    auto vec = Vector3T::Random().normalized();
    while (isNullvector(vec) || isCollinear(vec, axis))
    {
        vec = Vector3T::Random().normalized();
    }

    auto vec_orthonormal = (vec - vec.dot(axis) * axis).normalized(); // Cf. Gram-Schmidt process
    assert(abs(axis.dot(vec_orthonormal)) < 1000 * eps); // Axis and vec_orthonormal must be orthonormal. Implementation error.

    // Compute the rotation matrix

    // Make a change of basis such that the rotation axis is aligned
    // with the x-axis, rotate around this axis and revert the change of
    // basis.

    Matrix3T M;
    M.row(0) = axis;
    M.row(1) = vec_orthonormal;
    M.row(2) = axis.cross(vec_orthonormal);

    Matrix3T R_x;
    R_x << 1,          0,           0,
           0, cos(angle), -sin(angle),
           0, sin(angle),  cos(angle);

    return M.transpose() * R_x * M;
}


/** \tparam T scalar type
  *
  * \brief Rounds towards the next (signed) integer value.
  *
  * \return Rounded number of the \b same \b type as the scalar type.
  *
  * Here is an example of how to use this function:
  *
  * \code
  * using TRTK::Tools::round;
  *
  * double a = round(-1.7);   // a == -2.0
  * double b = round(-1.3);   // b == -1.0
  * double c = round(1.3);    // c ==  1.0
  * double d = round(1.7);    // d ==  2.0
  * \endcode
  *
  * \note The functions \c floor() and \c ceil() must be defined for T.
  *
  * \author Christoph Hänisch
  * \version 0.1.1
  * \date last changed on 2011-07-01
  */

template <class T>
T round(const T number)
{
    using std::ceil;
    using std::floor;

    return (number < 0.0) ? ceil(number - 0.5) : floor(number + 0.5);
}

template <>
inline int round<int>(const int number)
{
    return number;
}


/**
 * \tparam T scalar (floating point) type
 *
 * \param [in] r
 * \param [in] theta
 * \param [in] phi
 *
 * The spherical coordinate is converted to a Cartesian coordinate using the
 * ISO 80000-2 convention (i.e., as in physics a coordinate is given as a
 * triple with radius r, inclination theta, and azimuth phi).
 *
 * @returns triple (x, y, z)
 *
 * \see cartesian2Spherical
 *
 * \author Christoph Hänisch
 * \version 0.1.0
 * \date last changed on 2019-06-25
 */

template <class T>
std::tuple<T, T, T> spherical2Cartesian(const T & r, const T & theta, const T & phi)
{
    using ::sin;
    using ::cos;
    double x = r * sin(theta) * cos(phi);
    double y = r * sin(theta) * sin(phi);
    double z = r * cos(theta);
    return std::make_tuple(x, y, z);
}


/**
 * \tparam T scalar (floating point) type
 *
 * \param [in] point    (r, theta, phi)
 *
 * The spherical coordinate is converted to a Cartesian coordinate using the
 * ISO 80000-2 convention (i.e., as in physics a coordinate is given as a
 * triple with radius r, inclination theta, and azimuth phi).
 *
 * @returns 3d coordinate (x, y, z)
 *
 * \see cartesian2Spherical
 *
 * \author Christoph Hänisch
 * \version 0.1.0
 * \date last changed on 2019-06-25
 */

template <class T>
Coordinate<T> spherical2Cartesian(const Coordinate<T> & point)
{
    Coordinate<T> result(0, 0, 0);
    std::tie(result.x(), result.y(), result.z()) = spherical2Cartesian(point.x(), point.y(), point.z());
    return result;
}


/**
 * \tparam T scalar (floating point) type
 *
 * \param [in] point    (r, theta, phi)
 *
 * The spherical coordinate is converted to a Cartesian coordinate using the
 * ISO 80000-2 convention (i.e., as in physics a coordinate is given as a
 * triple with radius r, inclination theta, and azimuth phi).
 *
 * @returns 3d vector (x, y, z)
 *
 * \see cartesian2Spherical
 *
 * \author Christoph Hänisch
 * \version 0.1.0
 * \date last changed on 2019-06-25
 */

template <class T>
Eigen::Matrix<T, 3, 1> spherical2Cartesian(const Eigen::Matrix<T, 3, 1> & point)
{
    Eigen::Matrix<T, 3, 1> result;
    std::tie(result.x(), result.y(), result.z()) = spherical2Cartesian(point.x(), point.y(), point.z());
    return result;
}


/** \brief Computes the corrected sample standard deviation of all container elements.
  *
  * While \f$ s^2 = \frac{1}{n - 1} \sum_{i = 1}^{n} (x_i - \bar x)^2 \f$ is an unbiased
  * estimator for the population variance, s is a biased estimator for the population
  * standard deviation [1].
  *
  * However, for normally and independently distributed random variables an unbiased
  * estimator is given by [2, 3, 4]
  * \f[
  * s = \frac{1}{c_4(n)} \sqrt{ \frac{1}{n - 1} \sum_{i = 1}^{n} (x_i - \bar x)^2 }
  * \f]
  * where
  * \f[
  * c_4(n) = \sqrt{\frac{2}{n - 1}} \frac{\Gamma(\frac{n}{2})}{\Gamma(\frac{n-1}{2})}
  *        = 1 - \frac{1}{4n} - \frac{7}{32n^2} - \frac{19}{128n^3} + O(n^{-4})
  * \f]
  *
  * This function implements the above described estimator.
  *
  * \references
  *
  *	[1] http://en.wikipedia.org/wiki/Sample_standard_deviation
  *
  *	[2] http://en.wikipedia.org/wiki/Unbiased_estimation_of_standard_deviation
  *
  * [3] Richard M. Brugger, "A Note on Unbiased Estimation of the Standard Deviation",
  *     The American Statistician (23) 4 p. 32 (1969)
  *
  * [4] J. Gurland and R. C. Tripathi, Richard M. Brugger, "A Simple Approximation for
  *     Unbiased Estimation of the Standard Deviation", The American Statistician (25)
  *     4 p. 30 (1971)
  *
  * \note The container must provide an STL interface.
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-04-21
  */

template <class Container, class ValueType> // TODO: C++11 --> class ValueType = Container::value_type
inline ValueType standardDeviation(const Container & container, ValueType null_value = ValueType())
{
	size_t n = container.size();
	assert (n >= 0);

	if (n == 0) return null_value;

	ValueType mean = Tools::mean(container, null_value);
	ValueType variance = null_value;

	for (typename Container::const_iterator it = container.begin(); it != container.end(); ++it)
	{
		variance = variance + (*it - mean) * (*it - mean);
	}

	double c4 = 1.0 - 1.0/(4.0*n) - 7.0/(32.0*n*n) - 19.0/(128.0*n*n*n);

	using std::sqrt;

    return sqrt(variance / (n - 1)) / c4;
}


/** \brief Converts the input argument (int, double etc.) into a std::string.
  *
  * Here is an example of how to use this function:
  *
  * \code
  * using std::string;
  * using TRTK::Tools::toString;
  *
  * double d = -1.7;
  * string str = "The value of d is " + toString(d);
  * \endcode
  *
  * \author Christoph Hänisch
  * \version 0.2.0
  * \date last changed on 2011-08-12
  */

template <class T>
inline std::string toString (const T & value)
{
    std::stringstream ss;
    ss << value;

    return ss.str();
}


/** \brief Computes the sample variance of all container elements.
  *
  * The variance is computed using Bessel's correction:
  * \f[
  * s^2 = \frac{1}{n - 1} \sum_{i = 1}^{n} (x_i - \bar x)^2
  * \f]
  *
  * \note The container must provide an STL interface.
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-04-21
  */

template <class Container, class ValueType> // TODO: C++11 --> class ValueType = Container::value_type
inline ValueType variance(const Container & container, ValueType null_value = ValueType())
{
	assert(container.size() >= 0);

	if (container.size() == 0) return null_value;

	ValueType mean = Tools::mean(container, null_value);
	ValueType variance = null_value;

	for (typename Container::const_iterator it = container.begin(); it != container.end(); ++it)
	{
		variance = variance + (*it - mean) * (*it - mean);
	}

    return variance / (container.size() - 1);
}


/** \brief Converts an STL vector into an STL list.
  *
  * The list's class type is the same as the one of the vector.
  *
  * \author Christoph Hänisch
  * \version 0.1.0
  * \date last changed on 2011-04-21
  */

template <class T>
std::list<T> vectorToList(const std::vector<T> & vec)
{
    typename std::list<T> lst;

    for (unsigned i = 0; i < vec.size(); ++i)
    {
        lst.push_back(vec[i]);
    }

    return lst;
}


/** \brief Zips two STL vectors into a single one.
*
* The elements are stored in pairs. Both vectors must have the same size.
*
* \author Christoph Hänisch
* \version 0.1.0
* \date last changed on 2016-07-13
*/

template <class T1, class T2>
std::vector<std::pair<T1, T2> > zip(const std::vector<T1> & v1, const std::vector<T2> & v2)
{
    size_t size_v1 = v1.size();
    size_t size_v2 = v2.size();
    assert(size_v1 == size_v2);

    std::vector<std::pair<T1, T2> > zipped;
    zipped.reserve(size_v1);

    for (size_t i = 0; i < size_v1; ++i)
    {
        zipped.push_back(std::make_pair(v1[i], v2[i]));
    }

    return zipped;
}


} // namspace Tools


} // namespace TRTK


#endif // TOOLS_H_9034987422
