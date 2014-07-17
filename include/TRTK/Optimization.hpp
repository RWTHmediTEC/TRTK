/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.0 (2012-06-10)
*/

/** \file Optimization.hpp
  * \brief This file contains various functions to find the roots (zeros) or the local
  *        minima and maxima of a function.
  */


#ifndef OPTIMIZATION_HPP_4562354833
#define OPTIMIZATION_HPP_4562354833


#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Coordinate.hpp"
#include "Definitions.hpp"


namespace TRTK
{


/** \namespace TRTK::Optimization
  *
  * \brief This namespace contains various classes and functions to find
  *        the roots (zeros) or the local minima and maxima of a function.
  *
  * <b>List of classes and functions:</b>
  *
  * - \ref Gradient
  * - \ref jacobian()
  * - \ref makeGradient()
  * - \ref solve()
  *   - \ref solveLevenbergMarquardt()
  *   - \ref solveNewtonRaphsonMethod()
  * - \ref solve1D()
  *
  * \macros
  *
  * If \ref TRTK_PARALLELIZE is defined some algorithms will make use of
  * OpenMP.
  *
  * </p><b>Example:</b>
  *
  * The following example shows how these functions can be used to estimate
  * an unknown transformation between two point sets:
  *
  * \code
  * #include <cassert>
  * #include <cmath>
  * #include <iomanip>
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Clock.hpp>
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Optimization.hpp>
  * #include <TRTK/Tools.hpp>
  * #include <TRTK/Transform2D.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Optimization;
  * using namespace TRTK::Tools;
  *
  *
  * class EstimateTransformation
  * {
  * public:
  *     EstimateTransformation(const vector<Coordinate<double> > & source_points,
  *                            const vector<Coordinate<double> > & target_points);
  *
  *     // Returns the error of the transformation which is defined by the given coefficients.
  *     double operator()(const Coordinate<double> & coefficients) const;
  *
  * private:
  *     const vector<Coordinate<double> > & source_points;
  *     const vector<Coordinate<double> > & target_points;
  * };
  *
  *
  * EstimateTransformation::EstimateTransformation(const vector<Coordinate<double> > & source_points,
  *                                                const vector<Coordinate<double> > & target_points) :
  *     source_points(source_points),
  *     target_points(target_points)
  * {
  * }
  *
  *
  * double EstimateTransformation::operator()(const Coordinate<double> & coefficients) const
  * {
  *     // Returns the Mean Square Error (MSE) of the currently estimated transformation.
  *
  *     Transform2D<double> transform(coefficients[0], coefficients[1], coefficients[2],
  *                                   coefficients[3], coefficients[4], coefficients[5],
  *                                   0,               0,               1);
  *
  *     double error = 0;
  *
  *     const unsigned N = source_points.size();
  *
  *     for (unsigned i = 0; i < N; ++i)
  *     {
  *         double residual = (transform * source_points[i] - target_points[i]).norm();
  *         error += residual * residual;
  *     }
  *
  *     return error / N;
  * }
  *
  *
  * int main()
  * {
  *     // Generate an abitrary transformation.
  *
  *     Transform2D<double> transform;
  *
  *     transform.rotate(0.927295218).shear(1, 0).scale(2, 2).translate(1, -2);
  *
  *     // Construct some source points and some target points which
  *     // are the transformed source points.
  *
  *     const unsigned N = 200;
  *
  *     vector<Coordinate<double> > source_points;
  *     vector<Coordinate<double> > target_points;
  *
  *     for (unsigned i = 0; i < N; ++i)
  *     {
  *         double x = rand(-100.0, 100.0);
  *         double y = rand(-100.0, 100.0);
  *
  *         Coordinate<double> source_point(x, y);
  *         Coordinate<double> target_point = transform * source_point;
  *
  *         source_points.push_back(source_point);
  *         target_points.push_back(target_point);
  *     }
  *
  *     // Estimate the transformation.
  *
  *     Clock clock;
  *
  *     EstimateTransformation estimateTransformation(source_points, target_points);
  *
  *     Coordinate<double> coefficients; // = {a11, a12, a13, a21, a22, a23} in Transform2D
  *     coefficients.resize(6, 1);
  *
  *     Options<double> options;
  *     options.error_tolerance = 1e-10;
  *
  *     Result<double> result = solve(makeGradient<double>(estimateTransformation),
  *                                   coefficients,
  *                                   options);
  *
  *     // Print the result.
  *
  *     cout << "Original transformation matrix:" << endl << endl;
  *
  *     cout << setw(6) << transform.a11() << setw(6) << transform.a12() << setw(6) << transform.a13() << endl
  *          << setw(6) << transform.a21() << setw(6) << transform.a22() << setw(6) << transform.a23() << endl
  *          << setw(6) << transform.a31() << setw(6) << transform.a32() << setw(6) << transform.a33() << endl
  *          << endl;
  *
  *     ios_base::fmtflags flags = cout.flags();
  *
  *     cout << "Result:    " << endl << endl
  *          << "Error      " << result.error << endl
  *          << "Iterations " << result.number_of_iterations << endl
  *          << "Root       " << fixed << setprecision(3) << result.root << endl << endl;
  *
  *     cout.flags(flags);
  *
  *     cout << clock << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * The output is:
  *
  * \code
  * Original transformation matrix:
  *
  *    2.8  -0.4     1
  *    1.6   1.2    -2
  *      0     0     1
  *
  * Result:
  *
  * Error      8.0176e-14
  * Iterations 3
  * Root       (2.800, -0.400, 1.000, 1.600, 1.200, -2.000)
  *
  * Elapsed time: 0.11 seconds.
  * \endcode
  */

namespace Optimization
{


/** \brief Structure used by several functions to control their operation.
  *
  * Default values:
  *
  * - error_tolerance: 1e-15
  * - lambda: 1e-2
  * - max_number_iterations: 25
  * - nu: 10
  * - spacing: 1e-6
  * - algorithm: LEVENBERG_MARQUARDT
  *
  * \note In order to obtain small rounding errors while computing finit
  *       differences (approximations of the derivatives) you might want
  *       to set the spacing to \f$ \sqrt{\epsilon}x \f$ where the machine
  *       epsilon \f$ \epsilon \f$ is typically of the order 2.2e-16.
  *
  *</p>
  *
  * \note Not all fields are necessarily used by all functions.
  */

template <class ValueType>
struct Options
{
    typedef ValueType value_type;

    value_type   error_tolerance;               ///< This value is used by some algorithms as an abortion criterion.
    value_type   lambda;                        ///< Damping parameter used by the Levenberg-Marquardt algorithm.
    unsigned     max_number_iterations;         ///< Maximum number of iterations performed during an iteration.
    value_type   nu;                            ///< Damping parameter used by the Levenberg-Marquardt algorithm.
    value_type   spacing;                       ///< Spacing used while computing a finit difference (approximation of the derivative).

    enum Algorithm
    {
        LEVENBERG_MARQUARDT,                    ///< Levenberg-Marquardt Algorithm
        NEWTON_RAPHSON                          ///< Newton-Raphson Method
    } algorithm;                                ///< Algorithm used for the computations.

    Options() :
        error_tolerance(1e-15),
        lambda(1e-2),
        max_number_iterations(25),
        nu(10),
        spacing(1e-6),
        algorithm(LEVENBERG_MARQUARDT)
    {
    }
};


/** \brief Structure returned by several functions.
  *
  * \note Not all fields are necessarily used by all functions.
  */

template <class ValueType>
struct Result
{
    typedef Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic> matrix_type;
    typedef TRTK::Coordinate<ValueType> Coordinate;
    typedef ValueType value_type;

    value_type   error;
    matrix_type  jacobian;
    unsigned     number_of_iterations;
    Coordinate   root;

    Result() : error(0), number_of_iterations(0)
    {
    }
};


/** \brief Generates a functor which is the gradient of the given function.
  *
  * \param [in] function        Scalar-valued multivariate function or functor.
  * \param [in] options         Options to control the operation of this function.
  *
  * The gradient is computed using the five-point stencil method. The spacing
  * of the finit differences can be set via \p options.
  *
  * \note Unfortunately C++98 does not deduce the template arguments of classes.
  *       Use \ref makeGradient() to circumvent that.
  *
  * Example:
  *
  * \code
  * #include <cassert>
  * #include <cmath>
  * #include <iostream>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Optimization.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Optimization;
  *
  * double f(const TRTK::Coordinate<double> & arg)
  * {
  *     assert(arg.size() == 2);
  *
  *     const double x = arg.x();
  *     const double y = arg.y();
  *
  *     return (x - 2) * (x - 2)  - (y - 1) * (y - 1) - 9;
  * }
  *
  * int main()
  * {
  *     // Generate the gradient from f.
  *     // YOU MIGHT WANT TO HAVE A LOOK AT "makeGradient".
  *
  *     Gradient<double (const Coordinate<double> &), double> gradient(f);
  *
  *     Coordinate<double> arg(100, -2);
  *     cout << "Gradient(100, -2) = " << gradient(arg) << endl << endl;
  *
  *     // Find the minimum of f (i.e. the zero of grad f).
  *
  *     Result<double> result = solveNewtonRaphsonMethod(gradient, arg);
  *
  *     cout << "Result:    " << endl
  *          << "Error      " << result.error << endl
  *          << "Iterations " << result.number_of_iterations << endl
  *          << "Root       " << result.root << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output
  * \code
  * Gradient(100, -2) = (196, 6)
  *
  * Result:
  * Error      0
  * Iterations 3
  * Root       (2, 1)
  * \endcode
  *
  * \macros  If \macro{TRTK_PARALLELIZE_GRADIENT} is defined \c Gradient
  *          will make use of OpenMP. Be aware that \c function needs to
  *          be reentrant (i.e. it can be safely executed concurrently).
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2013-08-15
  */

template <class Function, class ValueType>
class Gradient
{
public:
    Gradient(Function & function, Options<ValueType> options = Options<ValueType>());
    virtual ~Gradient();

    Coordinate<ValueType> operator()(const Coordinate<ValueType> & value);

private:
    Function & function;
    Options<ValueType> options;
};


/** \brief Constructs a gradient function of the given function. */

template<class Function, class ValueType>
Gradient<Function, ValueType>::Gradient(Function & function, Options<ValueType> options) :
    function(function),
    options(options)
{
}


/** \brief Destructs the instance of Gradient. */

template<class Function, class ValueType>
Gradient<Function, ValueType>::~Gradient()
{
}


/** \brief Returns the gradient. */

template<class Function, class ValueType>
Coordinate<ValueType> Gradient<Function, ValueType>::operator()(const Coordinate<ValueType> & value)
{
    // function : R^n --> R

    const unsigned n = value.size();

    Coordinate<ValueType> gradient;
    gradient.resize(n);

    #ifdef TRTK_PARALLELIZE_GRADIENT
    #pragma omp parallel for
    #endif // TRTK_PARALLELIZE_GRADIENT

    for (int i = 0; i < int(n); ++i)
    {
        // Use the five-point stencil method to compute the derivatives.

        const ValueType spacing = options.spacing;

        Coordinate<ValueType> h;
        h.resize(n);
        h[i] = spacing; // = (0, ..., 0, spacing, 0, ..., 0)

        ValueType a = function(value - 2.0 * h);
        ValueType b = function(value -       h);
        ValueType c = function(value +       h);
        ValueType d = function(value + 2.0 * h);

        gradient[i] = (a - 8.0 * b + 8.0 * c - d) / (12.0 * spacing);
    }

    return gradient;
}


/** \brief Computes the Jacobian of f at x.
  *
  * \param [in] f           Vector-valued multivariate function or functor.
  * \param [in] x           Vector-valued input argument.
  * \param [in] options     Options to control the operation of this function.
  *
  * The Jacobian of \p function is computed numerically using a higher
  * order method, namely the five-point stencil method. The spacing used
  * when computing the finit difference can be set in \p options.
  *
  * \note The type of the input argument as well as the type of the return
  *       value of the given function must be Coordinate.
  *
  * Example:
  *
  * \code
  * #include <cassert>
  * #include <cmath>
  * #include <iostream>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Optimization.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Optimization;
  *
  * Coordinate<double> f(const Coordinate<double> & arg)
  * {
  *     assert(arg.size() == 2);
  *
  *     const double x = arg.x();
  *     const double y = arg.y();
  *
  *     Coordinate<double> result(0, 0);
  *
  *     result.x() = (x - 2) * (y - 1);
  *     result.y() = -3 * y;
  *
  *     return result;
  * }
  *
  * int main()
  * {
  *     Coordinate<double> arg(5, 5);
  *     cout << jacobian(f, arg);
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * 4  3
  * 0 -3
  * \endcode
  *
  * \macros  If \macro{TRTK_PARALLELIZE_JACOBIAN} is defined \c jacobian
  *          will make use of OpenMP. Be aware that \c f needs to be
  *          reentrant (i.e. it can be safely executed concurrently).
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2013-08-15
  */

template <class Function, class ValueType>
Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic> jacobian(Function f, const Coordinate<ValueType> & x, Options<ValueType> options = Options<ValueType>())
{
    using namespace Eigen;

    typedef TRTK::Coordinate<ValueType> Coordinate;

    // function : R^n --> R^m

    const unsigned n = x.size();
    const unsigned m = f(x).size();

    Matrix<ValueType, Dynamic, Dynamic> jacobian(m, n);

    #ifdef TRTK_PARALLELIZE_JACOBIAN
    #pragma omp parallel for
    #endif // TRTK_PARALLELIZE_JACOBIAN

    for (int col = 0; col < int(n); ++col)
    {
        // Use the five-point stencil method to compute the derivatives.

        const ValueType spacing = options.spacing;

        Coordinate h;
        h.resize(n);
        h[col] = spacing; // = (0, ..., 0, spacing, 0, ..., 0)

        Coordinate a = f(x - 2.0 * h);
        Coordinate b = f(x -       h);
        Coordinate c = f(x +       h);
        Coordinate d = f(x + 2.0 * h);

        Coordinate derivatives_in_col = (a - 8.0 * b + 8.0 * c - d) / (12.0 * spacing);

        for (unsigned row = 0; row < m; ++row)
        {
            jacobian(row, col) = derivatives_in_col[row];
        }
    }

    return jacobian;
}


/** \brief Returns an instance of Gradient. The template argument types are automatically deduced.
  *
  * Please, have a look at \ref Gradient for more details.
  *
  * Example:
  *
  * \code
  * #include <cassert>
  * #include <cmath>
  * #include <iostream>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Optimization.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Optimization;
  *
  * double f(const TRTK::Coordinate<double> & arg)
  * {
  *     assert(arg.size() == 2);
  *
  *     const double x = arg.x();
  *     const double y = arg.y();
  *
  *     return (x - 2) * (x - 2)  - (y - 1) * (y - 1) - 9;
  * }
  *
  * int main()
  * {
  *     // Find the minimum of f (i.e. the zero of grad f).
  *
  *     Coordinate<double> arg(100, -2); // initial guess
  *
  *     Result<double> result = solveNewtonRaphsonMethod(makeGradient<double>(f), arg);
  *
  *     cout << "Result:    " << endl
  *          << "Error      " << result.error << endl
  *          << "Iterations " << result.number_of_iterations << endl
  *          << "Root       " << result.root << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Result:
  * Error      0
  * Iterations 3
  * Root       (2, 1)
  * \endcode
  *
  * Example:
  *
  * \code
  * class A
  * {
  * public:
  *     double f(const Coordinate<double> & arg) const
  *     {
  *         // ...
  *     }
  *
  *     double operator()(const Coordinate<double> & arg) const
  *     {
  *         return f(arg);
  *     }
  * };
  *
  * int main()
  * {
  *     // ...
  *     Result<double> result = solveNewtonRaphsonMethod(makeGradient<double>(a), Coordinate<double> arg(0, 0));
  *     // ...
  * }
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2012-11-07
  */

template <class ValueType, class Function>
Gradient<Function, ValueType> makeGradient(Function & function, Options<ValueType> options = Options<ValueType>())
{
    return Gradient<Function, ValueType>(function, options);
}


/** \brief This function solves for the root (zero) of a given function.
  *
  * \param [in] function        Vector-valued multivariate function or functor.
  * \param [in] start_value     Value in the neighborhood of a root.
  * \param [in] options         Options to control the operation of this function.
  *
  * This functions serves as a uniform interface for all solver functions.
  *
  * By default the solver uses the Levenberg-Marquardt algorithm. This can be
  * changed with the \p options parameter. Also, all parameters relevant for a
  * particular algorithm can be set with an option object.
  *
  * For further details please have a look at the respective function documentation.
  *
  * Example:
  *
  * \code
  * #include <cassert>
  * #include <iostream>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Optimization.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Optimization;
  *
  * Coordinate<double> f(const Coordinate<double> & arg)
  * {
  *     assert(arg.size() == 2);
  *
  *     const double x = arg.x();
  *     const double y = arg.y();
  *
  *     Coordinate<double> result(0, 0);
  *
  *     result.x() = (x - 2) * (y - 1);
  *     result.y() = -3 * y;
  *
  *     return result;
  * }
  *
  * int main()
  * {
  *     Coordinate<double> arg(100, -2);
  *
  *     Result<double> result = solve(f, arg);
  *
  *     cout << "Result:    " << endl
  *          << "Iterations " << result.number_of_iterations << endl
  *          << "Error      " << result.error << endl
  *          << "Root       " << result.root << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Result: 
  * Iterations 5
  * Error      2.2523e-023
  * Root       (2, 1.06174e-023)
  * \endcode
  *
  * \see solveLevenbergMarquardt(), solveNewtonRaphsonMethod()
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2012-11-09
  */

template <class Function, class ValueType>
Result<ValueType> solve(Function function, const Coordinate<ValueType> & start_value, Options<ValueType> options = Options<ValueType>())
{
    switch(options.algorithm)
    {
        case Options<ValueType>::LEVENBERG_MARQUARDT:
            return solveLevenbergMarquardt(function, start_value, options);

        case Options<ValueType>::NEWTON_RAPHSON:
            return solveLevenbergMarquardt(function, start_value, options);

        default:
            throw std::invalid_argument("TRTK::Optimization::solve(): Unknown solver algorithm.");
    }
}


/** \brief This function solves for the root (zero) of a given function.
  *
  * \param [in] f                       Unary function or functor (e.g. <tt>double f(double)</tt>).
  * \param [in] start_value             Value in the neighborhood of a root.
  * \param [in] max_number_iterations   Maximum number of iterations to perform.
  *
  * This function implements the Newton-Raphson method. In order to
  * converge, the initial estimate (\p start_value) must be sufficient
  * close to the root. At most \p max_number_iterations iterations are
  * performed. The derivative of \p f is computed numerically using a
  * higher order method (five-point stencil).
  *
  * Example:
  *
  * \code
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Optimization.hpp>
  *
  * using namespace TRTK;
  * using namespace TRTK::Optimization;
  *
  * double f(double x)
  * {
  *     return x * x - 2;
  * }
  *
  * int main()
  * {
  *     solve1D(f, 1.0); // returns sqrt(2)
  *     return 0;
  * }
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2012-11-07
  */

template<class UnaryFunction, class ValueType>
ValueType solve1D(UnaryFunction f, ValueType start_value, unsigned max_number_iterations = 25)
{
    const ValueType epsilon = 0.000001;

    ValueType x = start_value;

    for (unsigned i = 0; i < max_number_iterations; ++i)
    {
        ValueType a = f(x - 2 * epsilon);
        ValueType b = f(x -     epsilon);
        ValueType c = f(x +     epsilon);
        ValueType d = f(x + 2 * epsilon);

        ValueType f_prime = (a - 8 * b + 8 * c - d) / (12 * epsilon); // five-point stencil method

        if (f_prime == 0) break; // Avoid a division by zero.

        x = x - f(x) / f_prime;
    }

    return x;
}


/** \brief This function solves for the root (zero) of a given function.
  *
  * \param [in] function        Vector-valued multivariate function or functor.
  * \param [in] start_value     Value in the neighborhood of a root.
  * \param [in] options         Options to control the operation of this function.
  *
  * This function implements the Levenberg-Marquardt algorithm [1] which
  * aims to minimize the error criterion
  * \f[
  *     \Phi = \| f(x) \|^2 = \sum_{i=0}^{n} f_i(x)^2
  * \f]
  * by iteratively solving the equation
  * \f[
  *     (JJ^T + \lambda \text{diag}(JJ^T)) \Delta x = -J^T f(x)
  * \f]
  * where \f$ f(x + \Delta x) \approx f(x) + J \Delta x \f$.
  *
  * The Levenberg-Marquardt algorithm can be interpreted as interpolating
  * between the Gradient Descent and the Newton-Raphson method depending
  * on the damping parameter \f$ \lambda \f$. A high value of \f$ \lambda \f$
  * leads to a more Gradient Descent like characteristic which is advantageous
  * in the case of a suboptimal start value. The parameter is adjusted at each
  * iteration. The damping can be controlled with the paramters
  * \p options.lambda and \p options.nu.
  *
  * In order to converge, the initial estimate (\p start_value) must be
  * sufficient close to the root. The algorithm is terminated if the
  * residual is smaller than the given error tolerance or if the maximum
  * number of iterations is reached.
  *
  * The Jacobian of \p function is computed numerically using a higher
  * order method (i.e. five-point stencil method). The spacing used when
  * computing the finit difference can be set in \p options.
  *
  * \note The type of the input argument as well as the type of the return
  *       value of the given function must be Coordinate.
  *
  * Example:
  *
  * \code
  * #include <cassert>
  * #include <iostream>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Optimization.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Optimization;
  *
  * Coordinate<double> f(const Coordinate<double> & arg)
  * {
  *     assert(arg.size() == 2);
  *
  *     const double x = arg.x();
  *     const double y = arg.y();
  *
  *     Coordinate<double> result(0, 0);
  *
  *     result.x() = (x - 2) * (y - 1);
  *     result.y() = -3 * y;
  *
  *     return result;
  * }
  *
  * int main()
  * {
  *     Coordinate<double> arg(100, -2);
  *
  *     Result<double> result = solveLevenbergMarquardt(f, arg);
  *
  *     cout << "Result:    " << endl
  *          << "Iterations " << result.number_of_iterations << endl
  *          << "Error      " << result.error << endl
  *          << "Root       " << result.root << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Result: 
  * Iterations 5
  * Error      2.2523e-023
  * Root       (2, 1.06174e-023)
  * \endcode
  *
  * \references
  *
  * [1] Donald W. Marquardt, "An Algorithm for Least Squares Estimation
  *     of Nonlinear Parameters", Journal of the Society dor Industrial
  *     and Applied Mathematics, Vol. 11, No. 2, 1963
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2012-11-07
  */

template <class Function, class ValueType>
Result<ValueType> solveLevenbergMarquardt(Function function, const Coordinate<ValueType> & start_value, Options<ValueType> options = Options<ValueType>())
{
    // The algorithm is implemented as described in the original article
    // of Marquardt. However, instead of scaling the matrix A and the vector
    // g we add lambda * diag(A) to A rather than lambda * I since this is
    // equivalent but more efficient.

    using std::numeric_limits;
    using std::sqrt;

    using namespace Eigen;

    typedef TRTK::Coordinate<ValueType> Coordinate;
    typedef Eigen::Matrix<ValueType, Dynamic, 1> Vector;
    typedef Eigen::Matrix<ValueType, Dynamic, Dynamic> Matrix;

    unsigned iteration = 0;
    const unsigned m = function(start_value).size();    // function : R^n --> R^m
    ValueType error = 0;
    ValueType lambda = options.lambda;                  // damping parameter
    ValueType nu = options.nu;                          // damping parameter
    ValueType Phi = numeric_limits<ValueType>::max();
    Coordinate x = start_value;
    Matrix jacobian;

    assert(lambda > 0);
    assert(nu > 1);

    for (iteration = 0; iteration < options.max_number_iterations; ++iteration)
    {
        // Compute Phi(lambda) and Phi(lambda/nu).

        // Now, compute (A + lambda * diag(A)) * delta = g where A = Jacobian(f)^T *
        // Jacobian(f) and g = Jacobian(f)^T * f. Split the computation due to
        // stability reasons; this also leads to a speed up since no inverse must
        // be computed.

        Vector x1 = x.toArray(); // --> Phi(lambda / nu)
        Vector x2 = x.toArray(); // --> Phi(lambda)
        Vector f  = function(x).toArray();

        jacobian = Optimization::jacobian(function, x, options);

        if (jacobian.norm() == 0)
        {
            error = function(x).norm() / sqrt(ValueType(m)); // RMSE
            break;
        }

        Matrix jj = jacobian.transpose() * jacobian;
        Matrix diagonal = Matrix(jj.diagonal().asDiagonal());
        Matrix left_term1 = jj + lambda / nu * diagonal;
        Matrix left_term2 = jj + lambda * diagonal;
        Vector right_term = jacobian.transpose() * -f;

        // Same as "x1 = x1 - (jj + lambda / nu * diagonal).inverse() * jacobian.transpose() * f;"
        // but faster and more stable:

        Vector delta = left_term1.fullPivLu().solve(right_term);
        x1 = delta + x1;

        delta = left_term2.fullPivLu().solve(right_term);
        x2 = delta + x2;

        ValueType Phi1 = function(Coordinate(x1)).squaredNorm();
        ValueType Phi2 = function(Coordinate(x2)).squaredNorm();

        // Case-by-case analysis

        if (Phi1 <= Phi)
        {
            // case i. in the original article

            lambda = lambda / nu;
            Phi = Phi1;
            x = Coordinate(x1);
        }
        else
        {
            if (Phi2 <= Phi)
            {
                // case ii. in the original article

                lambda = lambda;
                Phi = Phi2;
                x = Coordinate(x2);
            }
            else
            {
                // case iii. in the original article

                Vector x3 = x.toArray();

                ValueType lambda3 = lambda;
                ValueType Phi3 = Phi;

                unsigned w = 0;
                const unsigned MAX_POWER = 100; // this bound is arbitrarily chosen...

                do
                {
                    lambda3 *= nu; // increase lambda by successive multiplication

                    Matrix left_term = jj + lambda3 * diagonal;
                    x3 += left_term.fullPivLu().solve(right_term);

                    Phi3 = function(Coordinate(x3)).squaredNorm();
                }
                while (!(Phi3 <= Phi || w++ >= MAX_POWER));

                if (w > MAX_POWER)
                {
                    // increasing lambda does not seem to work, hence do nothing
                    // and keep the old value (Phi3 might now be even worse...)
                }
                else
                {
                    lambda = lambda3;
                    Phi = Phi3;
                    x = Coordinate(x3);
                }
            }
        }

        // Converged or small enough RMSE?

        Coordinate return_value = function(x);

        // Check whether the error (RMSE) is less than the given error
        // tolerance. Should the situation arise, stop the iteration.

        error = return_value.norm() / sqrt(ValueType(m));

        if (error < options.error_tolerance) break;
    }

    Result<ValueType> result;

    result.error = error;
    result.jacobian = jacobian;
    result.number_of_iterations = iteration + 1;
    result.root = x;

    return result;
}


/** \brief This function solves for the root (zero) of a given function.
  *
  * \param [in] function        Vector-valued multivariate function or functor.
  * \param [in] start_value     Value in the neighborhood of a root.
  * \param [in] options         Options to control the operation of this function.
  *
  * This function implements the Newton-Raphson method. In order to
  * converge, the initial estimate (\p start_value) must be sufficient
  * close to the root. The algorithm is terminated if the residual is
  * smaller than the given error tolerance or if the maximum number of
  * iterations is reached.
  *
  * The Jacobian of \p function is computed numerically using a higher
  * order method (i.e. five-point stencil method). The spacing used when
  * computing the finit difference can be set in \p options.
  *
  * \note The type of the input argument as well as the type of the return
  *       value of the given function must be Coordinate.
  *
  * Example:
  *
  * \code
  * #include <cassert>
  * #include <iostream>
  *
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Optimization.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Optimization;
  *
  * Coordinate<double> f(const Coordinate<double> & arg)
  * {
  *     assert(arg.size() == 2);
  *
  *     const double x = arg.x();
  *     const double y = arg.y();
  *
  *     Coordinate<double> result(0, 0);
  *
  *     result.x() = (x - 2) * (y - 1);
  *     result.y() = -3 * y;
  *
  *     return result;
  * }
  *
  * int main()
  * {
  *     Coordinate<double> arg(100, -2);
  *
  *     Result<double> result = solveNewtonRaphsonMethod(f, arg);
  *
  *     cout << "Result:    " << endl
  *          << "Iterations " << result.number_of_iterations << endl
  *          << "Error      " << result.error << endl
  *          << "Root       " << result.root << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Result:
  * Iterations 3
  * Error      1.04589e-31
  * Root       (2, -4.93038e-32)
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2012-06-10
  */

template <class Function, class ValueType>
Result<ValueType> solveNewtonRaphsonMethod(Function function, const Coordinate<ValueType> & start_value, Options<ValueType> options = Options<ValueType>())
{
    using namespace Eigen;

    using std::sqrt;

    typedef TRTK::Coordinate<ValueType> Coordinate;

    // function : R^n --> R^m

    // const unsigned n = start_value.size();
    const unsigned m = function(start_value).size();

    Coordinate x = start_value;

    Matrix<ValueType, Dynamic, Dynamic> jacobian;

    ValueType error = 0;

    unsigned iteration = 0;

    for (iteration = 0; iteration < options.max_number_iterations; ++iteration)
    {
        Coordinate return_value = function(x);

        // Check whether the error (RMS) is less than the given error
        // tolerance. Should the situation arise, stop the iteration.

        error = return_value.norm() / sqrt(ValueType(m));

        if (error < options.error_tolerance) break;

        jacobian = Optimization::jacobian(function, x, options);

        if (jacobian.norm() == 0) break;

        Matrix<ValueType, Dynamic, 1> x_ = x.toArray();
        Matrix<ValueType, Dynamic, 1> f_ = function(x).toArray();

        // Same as "x_ = x_ - Jacobian.inverse() * f_;" but faster and more stable:

        Matrix<ValueType, Dynamic, 1> delta = jacobian.fullPivLu().solve(-f_);
        x_ = delta + x_;

        x = Coordinate(x_);
    }

    Result<ValueType> result;

    result.error = error;
    result.jacobian = jacobian;
    result.number_of_iterations = iteration;
    result.root = x;

    return result;
}


} // namespace Optimization


} // namespace TRTK


#endif // OPTIMIZATION_HPP_4562354833
