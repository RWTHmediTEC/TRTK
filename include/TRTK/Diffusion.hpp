/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.1 (2011-11-29)
*/

/** \file Diffusion.hpp
  *
  * \brief This file contains several diffusion functions as well as some
  *        helper functions.
  */

#ifndef DIFFUSION_HPP_6431082743
#define DIFFUSION_HPP_6431082743


#include <assert.h>
#include <cmath>
#include <iostream>
#include <vector>

#include <Eigen/Core>

#include "ErrorObj.hpp"
#include "Tools.hpp"


namespace TRTK
{


namespace Diffusion
{


template<class T>
void print(const std::vector<T> & data)
{
    for (unsigned i = 0; i < data.size(); ++i)
    {
        std::cout.precision(4);
        std::cout.width(8);
        std::cout << std::fixed << data[i];
    }
    std::cout << std::endl;
}


/** \brief Interpolation method for border values. */

enum BorderInterpolation
{
    CONTINUED,      //!< Values outside the bounds of the data are the same as the nearest valid border values.
    CIRCULAR,       //!< Values outside the bounds of the data are computed by implicitly assuming the data is periodic.
    INTERPOLATED,   //!< Values outside the bounds of the data are linearly extrapolated from the nearest valid border values.
    REFLECTED,      //!< Values outside the bounds of the data are computed by reflecting the data.
    ZERO            //!< Values outside the bounds of the data are assumed to be zero.
};


/** \brief Error Codes*/

enum Error
{
    INVALID_ARGUMENT,   //!< An invalid argument was assigned.
    UNKNOWN_ERROR       //!< An unknown error occurred.
};

/** \namespace TRTK::Diffusion
  *
  * \brief This namespace contains various functions regarding diffusion
  *        processes.
  */


/** \tparam T scalar type (must be a floating point type)
  *
  * \param [in] signal          input signal
  * \param [in] interpolation   \ref TRTK::Diffusion::BorderInterpolation
  *                             "interpolation method" for the border values
  *
  * \brief Computes the 1st derivative of a 1-dimensional signal.
  *
  * The derivative is computed by convolving \p signal with the kernel
  * <tt>[0.5 0 -0.5]</tt>.
  *
  * \return Derivative of the input signal.
  *
  * \throw ErrorObj If an unknown interpolation method is assigned, an error
  *                 object is thrown and its error code is set to
  *                 \c INVALID_ARGUMENT.
  *
  * Here ist an example:
  *
  * \code
  *
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Diffusion.hpp>
  *
  * using namespace TRTK;
  * using namespace TRTK::Diffusion;
  *
  * void print(const std::vector<double> & data)
  * {
  *     for (unsigned i = 0; i < data.size(); ++i)
  *     {
  *         std::cout.precision(2);
  *         std::cout.width(8);
  *         std::cout << std::fixed << data[i];
  *     }
  *     std::cout << std::endl;
  * }
  *
  * int main()
  * {
  *     std::vector<double> signal;
  *
  *     for (unsigned i = 0; i < 10; ++i)
  *     {
  *         signal.push_back(0.5 * i);
  *     }
  *
  *     print(firstDerivative(signal, CONTINUED));
  *     print(firstDerivative(signal, INTERPOLATED));
  *     print(firstDerivative(signal, REFLECTED));
  *     print(firstDerivative(signal, ZERO));
  *
  *     signal.clear();
  *
  *     for (unsigned i = 0; i < 10; ++i)
  *     {
  *         signal.push_back(i * i + 3);
  *     }
  *
  *     std::cout << std::endl;
  *     print(firstDerivative(signal, CONTINUED));
  *     print(firstDerivative(signal, INTERPOLATED));
  *     print(firstDerivative(signal, REFLECTED));
  *     print(firstDerivative(signal, ZERO));
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
  * 0.25    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.25
  * 0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50
  * 0.25    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.25
  * 0.25    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50   -2.00
  *
  * 0.50    2.00    4.00    6.00    8.00   10.00   12.00   14.00   16.00    8.50
  * 1.00    2.00    4.00    6.00    8.00   10.00   12.00   14.00   16.00   17.00
  * 0.50    2.00    4.00    6.00    8.00   10.00   12.00   14.00   16.00    8.50
  * 2.00    2.00    4.00    6.00    8.00   10.00   12.00   14.00   16.00  -33.50
  *
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2011-11-17
  */

template <class T>
const std::vector<T> firstDerivative(const std::vector<T> & signal, const BorderInterpolation interpolation = CONTINUED)
{
    const unsigned size = signal.size();
    const unsigned n = signal.size() - 1;

    std::vector<T> derivative(size);

    if (size == 0)
    {
        return derivative;
    }
    else if (size == 1)
    {
        derivative.push_back(T(0));
        return derivative;
    }
    else
    {
        // compute boundaries

        // left boundary
        // derivative[0] = 0.5 * (signal[1] - signal[-1])

        switch(interpolation)
        {
            case CONTINUED:
                // signal[-1] := signal[0]
                derivative[0] = 0.5 * (signal[1] - signal[0]);
                break;

            case CIRCULAR:
                // signal[-1] := signal[n]
                derivative[0] = 0.5 * (signal[1] - signal[n]);
                break;

            case INTERPOLATED:
                // signal[-1] := signal[0] + (signal[0] - signal[1]) = 2 * signal[0] - signal[1]
                derivative[0] = signal[1] - signal[0];
                break;

            case REFLECTED:
                // signal[-1] := signal[0]
                derivative[0] = 0.5 * (signal[1] - signal[0]);
                break;

            case ZERO:
                // signal[-1] := 0
                derivative[0] = signal[1] / 2.0;
                break;

            default:
                ErrorObj error("Unknown interpolation method.");
                error.setFunctionName("Differentiate");
                error.setErrorCode(INVALID_ARGUMENT);
                throw error;
        }

        // right boundary
        // derivative[n] = 0.5 * (signal[n+1] - signal[n-1])

        switch(interpolation)
        {
            case CONTINUED:
                // signal[n+1] := signal[n]
                derivative[n] = 0.5 * (signal[n] - signal[n-1]);
                break;

            case CIRCULAR:
                // signal[n+1] := signal[0]
                derivative[n] = 0.5 * (signal[0] - signal[n-1]);
                break;

            case INTERPOLATED:
                // signal[n+1] := signal[n] + (signal[n] - signal[n-1]) = 2 * signal[n] - signal[n-1]
                derivative[n] = signal[n] - signal[n-1];
                break;

            case REFLECTED:
                // signal[n+1] := signal[n]
                derivative[n] = 0.5 * (signal[n] - signal[n-1]);
                break;

            case ZERO:
                // signal[n+1] := 0
                derivative[n] = -signal[n-1] / 2.0;
                break;

            default:
                ErrorObj error("Unknown interpolation method.");
                error.setFunctionName("Differentiate");
                error.setErrorCode(INVALID_ARGUMENT);
                throw error;
        }

        // compute inner part

        for (unsigned i = 1; i < n; ++i)
        {
            derivative[i] = 0.5 * (signal[i+1] - signal[i-1]);
        }

        return derivative;
    }
}


/** \tparam T scalar type (must be a floating point type)
  *
  * \param [in] signal          input signal
  * \param [in] interpolation   \ref TRTK::Diffusion::BorderInterpolation
  *                             "interpolation method" for the border values
  *
  * \brief Computes the 2nd derivative of a 1-dimensional signal.
  *
  * The derivative is computed by convolving \p signal with the kernel
  * <tt>[1 -2 1]</tt>.
  *
  * \return 2nd derivative of the input signal.
  *
  * \throw ErrorObj If an unknown interpolation method is assigned, an error
  *                 object is thrown and its error code is set to
  *                 \c INVALID_ARGUMENT.
  *
  * Here ist an example:
  *
  * \code
  *
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Diffusion.hpp>
  *
  * using namespace TRTK;
  * using namespace TRTK::Diffusion;
  *
  * void print(const std::vector<double> & data)
  * {
  *     for (unsigned i = 0; i < data.size(); ++i)
  *     {
  *         std::cout.precision(2);
  *         std::cout.width(8);
  *         std::cout << std::fixed << data[i];
  *     }
  *     std::cout << std::endl;
  * }
  *
  * int main()
  * {
  *     std::vector<double> signal;
  *
  *     for (unsigned i = 0; i < 10; ++i)
  *     {
  *         signal.push_back(i * i + 3);
  *     }
  *
  *     print(secondDerivative(signal, CONTINUED));
  *     print(secondDerivative(signal, INTERPOLATED));
  *     print(secondDerivative(signal, REFLECTED));
  *     print(secondDerivative(signal, ZERO));
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
  *   1.00    2.00    2.00    2.00    2.00    2.00    2.00    2.00    2.00  -17.00
  *   0.00    2.00    2.00    2.00    2.00    2.00    2.00    2.00    2.00    0.00
  *   1.00    2.00    2.00    2.00    2.00    2.00    2.00    2.00    2.00  -17.00
  *  -2.00    2.00    2.00    2.00    2.00    2.00    2.00    2.00    2.00 -101.00
  *
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2011-06-22
  */

template <class T>
const std::vector<T> secondDerivative(const std::vector<T> & signal, const BorderInterpolation interpolation = CONTINUED)
{
    const unsigned size = signal.size();
    const unsigned n = signal.size() - 1;

    std::vector<T> derivative(size);

    if (size == 0)
    {
        return derivative;
    }
    else if (size == 1)
    {
        derivative.push_back(T(0));
        return derivative;
    }
    else
    {
        // compute boundaries

        // left boundary
        // derivative[0] = signal[-1] - 2 * signal[0] + signal[1]

        switch(interpolation)
        {
            case CONTINUED:
                // signal[-1] := signal[0]
                derivative[0] = -signal[0] + signal[1];
                break;

            case CIRCULAR:
                // signal[-1] := signal[n]
                derivative[0] = signal[n] - 2 * signal[0] + signal[1];
                break;

            case INTERPOLATED:
                // signal[-1] := signal[0] + (signal[0] - signal[1]) = 2 * signal[0] - signal[1]
                derivative[0] =  0;
                break;

            case REFLECTED:
                // signal[-1] := signal[0]
                derivative[0] = -signal[0] + signal[1];
                break;

            case ZERO:
                // signal[-1] := 0
                derivative[0] = -2 * signal[0] + signal[1];
                break;

            default:
                ErrorObj error("Unknown interpolation method.");
                error.setFunctionName("Differentiate");
                error.setErrorCode(INVALID_ARGUMENT);
                throw error;
        }

        // right boundary
        // derivative[n] = signal[n-1] - 2 * signal[n] + signal[n+1]

        switch(interpolation)
        {
            case CONTINUED:
                // signal[n+1] := signal[n]
                derivative[n] = signal[n-1] -signal[n];
                break;

            case CIRCULAR:
                // signal[n+1] := signal[0]
                derivative[n] = signal[n-1] - 2 * signal[n] + signal[0];
                break;

            case INTERPOLATED:
                // signal[n+1] := signal[n] + (signal[n] - signal[n-1]) = 2 * signal[n] - signal[n-1]
                derivative[n] = 0;
                break;

            case REFLECTED:
                // signal[n+1] := signal[n]
                derivative[n] = signal[n-1] -signal[n];
                break;

            case ZERO:
                // signal[n+1] := 0
                derivative[n] = signal[n-1] - 2 * signal[n];
                break;

            default:
                ErrorObj error("Unknown interpolation method.");
                error.setFunctionName("Differentiate");
                error.setErrorCode(INVALID_ARGUMENT);
                throw error;
        }

        // compute inner part

        for (unsigned i = 1; i < n; ++i)
        {
            derivative[i] = signal[i-1] - 2 * signal[i] + signal[i+1];
        }

        return derivative;
    }
}


/** \param [in] signal                  input signal \f$ f(x, t = 0) \f$
  * \param [in] time                    diffusion time \f$ t_{end} \f$
  * \param [in] step_size               should be less than 0.25 to guarantee stability
  * \param [in] interpolation_method    \ref TRTK::Diffusion::BorderInterpolation
  *                                     "interpolation method" used for the differentiation
  *
  * \return Returns the diffused signal \f$ f(x, t = t_{end}) \f$.
  *
  * \brief 1-dimensional linear diffusion.
  *
  * This function computes the diffusion of a given signal \f$ f(x, t = 0) \f$
  * by solving the following diffusion equation:
  *
  * \f[
  *     \frac{d}{dt} f(x, t) = \frac{d^2}{dx^2} f(x, t)
  * \f]
  *
  * Note, that the above diffusion is equivalent to a convolution with a
  * Gaussian kernel with variance \f$ \sigma = \sqrt{2t_{end}} \f$.
  *
  * Here is an example:
  *
  * \code
  *
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Diffusion.hpp>
  *
  * using namespace TRTK;
  * using namespace TRTK::Diffusion;
  *
  * template <typename T, int sz>
  * char (&array(T(&)[sz]))[sz];
  *
  * void print(const std::vector<double> & data)
  * {
  *     for (unsigned i = 0; i < data.size(); ++i)
  *     {
  *         std::cout.precision(2);
  *         std::cout.width(8);
  *         std::cout << std::fixed << data[i];
  *     }
  *     std::cout << std::endl;
  * }
  *
  * int main()
  * {
  *     double data[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  *                      1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  *
  *     std::vector<double> signal =
  *             std::vector<double>(data, data + sizeof array(data));
  *
  *     for (double time = 0; time <= 1; time += 0.25)
  *     {
  *         print(linearDiffusion(signal, time, 0.25));
  *     }
  *
  *     std::cout << std::endl;
  *     print(linearDiffusion(signal, 2.0, 0.25, CONTINUED));
  *     print(linearDiffusion(signal, 2.0, 0.25, INTERPOLATED));
  *     print(linearDiffusion(signal, 2.0, 0.25, REFLECTED));
  *     print(linearDiffusion(signal, 2.0, 0.25, ZERO));
  *
  *     std::cout << std::endl;
  *     print(linearDiffusion(signal, 1000.0, 0.25, CONTINUED));
  *     print(linearDiffusion(signal, 1000.0, 0.25, ZERO));
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
  *  0.00    0.00    0.00    0.00    0.00    0.00    1.00    1.00    1.00    1.00    1.00    1.00
  *  0.00    0.00    0.00    0.00    0.00    0.25    0.75    1.00    1.00    1.00    1.00    1.00
  *  0.00    0.00    0.00    0.00    0.06    0.31    0.69    0.94    1.00    1.00    1.00    1.00
  *  0.00    0.00    0.00    0.02    0.11    0.34    0.66    0.89    0.98    1.00    1.00    1.00
  *  0.00    0.00    0.00    0.04    0.14    0.36    0.64    0.86    0.96    1.00    1.00    1.00
  *
  *  0.00    0.01    0.04    0.11    0.23    0.40    0.60    0.77    0.89    0.96    0.99    1.00
  *  0.00    0.01    0.04    0.11    0.23    0.40    0.60    0.77    0.89    0.96    0.99    1.00
  *  0.00    0.01    0.04    0.11    0.23    0.40    0.60    0.77    0.89    0.96    0.99    1.00
  *  0.00    0.01    0.04    0.11    0.23    0.40    0.60    0.76    0.85    0.82    0.66    0.37
  *
  *  0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50    0.50
  *  0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00    0.00
  *
  * \endcode
  *
  * \note An interpolation method of "INTERPOLATED" might invalidate the
  *       minimum-maximum principle of this diffusion equation.
  *
  * \references Perona and Malik, "Scale-space and edge detection using
  *             anisotropic diffusion", Pattern Analysis and Machine
  *             Intelligence, 1990
  *
  * \references Brox et al., "Nonlinear Structure Tensors",
  *             Image and Vision Computing, 2006
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2011-06-22
  */

template <class T>
const std::vector<T> linearDiffusion(const std::vector<T> & signal,
                                     T time,
                                     T step_size = 0.25,
                                     BorderInterpolation interpolation_method = CONTINUED)
{
    /* The general formulation of a nonlinear diffusion is
     *
     *      d/dt f(x, t) = div grad f(x, t) = laplace f(x, t)
     *
     * In the 1-dimensional case, the divergence and the gradient simplify to
     * an ordinary differentiation. Thus we get
     *
     *      d/dt f(x, t) = d^2/dx^2 f(x, t)
     *
     * The diffusion equation can be easily solved by using the following
     * approximation
     *
     *      d/dt f(x, t) = [f(x, t+h) - f(x, t) ] / h
     *
     * where h is the step size. With this, we get
     *
     *      d/dt f(x, t+h) = f(x, t) + h *  d^2/dx^2 f(x, t)
     *
     * Iterating until t + n * h = time, yields the solution.
     */


    // If time == 0 return the unchanged signal.

    if (Tools::isZero(time))
    {
        return signal;
    }

    // Modify step_size such, that after n iterations, time is really reached.

    step_size = time / std::ceil(time / step_size);

    // Compute the diffusion.

    const unsigned N = signal.size();
    std::vector<T> B(N);

    std::vector<T> diffused_signal = signal;

    for (T t = step_size; t <= time; t += step_size)
    {
        const std::vector<T> & derivative = secondDerivative(diffused_signal, interpolation_method);

        for (unsigned i = 0; i < signal.size(); ++i)
        {
            diffused_signal[i] = diffused_signal[i] + step_size * derivative[i];
        }
    }

    return diffused_signal;
}


/** \param [in] signal                  input signal \f$ f(x, t = 0) \f$
  * \param [in] time                    diffusion time \f$ t_{end} \f$
  * \param [in] step_size               should be less than 0.25 to guarantee stability
  * \param [in] alpha                   steers the diffusion process; should between 0 and 1.5
  * \param [in] interpolation_method    \ref TRTK::Diffusion::BorderInterpolation
  *                                     "interpolation method" used for the differentiation
  *
  * \return Returns the diffused signal \f$ f(x, t = t_{end}) \f$.
  *
  * \brief 1-dimensional nonlinear isotropic diffusion.
  *
  * This function computes the diffusion of a given signal \f$ f(x, t = 0) \f$
  * by solving the following diffusion equation:
  *
  * \f[
  *     \frac{d}{dt} f(x, t) = \frac{d}{dx} \left[ \frac{1}{| \frac{d}{dx} f(x, t) |^\alpha}
  *                                \cdot \frac{d}{dx} f(x, t) \right]
  * \f]
  *
  * Note, if \f$ \alpha = 0 \f$, the above diffusion is a linear diffusion which
  * is equivalent to a convolution with a Gaussian kernel with variance
  * \f$ \sigma = \sqrt{2t_{end}} \f$, and if \f$ \alpha = 1 \f$, the diffusion
  * is equivalent to the total variation (TV) flow.
  *
  * Here is an example:
  *
  * \code
  *
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Diffusion.hpp>
  *
  * using namespace TRTK;
  * using namespace TRTK::Diffusion;
  *
  * template <typename T, int sz>
  * char (&array(T(&)[sz]))[sz];
  *
  * void print(const std::vector<double> & data)
  * {
  *     for (unsigned i = 0; i < data.size(); ++i)
  *     {
  *         std::cout.precision(4);
  *         std::cout.width(8);
  *         std::cout << std::fixed << data[i];
  *     }
  *     std::cout << std::endl;
  * }
  *
  * int main()
  * {
  *     double data[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  *                      1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  *
  *     std::vector<double> signal =
  *             std::vector<double>(data, data + sizeof array(data));
  *
  *     for (double time = 0; time <= 1; time += 0.25)
  *     {
  *         print(nonlinearIsotropicDiffusion(signal, time, 0.25));
  *     }
  *
  *     std::cout << std::endl;
  *     print(nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, CONTINUED));
  *     print(nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, INTERPOLATED));
  *     print(nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, REFLECTED));
  *     print(nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, ZERO));
  *
  *     std::cout << std::endl;
  *     print(nonlinearIsotropicDiffusion(signal, 1000.0, 0.25, 0.0, CONTINUED));
  *     print(nonlinearIsotropicDiffusion(signal, 1000.0, 0.25, 0.0, ZERO));
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
  *   0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  1.0000  1.0000  1.0000  1.0000  1.0000  1.0000
  *   0.0000  0.0000  0.0000  0.0000  0.0625  0.0625  0.9375  0.9375  1.0000  1.0000  1.0000  1.0000
  *   0.0000  0.0000  0.0039  0.0039  0.1133  0.1133  0.8867  0.8867  0.9961  0.9961  1.0000  1.0000
  *   0.0002  0.0002  0.0105  0.0105  0.1548  0.1548  0.8452  0.8452  0.9895  0.9895  0.9998  0.9998
  *   0.0009  0.0009  0.0189  0.0189  0.1889  0.1889  0.8111  0.8111  0.9811  0.9811  0.9991  0.9991
  *
  *   0.0085  0.0085  0.0595  0.0595  0.2772  0.2772  0.7228  0.7228  0.9405  0.9405  0.9915  0.9915
  *   0.0186  0.0100  0.0603  0.0596  0.2773  0.2772  0.7228  0.7227  0.9404  0.9397  0.9900  0.9814
  *   0.0085  0.0085  0.0595  0.0595  0.2772  0.2772  0.7228  0.7228  0.9405  0.9405  0.9915  0.9915
  *   0.0085  0.0078  0.0595  0.0595  0.2765  0.2772  0.7142  0.7228  0.8732  0.9405  0.6555  0.9915
  *
  *   0.5000  0.5000  0.5000  0.5000  0.5000  0.5000  0.5000  0.5000  0.5000  0.5000  0.5000  0.5000
  *   0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000  0.0000
  *
  * \endcode
  *
  * \note An interpolation method of "INTERPOLATED" might invalidate the
  *       minimum-maximum principle of this diffusion equation.
  *
  * \references Perona and Malik, "Scale-space and edge detection using
  *             anisotropic diffusion", Pattern Analysis and Machine
  *             Intelligence, 1990
  *
  * \references Brox et al., "Nonlinear Structure Tensors",
  *             Image and Vision Computing, 2006
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2011-06-22
  */

template <class T>
const std::vector<T> nonlinearIsotropicDiffusion(const std::vector<T> & signal,
                                                 T time,
                                                 T step_size = 0.25,
                                                 T alpha = 0.0,
                                                 BorderInterpolation interpolation_method = CONTINUED)
{
    /* The general formulation of a nonlinear diffusion is
     *
     *      d/dt f(x, t) = div [ D(grad f(x, t)) * grad f(x, t) ]
     *
     * where D is the so-called diffusivity which we define to be
     *
     *      D(x) = 1 / abs(x)^alpha
     *
     * In the 1-dimensional case, the divergence and the gradient simplify to
     * an ordinary differentiation. Thus we get
     *
     *      d/dt f(x, t) = d/dx [ D(d/dx f(x, t)) * d/dx f(x, t) ]
     *
     * The diffusion equation can be easily solved by using the following
     * approximation
     *
     *      d/dt f(x, t) = [f(x, t+h) - f(x, t) ] / h
     *
     * where h is the step size. With this, we get (A, B, and C are used within
     * the implementation)
     *
     *      d/dt f(x, t+h) = f(x, t) + h *  { d/dx [ D(d/dx f(x, t)) * d/dx f(x, t) ] }
     *                     = f(x, t) + h *  { d/dx [ D(A)            * A            ] }
     *                     = f(x, t) + h *  { d/dx [ B                              ] }
     *                     = f(x, t) + h *  { C                                       }
     *
     * Iterating until t + n * h = time, yields the solution.
     */


    // If time == 0 return the unchanged signal.

    if (Tools::isZero(time))
    {
        return signal;
    }

    // Modify step_size such, that after n iterations, time is really reached.

    step_size = time / std::ceil(time / step_size);

    // Compute the diffusion.

    const unsigned N = signal.size();
    std::vector<T> B(N);

    std::vector<T> diffused_signal = signal;

    for (T t = step_size; t <= time; t += step_size)
    {
        const std::vector<T> & A = firstDerivative(diffused_signal, interpolation_method);

        for (unsigned i = 0; i < N; ++i)
        {
            const double epsilon = 1e-14;
            B[i] = 1 / std::pow(std::abs(A[i] + epsilon), alpha) * A[i];
        }

        const std::vector<T> & C = firstDerivative(B, interpolation_method);

        for (unsigned i = 0; i < signal.size(); ++i)
        {
            diffused_signal[i] = diffused_signal[i] + step_size * C[i];
        }
    }

    return diffused_signal;
}


/** \param [in] signal                  input signal \f$ f(x, t = 0) \f$
  * \param [in] diffusivity             diffusivity \f$ D(x) \f$
  * \param [in] time                    diffusion time \f$ t_{end} \f$
  * \param [in] step_size               should be less than 0.25 to guarantee stability
  * \param [in] interpolation_method    \ref TRTK::Diffusion::BorderInterpolation
  *                                     "interpolation method" used for the differentiation
  *
  * \return Returns the diffused signal \f$ f(x, t = t_{end}) \f$.
  *
  * \brief 1-dimensional nonlinear isotropic diffusion.
  *
  * This function computes the diffusion of a given signal \f$ f(x, t = 0) \f$
  * by solving the following diffusion equation:
  *
  * \f[
  *     \frac{d}{dt} f(x, t) = \frac{d}{dx} \left[ D \left( \frac{d}{dx} f(x, t) \right)
  *                                \cdot \frac{d}{dx} f(x, t) \right]
  * \f]
  *
  * Note, if \f$ D(x) = 1 \f$, the above diffusion is a linear diffusion which
  * is equivalent to a convolution with a Gaussian kernel with the variance
  * \f$ \sigma = \sqrt{2t_{end}} \f$.
  *
  * Here is an example:
  *
  * \code
  *
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Diffusion.hpp>
  *
  * using namespace TRTK;
  * using namespace TRTK::Diffusion;
  *
  * template <typename T, int sz>
  * char (&array(T(&)[sz]))[sz];
  *
  * void print(const std::vector<double> & data)
  * {
  *     for (unsigned i = 0; i < data.size(); ++i)
  *     {
  *         std::cout.precision(4);
  *         std::cout.width(8);
  *         std::cout << std::fixed << data[i];
  *     }
  *     std::cout << std::endl;
  * }
  *
  * double diffusivity(double value)
  * {
  *     const double epsilon = 1e-14;
  *     return 1.0 / (std::abs(value) + epsilon);
  * }
  *
  * int main()
  * {
  *     double data[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  *
  *     std::vector<double> signal = std::vector<double>(data, data + sizeof array(data));
  *
  *     for (double time = 0; time <= 1; time += 0.25)
  *     {
  *         print(nonlinearIsotropicDiffusion(signal, diffusivity, time, 0.5));
  *     }
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
  *   0.00    0.00    0.00    0.00    0.00    0.00    1.00    1.00    1.00    1.00    1.00    1.00
  *   0.00    0.00    0.00    0.00    0.12    0.12    0.88    0.88    1.00    1.00    1.00    1.00
  *   0.00    0.00    0.00    0.00    0.25    0.25    0.75    0.75    1.00    1.00    1.00    1.00
  *   0.00    0.00    0.19    0.19    0.19    0.19    0.81    0.81    0.81    0.81    1.00    1.00
  *   0.00    0.00    0.25    0.25    0.25    0.25    0.75    0.75    0.75    0.75    1.00    1.00
  *
  * \endcode
  *
  * \note An interpolation method of "INTERPOLATED" might invalidate the
  *       minimum-maximum principle of this diffusion equation.
  *
  * \references Perona and Malik, "Scale-space and edge detection using
  *             anisotropic diffusion", Pattern Analysis and Machine
  *             Intelligence, 1990
  *
  * \references Brox et al., "Nonlinear Structure Tensors",
  *             Image and Vision Computing, 2006
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2011-06-22
  */

template <class T>
const std::vector<T> nonlinearIsotropicDiffusion(const std::vector<T> & signal,
                                                 T (* diffusivity)(T),
                                                 T time,
                                                 T step_size = 0.25,
                                                 BorderInterpolation interpolation_method = CONTINUED)
{
    /* The general formulation of a nonlinear diffusion is
     *
     *      d/dt f(x, t) = div [ D(grad f(x, t)) * grad f(x, t) ]
     *
     * where D is the so-called diffusivity. In the 1-dimensional case, the
     * divergence and the gradient simplify to an ordinary differentiation.
     * Thus we get
     *
     *      d/dt f(x, t) = d/dx [ D(d/dx f(x, t)) * d/dx f(x, t) ]
     *
     * The diffusion equation can be easily solved by using the following
     * approximation
     *
     *      d/dt f(x, t) = [f(x, t+h) - f(x, t) ] / h
     *
     * where h is the step size. With this, we get (A, B, and C are used within
     * the implementation)
     *
     *      d/dt f(x, t+h) = f(x, t) + h *  { d/dx [ D(d/dx f(x, t)) * d/dx f(x, t) ] }
     *                     = f(x, t) + h *  { d/dx [ D(A)            * A            ] }
     *                     = f(x, t) + h *  { d/dx [ B                              ] }
     *                     = f(x, t) + h *  { C                                       }
     *
     * Iterating until t + n * h = time, yields the solution.
     */


    // If time == 0 return the unchanged signal.

    if (Tools::isZero(time))
    {
        return signal;
    }

    // Modify step_size such, that after n iterations, time is really reached.

    step_size = time / std::ceil(time / step_size);

    // Compute the diffusion.

    const unsigned N = signal.size();
    std::vector<T> B(N);

    std::vector<T> diffused_signal = signal;

    for (T t = step_size; t <= time; t += step_size)
    {
        const std::vector<T> & A = firstDerivative(diffused_signal, interpolation_method);

        for (unsigned i = 0; i < N; ++i)
        {
            B[i] = diffusivity(A[i]) * A[i];
        }

        const std::vector<T> & C = firstDerivative(B, interpolation_method);

        for (unsigned i = 0; i < signal.size(); ++i)
        {
            diffused_signal[i] = diffused_signal[i] + step_size * C[i];
        }
    }

    return diffused_signal;
}


} // namespace Diffusion


} // namespace TRTK


#endif // DIFFUSION_HPP_6431082743
