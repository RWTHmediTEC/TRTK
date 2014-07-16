/*
    Commonly used definitions of the TRTK library.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.1.0 (2013-08-16)
*/

/** \file Definitions.hpp
  * \brief This file contains some commonly used constants of the TRTK
  *        library as well as some macro definitions.
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2013-08-16
  */


#ifndef DEFINITIONS_HPP_4576401257
#define DEFINITIONS_HPP_4576401257

#include "Version.hpp"


/* Macros */

/** \brief Declare a class as non-copyable.
  *
  * The macro takes the class name as its first argument. The
  * macro must be placed in the private section of the class.
  *
  * Example:
  *
  * \code
  * class Example
  * {
  *     MAKE_NONCOPYABLE(Example)
  * public:
  *     Example();
  *     // ...
  * };
  * \endcode
  */

#define MAKE_NONCOPYABLE(ClassName) \
    ClassName(const ClassName &); \
    void operator=(const ClassName &);


/** \def TRTK_PARALLELIZE
    *
    * If the macro \macro{TRTK_PARALLELIZE} is defined some algorithms
    * will make use of OpenMP.
    *
    * The definition of \macro{TRTK_PARALLELIZE} will trigger the
    * following definitions:
    * - \macro{TRTK_PARALLELIZE_GRADIENT}
    * - \macro{TRTK_PARALLELIZE_JACOBIAN}
    */

#ifdef TRTK_PARALLELIZE

    #define TRTK_PARALLELIZE
    #define TRTK_PARALLELIZE_GRADIENT
    #define TRTK_PARALLELIZE_JACOBIAN

#endif // TRTK_PARALLELIZE


/* Constants */

namespace TRTK
{
    const double pi = 3.141592653589793238;
}


#endif // DEFINITIONS_HPP_4576401257