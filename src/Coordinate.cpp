/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.6.0 (2012-07-31)
*/

/** \file Coordinate.cpp
  * \brief This file contains functions related to the \ref TRTK::Coordinate
  *        "Coordinate" class.
  */


#include <Coordinate.hpp>


namespace TRTK
{


// Operators of the form int op Coordinate<class double>

/** \relates Coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * Adds \c value to each component of \c coordinate.
  */

Coordinate<double> operator+(int value, const Coordinate<double> & coordinate)
{
    return coordinate + value;
}


/** \relates Coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * Element-wise subtraction where \c coordinate is subtracted from
  * \f$ (value, value, ..., value)^T \f$.
  */

Coordinate<double> operator-(int value, const Coordinate<double> & coordinate)
{
    return coordinate * (-1) + value;
}


/** \relates Coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * Multiplies each component of \c coordinate with \c value.
  */

Coordinate<double> operator*(int value, const Coordinate<double> & coordinate)
{
    return coordinate * value;
}


/** \relates Coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * Element-wise division of \f$ (value, value, \dots, value)^T \f$ by
  * \c coordinate.
  */

Coordinate<double> operator/(int value, const Coordinate<double> & coordinate)
{
    Coordinate<double> coordinate2;
    coordinate2.resize(coordinate.size(), value);

    return coordinate2.operator/(coordinate);
}


// Operators of the form double op Coordinate<int>

/** \relates Coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * Adds <tt>int(value)</tt> to each component of \c coordinate.
  */

Coordinate<int> operator+(double value, const Coordinate<int> & coordinate)
{
    return coordinate + static_cast<int>(value);
}


/** \relates Coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * Element-wise subtraction where, first, \c value is typecasted into
  * an integer and then, \c coordinate is subtracted from
  * \f$ (value, value, ..., value)^T \f$.
  */

Coordinate<int> operator-(double value, const Coordinate<int> & coordinate)
{
    return coordinate * (-1) + static_cast<int>(value);
}


/** \relates Coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * Multiplies each component of \c coordinate with \c value.
  *
  * \note Internally, the computations are done with doubles.
  *
  */

Coordinate<int> operator*(double value, const Coordinate<int> & coordinate)
{
    return Coordinate<double>(coordinate) * value;
    // return coordinate * static_cast<int>(value);
}


/** \relates Coordinate
  *
  * \param [in] value
  * \param [in] coordinate
  *
  * \return Coordinate
  *
  * Element-wise division of \f$ (value, value, \dots, value)^T \f$ by
  * \c coordinate.
  *
  * \note Internally, the computation is done with doubles.
  */

Coordinate<int> operator/(double value, const Coordinate<int> & coordinate)
{
    Coordinate<double> coordinate2;
    coordinate2.resize(coordinate.size(), value);

    return coordinate2.operator/(coordinate);
}


} // namespace TRTK
