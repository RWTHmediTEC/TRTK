/*
    Circle structure.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany
    
    See licences.txt for more information.

    Version 0.1.0 (2011-06-17)
*/

#include "Circle.hpp"


/** \file Circle.cpp
  * \brief This file contains the definition of the \ref TRTK::Circle "Circle"
  *        structure.
  */


namespace TRTK
{


/** \brief Constructs an  instance of Circle.
  *
  * The center point is initialized to (0, 0) and the radius to 0.
  */

Circle::Circle() :
    x(0),
    y(0),
    radius(0)
{
}


/** \brief Constructs an  instance of Circle.
  *
  * \param [in] x       x-coordinate of the center point
  * \param [in] y       y-coordinate of the center point
  * \param [in] radius  circle radius
  *
  * The center point is initialized to (\p x, \p y) and the radius to
  * \p radius.
  */

Circle::Circle(const double x, const double y, const double radius) :
    x(x),
    y(y),
    radius(radius)
{
}


} // namespace TRTK
