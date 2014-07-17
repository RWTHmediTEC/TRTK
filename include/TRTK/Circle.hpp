/*
    Circle structure.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.1.0 (2011-06-17)
*/

#ifndef CIRCLE_HPP_3691254659
#define CIRCLE_HPP_3691254659


/** \file Circle.hpp
  *
  * \brief This file contains the declaration of the \ref TRTK::Circle
  *        "Circle" structure.
  */


namespace TRTK
{


/** \brief A simple circle structure.
 *
 * \author Christoph Haenisch
 * \version 0.1.0
 * \date last changed on 2011-06-17
 */

struct Circle
{
    Circle();
    Circle(const double x, const double y, const double radius);

    double x;
    double y;
    double radius;
};


} // namespace TRTK


#endif // CIRCLE_HPP_3691254659
