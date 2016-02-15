/*
    Error class that incorporates additional information.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.7.1 (2011-09-09)
*/

/** \file ErrorObj.cpp
  * \brief This file contains the implementation of the \ref TRTK::ErrorObj
  *        "ErrorObj" class.
  */

#include <iostream>
#include <sstream>
#include <string>

#include "TRTK/ErrorObj.hpp"


namespace TRTK
{


/** Constructs an empty ErrorObj. */

ErrorObj::ErrorObj() : error_code(0)
{
    return;
}


/** Constructs an ErrorObj.
  *
  * \param [in] error_message Error message that is returned when calling
  *             getErrorMessage() or what().
  */

ErrorObj::ErrorObj(const std::string error_message) :
    error_code(0),
    error_message(error_message)
{
    return;
}


/** Constructs an ErrorObj.
  *
  * \param [in] error_message Error message that is returned when calling
  *             getErrorMessage() or what().
  *
  * \param [in] class_name Class name that is returned when calling
  *             getClassName().
  */

ErrorObj::ErrorObj(const std::string error_message,
                   const std::string class_name) :
    error_code(0),
    class_name(class_name),
    error_message(error_message)
{
    return;
}


/** Constructs an ErrorObj.
  *
  * \param [in] error_message Error message that is returned when calling
  *             getErrorMessage() or what().
  *
  * \param [in] class_name Class name that is returned when calling
  *             getClassName().
  *
  * \param [in] function_name Function name that is returned when calling
  *             getFunctionName().
  *
  * \param [in] error_code Error code that is returned when calling
  *             getErrorCode().
  */

ErrorObj::ErrorObj(const std::string error_message,
                   const std::string class_name,
                   const std::string function_name,
                   const int error_code) :
    error_code(error_code),
    class_name(class_name),
    function_name(function_name),
    error_message(error_message)
{
    return;
}


/** Destroys ErrorObj. */

ErrorObj::~ErrorObj() throw()
{
    return;
}


/** \return Returns the class name. */

std::string ErrorObj::getClassName(void) const
{
    return class_name;
}


/** \return Returns the error code. */

int ErrorObj::getErrorCode(void) const
{
    return error_code;
}


/** \return Returns the error message. */

std::string ErrorObj::getErrorMessage(void) const
{
    return error_message;
}


/** \return Returns the function name. */

std::string ErrorObj::getFunctionName(void) const
{
    return function_name;
}


// /** \return Returns the value stored in ErrorObj. */
//
// boost::any ErrorObj::getValue(void) const
// {
//     return value;
// }
// */


/** \param [in] class_name Class name that is returned when calling
  *             getClassName().
  */

void ErrorObj::setClassName(const std::string class_name)
{
    this->class_name = class_name;
    return;
}


/** \param [in] error_code Error code that is returned when calling
  *             getErrorCode().
  */

void ErrorObj::setErrorCode(const int error_code)
{
    this->error_code = error_code;
    return;
}


/** \param [in] error_message Error message that is returned when calling
  *             getErrorMessage().
  */

void ErrorObj::setErrorMessage(const std::string error_message)
{
    this->error_message = error_message;
    return;
}


/** \param [in] function_name Function name that is returned when calling
  *             getFunctionName().
  */

void ErrorObj::setFunctionName(const std::string function_name)
{
    this->function_name = function_name;
    return;
}


// /**  \param [in] value This can be an arbitrary value that is returned when
//  *               calling getValue().
//  */
//
// void ErrorObj::setValue(const boost::any value)
// {
//     this->value = value;
//     return;
// }
// */


/** Only the pure error message is returned and no further information about
  * the class, function etc. in which the object was thrown.
  *
  * \return Error message.
  *
  * \see what(const Verbosity verbose = NON_VERBOSE)
  */


const char * ErrorObj::what() const throw()
{
    return what(NON_VERBOSE);
}



/** Depending on the input argument this method returns the pure error message
  * or additional information.
  *
  * \param [in] verbose Specifies the level of verbosity.
  *
  *  \see Verbosity and what()
  */

const char * ErrorObj::what(const Verbosity verbose) const throw()
{
    if(verbose)
    {
        std::stringstream ss;
        ss << "Error:" << std::endl;

        if(class_name.size())
            ss << "  - Class: " << class_name << std::endl;

        if(class_name.size())
            ss << "  - Function: " << function_name << std::endl;

        if(class_name.size())
            ss << "  - Error message: " << error_message << std::endl;

        if(class_name.size())
            ss << "  - Error Code: " << error_code << std::endl;

        return ss.str().c_str();
    }
    else
    {
        return error_message.c_str();
    }
}


} // namespace TRTK


// kate: indent-mode cstyle; space-indent on; indent-width 4;
