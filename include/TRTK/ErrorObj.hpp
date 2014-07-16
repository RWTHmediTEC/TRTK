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

/** \file ErrorObj.hpp
  * \brief This file contains the declaration of the \ref TRTK::ErrorObj
  *        "ErrorObj" class.
  */


#ifndef ERROR_OBJ_HPP_6804646817
#define ERROR_OBJ_HPP_6804646817

// #include <boost/any.hpp> // disabled for the moment
#include <exception>
#include <string>


namespace TRTK
{


/** \class ErrorObj
  *
  * \brief Error class that incorporates additional information.
  *
  * Compared to \c std::exception ErrorObj allows to store additional information
  * such as the name of the class and/or function in which is was thrown. In fact,
  * ErrorObj is derived from the standard exception class, that is, catching can
  * be done as for most components of the standard library like
  *
  * \code
  * try
  * {
  *     ...
  * }
  * catch (std::exception & e)
  * {
  *     ...
  * }
  * \endcode
  *
  * The virtual member function what() works as expected.
  *
  * If you want to be able to get access to possibly further stored information,
  * you must catch a reference to ErrorObj like
  *
  * \code
  * try
  * {
  *     ...
  * }
  * catch (TRTK::ErrorObj & e)
  * {
  *     ...
  * }
  * \endcode
  *
  * Now you are able to access information like the class or function name, for
  * instance. You can use the overloaded function of what() to get a more
  * verbose error message, that is, in addition to the pure error message all
  * other available information is returned.
  *
  * Here is a more elaborated example of how to use ErrorObj
  *
  * \code
  * #include <iostream>
  * #include <TRTK/ErrorObj.hpp>
  *
  * using namespace TRTK;
  *
  * class A
  * {
  * public:
  *     void func()
  *     {
  *         // ...
  *         ErrorObj error("An error occurred.");
  *         error.setClassName("A");
  *         error.setFunctionName("func");
  *         throw error;
  *     }
  * };
  *
  * class B
  * {
  * public:
  *     void func()
  *     {
  *         A a;
  *         a.func();
  *     }
  * };
  *
  * int main()
  * {
  *     B b;
  *
  *     try
  *     {
  *         b.func();
  *     }
  *     catch(ErrorObj & error)
  *     {
  *         std::cout << error.what(ErrorObj::VERBOSE);
  *     }
  *
  *     return 0;
  * }
  * \endcode
  *
  * The output is
  *
  * \code
  * Error:
  *   - Class: A
  *   - Function: func
  *   - Error message: An error occurred.
  *   - Error Code: 0
  * \endcode
  *
  * \warning When using exceptions, you need to compile all of your code using
  *          the same compiler, since exceptions do not work across compiler
  *          bounds! Keep this in mind, when building this class in a static or
  *          dynamic library.
  *
  * \author  Christoph Haenisch
  * \version 0.7.1
  * \date    last changed on 2011-09-09
  */

class ErrorObj : public std::exception
{
public:
    ErrorObj();

    ErrorObj(const std::string error_message);

    ErrorObj(const std::string error_message,
             const std::string class_name);

    ErrorObj(const std::string error_message,
             const std::string class_name,
             const std::string function_name,
             const int error_code = 0);

    virtual ~ErrorObj() throw();

    /** \brief ErrorObj::Verbosity denotes whether the output of
      * <tt> what(const Verbosity) </tt> is more or less detailed.
      *
      * \note \c NON_VERBOSE is always assumed to be zero.
      */

    enum Verbosity {NON_VERBOSE,    ///< return error message only when calling what()
                    VERBOSE         ///< return all information available when calling what()
    };

    virtual const char * what() const throw();                                  ///< Returns the error message.
    virtual const char * what(const Verbosity = NON_VERBOSE) const throw();     ///< Returns the error message.

    std::string getClassName(void) const;       ///< Returns the stored class name (this should be the name of the class in which ErrorObj was thrown).
    int getErrorCode(void) const;               ///< Returns the stored error Code.
    std::string getErrorMessage(void) const;    ///< Returns the stored error message.
    std::string getFunctionName(void) const;    ///< Returns the stored function name (this should be the name of the function in which ErrorObj was thrown).
    // boost::any getValue(void) const;         ///< Returns the stored value. // disabled for the moment

    void setClassName(const std::string class_name);        ///< Sets the class name (this should be the name of the class in which ErrorObj is thrown).
    void setErrorCode(const int error_code);                ///< Sets the error code.
    void setErrorMessage(const std::string error_message);  ///< Sets the error message.
    void setFunctionName(const std::string function_name);  ///< Sets the function name (this should be the name of the function in which ErrorObj is thrown).
    // void setValue(const boost::any value);               ///< Stores an abitrary value. // disabled for the moment

private:
    int error_code;
    std::string class_name;
    std::string function_name;
    std::string error_message;
    // boost::any value; // disabled for the moment
};


} // namespace TRTK


#endif // ERROR_OBJ_HPP_6804646817

// kate: indent-mode cstyle; space-indent on; indent-width 4;
