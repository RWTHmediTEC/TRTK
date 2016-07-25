/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.3.1 (2014-07-05)
*/

/** \file Timestamp.cpp
  * \brief This file contains the definitions of the functions in the \ref TRTK::Tools "Tools" namespace.
  */


#include <ctime>
#include <fstream>
#include <string>

#include "TRTK/Tools.hpp"


using namespace std;


namespace TRTK
{


namespace Tools
{


/** \param [in] file_name   Name of the file to check for.
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2011-11-29
  */

bool fileExists(const char * file_name)
{
    ifstream file(file_name);
    if (file)
    {
        file.close();
        return true;
    }
    else
    {
        return false;
    }
}


/** If a file does not exist zero is returned.
  *
  * \code
  *  unsigned long long length = fileLength("file name");
  * \endcode
  *
  * \see fileLength(std::ifstream & file_stream)
  *
  * \author Christoph Haenisch
  * \version 0.2.1
  * \date last changed on 2013-03-19
  */

unsigned long long fileLength(const char * file_name)
{
    ifstream file(file_name, ifstream::binary);

    if (file)
    {
        file.seekg(0, file.end);
        unsigned long long length = (unsigned long long) file.tellg();

        file.close();

        return length;
    }
    else
    {
        return 0;
    }
}


/** \p file_stream must be a valid stream, otherwise the result is undefined.
  *
  * \see fileLength(const char * file_name)
  *
  * \code
  *  std::ifstream file("file name");
  *
  *  if (file)
  *  {
  *      unsigned long long length = fileLength(file);
  *  }
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.2.0
  * \date last changed on 2012-08-03
  */

unsigned long long fileLength(ifstream & file_stream)
{
    unsigned long long old_position = (unsigned long long) file_stream.tellg();

    file_stream.seekg(0, ios::end);
    unsigned long long length = (unsigned long long) file_stream.tellg();

    file_stream.seekg(old_position);

    return length;
}


/** \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2013-08-19
  */

string getCurrentDate()
{
    time_t rawtime = time(0);
    tm * now = localtime(& rawtime);

    int year = now->tm_year + 1900;
    int month = now->tm_mon + 1;
    int day = now->tm_mday;
    
    char buffer[11];
    sprintf(buffer, "%04i-%02i-%02i", year, month, day);
    
    return string(buffer);
}


/** \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2013-08-19
  */

string getCurrentTime()
{
    time_t rawtime = time(0);
    tm * now = localtime(& rawtime);

    int hours = now->tm_hour;
    int minutes = now->tm_min;
    int seconds = now->tm_sec;
    
    char buffer[9];
    sprintf(buffer, "%02i:%02i:%02i", hours, minutes, seconds);

    return string(buffer);
}


/** Convenience function for randn<double>().
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2012-03-15
  */

double randn()
{
    return randn<double>();
}


} // namespace Tools


} // namespace TRTK
