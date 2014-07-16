/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.2 (2011-06-17)
*/

/** \file Clock.cpp
  * \brief This file contains the definitions of the \ref TRTK::Clock "Clock"
  *        class and related functions.
  */


#include "Clock.hpp"
#include "ErrorObj.hpp"


namespace TRTK
{


/** \brief Constructs a Clock object. */

Clock::Clock() : m_time_when_paused(0), state(RUNNING)
{
    m_time = std::clock();
}


/** \brief Returns the elapsed time in seconds.
  *
  * The elapsed time is the time interval between the point in time of the
  * creation (and resetting, respectively) of this object and the point in time
  * of calling this function.
  *
  * \note If the pause() function is called, the time measurement pauses until
  *       the resume() function is called.
  *
  * \return Elapsed time in seconds.
  *
  * \throw ErrorObj In case of an unexpected error, an \ref TRTK::ErrorObj
  *        "error object" is thrown and its error code is set to
  *        \c UNKNOWN_STATE.
  *
  * \see reset(), pause(), and resume()
  */

double Clock::elapsed_time() const
{
    switch(state)
    {
        case RUNNING:
            return static_cast<double>(std::clock() - m_time) / CLOCKS_PER_SEC;
            break;

        case PAUSED:
            return static_cast<double>(m_time_when_paused) / CLOCKS_PER_SEC;
            break;

		default:
			ErrorObj error;
			error.setClassName("Clock");
			error.setFunctionName("elapsed_time");
			error.setErrorCode(UNKNOWN_STATE);
			throw error;
    }
}


/** \brief Pauses the time measurement.
  *
  * If called, this function pauses the time measurement. The time measurement
  * can be continued by calling the resume() function.
  *
  * \see reset() and resume()
  */

void Clock::pause()
{
    switch(state)
    {
        case RUNNING:
            m_time_when_paused = std::clock() - m_time;
            state = PAUSED;
            break;

        case PAUSED:
            // do nothing
            break;
    }
}


/** \brief Resets the time measurement.
  *
  * \see elapsed_time(), pause(), and resume()
  */

void Clock::reset()
{
    m_time = std::clock();
    m_time_when_paused = 0;
}


/** \brief Resumes the time measurement.
  *
  * If called, this function resumes the time measurement. The time measurement
  * can be stopped by calling the pause() function.
  *
  * \see pause() and reset()
  */

void Clock::resume()
{
    switch(state)
    {
        case RUNNING:
            // do nothing
            break;

        case PAUSED:
            m_time = std::clock() - m_time_when_paused;
            state = RUNNING;
            break;
    }
}


/** \brief Waits for a defined period of time.
  *
  * \param [in] time    time in milliseconds
  *
  * This function waits for \p time milliseconds. This is done by continuously
  * checking if the period of time has already elapsed. Thus, be aware, that
  * this function might be computationally expensive!
  *
  * \note Depeding on the underlying operating system, the function might be
  *       inaccurate!
  *
  * \see wait_seconds()
  */

void Clock::wait_milliseconds(const double time) const
{
    clock_t start_time = std::clock();

    while (static_cast<double>(std::clock() - start_time) / CLOCKS_PER_SEC < time / 1000.0)
    {
        // do nothing
    }
}


/** \brief Waits for a defined period of time.
  *
  * \param [in] time    time in seconds
  *
  * This function waits for \p time seconds. This is done by continuously
  * checking if the period of time has already elapsed. Thus, be aware, that
  * this function might be computationally expensive!
  *
  * \note Depeding on the underlying operating system, the function might be
  *       inaccurate!
  *
  * \see wait_seconds()
  */

void Clock::wait_seconds(const double time) const
{
    clock_t start_time = std::clock();

    while (static_cast<double>(std::clock() - start_time) / CLOCKS_PER_SEC < time)
    {
        // do nothing
    }
}


//////////////////////////////////////////////////////////////////////////////


/** \relates TRTK::Clock
  *
  * \brief Writes the elapsed time of a Clock object to the given stream.
  *
  * Here is an exampe of how to use the stream operator:
  *
  * \code
  * Clock clock;
  * // ...
  * std::cout << clock;
  * \endcode
  *
  * Output:
  *
  * \code
  * Elapsed time: 1.32 seconds.
  * \endcode
  */

std::ostream & operator<< (std::ostream & output, Clock & clock)
{
    output << "Elapsed time: " << clock.elapsed_time() << " seconds." << std::endl;
    return output;
}


} // namespace TRTK
