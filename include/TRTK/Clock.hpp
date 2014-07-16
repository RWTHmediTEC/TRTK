/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.3 (2013-08-20)
*/

/** \file Clock.hpp
  * \brief This file contains the declarations of the \ref TRTK::Clock "Clock"
  *        class and related functions.
  */

#ifndef CLOCK_HPP_3123312070
#define CLOCK_HPP_3123312070

#include <ctime>
#include <iostream>


namespace TRTK
{


/** \brief A simple clock class.
  *
  * This class provides means for time measurement and some functionality for
  * waiting for a defined period of time. The time measurement can be paused
  * and resumed.
  *
  * Please be aware of the fact, that the wait_seconds() and wait_milliseconds()
  * functions do \b not idle, i.e., they might be computationally expensive!
  *
  * Here is an example which shows the Clock class in action:
  *
  * \code
  *
  * #include <iostream>
  * #include <TRTK/Clock.hpp>
  *
  * using namespace TRTK;
  * using std::cout;
  *
  * int main()
  * {
  *     Clock clock;
  *
  *     clock.wait_seconds(1);
  *     cout << clock;
  *
  *     clock.wait_seconds(2.123);
  *     cout << clock;
  *
  *     clock.reset();
  *     clock.wait_milliseconds(60);
  *     cout << clock;
  *
  *     clock.reset();
  *     cout << clock;
  *
  *     Clock().wait_seconds(0.5);
  *     cout << clock;
  *
  *     clock.pause();
  *     Clock().wait_seconds(0.5);
  *     cout << clock;
  *
  *     clock.resume();
  *     Clock().wait_seconds(0.5);
  *     cout << clock;
  *
  *     return 0;
  * }
  *
  * \endcode
  *
  * This is, what the output might look like:
  *
  * \code
  *
  * Elapsed time: 1 seconds.
  * Elapsed time: 3.124 seconds.
  * Elapsed time: 0.06 seconds.
  * Elapsed time: 0 seconds.
  * Elapsed time: 0.501 seconds.
  * Elapsed time: 0.501 seconds.
  * Elapsed time: 1.001 seconds.
  *
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.1.3
  * \date last changed on 2013-08-20
  */

class Clock
{
public:
    enum Error {UNKNOWN_ERROR, UNKNOWN_STATE};

    Clock();

    double elapsed_time() const;
    void pause();
    void reset();
    void resume();
    void wait_milliseconds(double time) const;
    void wait_seconds(double time) const;

private:
    enum State {RUNNING, PAUSED};

    clock_t m_time;
    clock_t m_time_when_paused;
    State state;
};


std::ostream & operator<< (std::ostream & output, Clock & clock);


} // namespace TRTK


#endif // CLOCK_HPP_3123312070
