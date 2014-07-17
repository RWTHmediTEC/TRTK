// Last changed on 2014-07-05


#include <cmath>
#include <sstream>

#include <TRTK/Clock.hpp>

#include "unit_test.hpp"


void unit_test_Clock()
{
    using namespace TRTK;


    HEADING(Clock)

    // Note: Clock::elapsed_time() is assumed to work properly.


    SUBHEADING(Constructor)


        START_TEST
            Clock clock;
        STOP_TEST


    SUBHEADING(wait_seconds())

        const double threshold = 5e-2;

        START_TEST
            clock.wait_seconds(1);
            double residual = std::abs(clock.elapsed_time() - 1.000);
            assert(residual < threshold);
        STOP_TEST


        START_TEST
            clock.wait_seconds(2.123);
            residual = std::abs(clock.elapsed_time() - 3.123);
            assert(residual < threshold);
        STOP_TEST


    SUBHEADING(wait_milliseconds())


        START_TEST
            clock.reset();
            clock.wait_milliseconds(600);
            residual = std::abs(clock.elapsed_time() - 0.600);
            assert(residual < threshold);
        STOP_TEST


    SUBHEADING(reset())


        START_TEST
            clock.reset();
            residual = std::abs(clock.elapsed_time() - 0.000);
            assert(residual < threshold);
        STOP_TEST


        START_TEST
            Clock().wait_seconds(0.5);
            residual = std::abs(clock.elapsed_time() - 0.500);
            assert(residual < threshold);
        STOP_TEST


    SUBHEADING(pause() and resume())


        START_TEST
            clock.pause();
            Clock().wait_seconds(0.5);
            residual = std::abs(clock.elapsed_time() - 0.500);
            assert(residual < threshold);
        STOP_TEST


        START_TEST
            clock.resume();
            Clock().wait_seconds(0.5);
            residual = std::abs(clock.elapsed_time() - 1.000);
            assert(residual < threshold);
        STOP_TEST


        START_TEST
            clock.pause();
            Clock().wait_seconds(0.5);
            residual = std::abs(clock.elapsed_time() - 1.000);
            assert(residual < threshold);
        STOP_TEST


        START_TEST
            clock.resume();
            Clock().wait_seconds(0.5);
            residual = std::abs(clock.elapsed_time() - 1.500);
            assert(residual < threshold);
        STOP_TEST


    SUBHEADING(operator<<())


        START_TEST
            clock.reset();
            clock.wait_seconds(0.5);

            std::stringstream ss;
            ss << clock;

            assert(ss.str().substr(0, 17) == "Elapsed time: 0.5");
            assert(ss.str().substr(ss.str().length()-1-9, 9) == " seconds.");
        STOP_TEST
}
