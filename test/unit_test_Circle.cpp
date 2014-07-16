// Last changed on 2011-04-27


#include <TRTK/Circle.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_Circle()
{
    using namespace TRTK;
    using namespace TRTK::Tools;


    HEADING(Circle)

        START_TEST
            Circle c1;
            assert(isZero(c1.x) &&
                   isZero(c1.y) &&
                   isZero(c1.radius));
        STOP_TEST


        START_TEST
            Circle c2(1.0, 2.0, 20.0);
            assert(isEqual(c2.x, 1.0) &&
                   isEqual(c2.y, 2.0) &&
                   isEqual(c2.radius, 20.0));
        STOP_TEST
}
