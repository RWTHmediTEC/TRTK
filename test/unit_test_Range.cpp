// Last changed on 2013-04-02.

#include <vector>

#include <TRTK/Range.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


using namespace std;
using namespace TRTK;
using namespace TRTK::Tools;


////////////////////////////////////////////////////////////////////////////
//                      Helper Classes and Functions                      //
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//                              Test Classes                              //
////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////
//                               Unit Tests                               //
////////////////////////////////////////////////////////////////////////////

void unit_test_Range()
{
    HEADING(Range)

    double value = 0.0;

    vector<double> vec;
    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);

    vector<double> vec2;

    Iterator<double> begin = make_iterator(vec.begin());
    Iterator<double> end = make_iterator(vec.end());


    SUBHEADING(Constructor)

        START_TEST
            // Default constructor
            Range<double> range;
        STOP_TEST


        START_TEST
            // Constructor
            Range<double> range2(begin, end);
        STOP_TEST


    SUBHEADING(Operators)

        Range<double> range3;
        Range<double> range4;
        Range<double> range5(begin, end);
        Range<double> range6(begin, end);
        Range<double> range7(++begin, end);

        START_TEST
            // operator==
            assert(range3 == range3);
            assert(range3 == range4);
            assert(range5 == range5);
            assert(range5 == range6);
        STOP_TEST


        START_TEST
            // operator!=
            assert(range3 != range5);
            assert(range5 != range7);
        STOP_TEST


    SUBHEADING(begin() and end())

        START_TEST
            Iterator<double> current = range2.begin();
            Iterator<double> last = range2.end();
            value = 0;
            while(current != last)
            {
                value += *current++;
            }
            assert(isEqual(value, 6.0));
        STOP_TEST


    SUBHEADING(isValid())

        START_TEST
            assert(!range3.isValid());
            assert(range5.isValid());
        STOP_TEST


    SUBHEADING(isEmpty())

        START_TEST
            Range<double> range8(make_iterator(vec2.begin()), make_iterator(vec2.end()));
            assert(range3.isEmpty());
            assert(range8.isEmpty());
            assert(!range5.isEmpty());
        STOP_TEST


    SUBHEADING(Traversal)

        START_TEST
            // isDone()
            // currentItem()
            // next()
            value = 0;
            while(!range2.isDone())
            {
                value += range2.currentItem();
                range2.next();
            }
            assert(isEqual(value, 6.0));
        STOP_TEST


        START_TEST
            // first()
            value = 0;
            while(!range2.isDone())
            {
                value += range2.currentItem();
                range2.next();
            }
            assert(isEqual(value, 0.0));
        STOP_TEST


        START_TEST
            // first()
            value = 0;
            range2.first();
            while(!range2.isDone())
            {
                value += range2.currentItem();
                range2.next();
            }
            assert(isEqual(value, 6.0));
        STOP_TEST


        START_TEST
            // copy constructor (after iteration over range2 an iteration over the copied range should be possible)
            Range<double> range9 = range2;
            value = 0;
            while(!range9.isDone())
            {
                value += range9.currentItem();
                range9.next();
            }
            assert(isEqual(value, 6.0));
        STOP_TEST


        START_TEST
            // assignment operator (after iteration over range2 an iteration over the copied rang8 should be possible)
            Range<double> range10;
            range10 = range2;
            value = 0;
            while(!range10.isDone())
            {
                value += range10.currentItem();
                range10.next();
            }
            assert(isEqual(value, 6.0));
        STOP_TEST


    SUBHEADING(size())

        START_TEST
            assert(range2.size() == 3);
        STOP_TEST


    SUBHEADING(make_range())

        START_TEST
            Range<double> range11 = make_range(vec);
            value = 0;
            while(!range11.isDone())
            {
                value += range11.currentItem();
                range11.next();
            }
            assert(isEqual(value, 6.0));
        STOP_TEST
}
