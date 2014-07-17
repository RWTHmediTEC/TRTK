// Last changed on 2013-03-05.

#include <list>
#include <vector>

#include <TRTK/Iterator.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


using namespace std;
using namespace TRTK;
using namespace TRTK::Tools;


////////////////////////////////////////////////////////////////////////////
//                              Test Classes                              //
////////////////////////////////////////////////////////////////////////////

class Interface
{
public:
    virtual ~Interface() {}
    virtual double sum(Iterator<double> begin, Iterator<double> end) = 0;
};


class Implementation
{
public:
    double sum(Iterator<double> begin, Iterator<double> end)
    {
        double sum = 0;
        Iterator<double> it = begin;

        while (it != end)
        {
            sum += *it;
            ++it;
        }

        return sum;
    }
};


////////////////////////////////////////////////////////////////////////////
//                               Unit Tests                               //
////////////////////////////////////////////////////////////////////////////

void unit_test_Iterator()
{
    HEADING(Iterator)

    SUBHEADING(Iterator and IteratorAdapter)

        vector<double> vec;
        vec.push_back(1);
        vec.push_back(2);

        const vector<double> vec2 = vec;

        vector<int> vec3;
        vec3.push_back(1);
        vec3.push_back(2);

        list<double> lst;
        lst.push_back(1);
        lst.push_back(2);

        Implementation impl;

        double result = 0;

        Iterator<double> it;


        START_TEST
            // dereference operator
            Iterator<double> itBegin = IteratorAdapter<double, vector<double>::iterator>(vec.begin());
            Iterator<double> itEnd = IteratorAdapter<double, vector<double>::iterator>(vec.end());
            assert(isEqual(*itBegin, 1.0));
        STOP_TEST


        START_TEST
            // use with STL
            assert(distance(itBegin, itEnd) == 2);
        STOP_TEST


        START_TEST
            // use with interface class (includes increment and comparison operator)
            result = impl.sum(itBegin, itEnd);
            assert(isEqual(result, 3.0));
        STOP_TEST


        START_TEST
            // conversion of value type of input iterator to value type of Iterator<T>
            Iterator<double> itBegin3 = IteratorAdapter<double, vector<int>::iterator>(vec3.begin());
            Iterator<double> itEnd3 = IteratorAdapter<double, vector<int>::iterator>(vec3.end());
            result = impl.sum(itBegin3, itEnd3);
            assert(isEqual(result, 3.0));
        STOP_TEST


        START_TEST
            // post-increment operator
            itBegin = IteratorAdapter<double, vector<double>::iterator>(vec.begin());
            assert(isEqual(*itBegin++, 1.0));
            assert(isEqual(*itBegin, 2.0));
        STOP_TEST


        START_TEST
            // isValid()
            it = Iterator<double>();
            assert(it.isValid() == false);
        STOP_TEST


        START_TEST
            // isValid()
            it = make_iterator(vec.begin());
            assert(it.isValid() == true);
        STOP_TEST


        START_TEST
            // isValid()
            IteratorAdapter<double, vector<double>::iterator> ita;
            assert(ita.isValid() == false);
        STOP_TEST


        START_TEST
            // isValid()
            ita = vec.begin();
            assert(ita.isValid() == true);
        STOP_TEST


        START_TEST
            // operator bool_type()
            it = Iterator<double>();
            if (it)
            {
                const bool THIS_BRANCH_MAY_NOT_BE_ENTERED = false;
                assert(THIS_BRANCH_MAY_NOT_BE_ENTERED);
            }
        STOP_TEST


        START_TEST
            // operator bool_type()
            it = make_iterator(vec.begin());
            if (it)
            {
                // do nothing
            }
            else
            {
                const bool THIS_BRANCH_MAY_NOT_BE_ENTERED = false;
                assert(THIS_BRANCH_MAY_NOT_BE_ENTERED);
            }
        STOP_TEST


        START_TEST
            // operator bool_type()
            it = Iterator<double>();
            if (!it)
            {
                // do nothing
            }
            else
            {
                const bool THIS_BRANCH_MAY_NOT_BE_ENTERED = false;
                assert(THIS_BRANCH_MAY_NOT_BE_ENTERED);
            }
        STOP_TEST


        START_TEST
            // operator bool_type()
            ita = IteratorAdapter<double, vector<double>::iterator>();
            if (ita)
            {
                const bool THIS_BRANCH_MAY_NOT_BE_ENTERED = false;
                assert(THIS_BRANCH_MAY_NOT_BE_ENTERED);
            }
        STOP_TEST


        START_TEST
            // operator bool_type()
            ita = IteratorAdapter<double, vector<double>::iterator>(vec.begin());
            if (ita)
            {
                // do nothing
            }
            else
            {
                const bool THIS_BRANCH_MAY_NOT_BE_ENTERED = false;
                assert(THIS_BRANCH_MAY_NOT_BE_ENTERED);
            }
        STOP_TEST


        START_TEST
            // operator bool_type()
            ita = IteratorAdapter<double, vector<double>::iterator>();
            if (!ita)
            {
                // do nothing
            }
            else
            {
                const bool THIS_BRANCH_MAY_NOT_BE_ENTERED = false;
                assert(THIS_BRANCH_MAY_NOT_BE_ENTERED);
            }
        STOP_TEST


        START_TEST
            // operator==
            IteratorAdapter<double, vector<double>::iterator> itBegin4(vec.begin());
            IteratorAdapter<double, vector<double>::iterator> itBegin5(vec.begin());
            Iterator<double> itBegin6 = itBegin4;
            Iterator<double> itBegin7 = itBegin4;

            assert(itBegin4 == itBegin4);
            assert(itBegin4 == itBegin5);
            assert(itBegin6 == itBegin6);
            assert(itBegin6 == itBegin7);
            assert(itBegin6 == itBegin4);
            assert(itBegin4 == itBegin6);
        STOP_TEST


        START_TEST
            // operator!=
            IteratorAdapter<double, vector<double>::iterator> itBegin8(vec.end());
            Iterator<double> itBegin9 = itBegin8;

            assert(itBegin4 != itBegin8);
            assert(itBegin4 != itBegin9);
            assert(itBegin8 != itBegin4);
            assert(itBegin9 != itBegin4);
        STOP_TEST


        Iterator<double> it1, it2;
        IteratorAdapter<double, vector<double>::iterator> ia1, ia2;
        IteratorAdapter<double, list<double>::iterator> ia3;

        ia1 = vec.begin();


        // The following checks were performed using a debugger
        // and are confirmed to be ok.

        START_TEST
            it1 = ia1;      // delete nothing; clone ia1
        STOP_TEST


        START_TEST
            it1 = ia1;      // delete clone; clone ia1
        STOP_TEST


        START_TEST
            it1 = it1;      // do nothing
        STOP_TEST


        START_TEST
            it1 = it2;      // delete clone; set internal state to NULL
        STOP_TEST


        START_TEST
            it1 = ia1;      // delete nothing; clone ia1
        STOP_TEST


        START_TEST
            it2 = it1;      // delete clone; clone ia1
        STOP_TEST


        START_TEST
            ia2 = ia1;      // copy ia1
        STOP_TEST


        START_TEST
            ia1 = ia1;      // do nothing
        STOP_TEST


        START_TEST
            ia1 = it1;      // copy ia1
        STOP_TEST


        START_TEST
            ia3 = make_iterator(lst.begin());

            it1 = ia3;

            try
            {
                ia1 = it1;
                const bool TYPE_MISMATCH_NOT_DETECTED = false;
                assert(TYPE_MISMATCH_NOT_DETECTED);
            }
            catch (std::exception &)
            {
            }
        STOP_TEST


    SUBHEADING(make_iterator)


        START_TEST
            // test automatic type deduction
            itBegin = make_iterator(vec.begin()); // type Iterator<double>
            itEnd = make_iterator(vec.end()); // type Iterator<double>
        STOP_TEST


    SUBHEADING(distance)


        START_TEST
            IteratorAdapter<double, vector<double>::iterator> itaBegin(vec.begin());
            IteratorAdapter<double, vector<double>::iterator> itaEnd(vec.end());
            unsigned long dist = distance(itaBegin, itaEnd);
            assert(dist == 2);
        STOP_TEST


        START_TEST
            itBegin = make_iterator(vec.begin()); // type Iterator<double>
            itEnd = make_iterator(vec.end()); // type Iterator<double>
            dist = distance(itBegin, itEnd);
            assert(dist == 2);
        STOP_TEST
}
