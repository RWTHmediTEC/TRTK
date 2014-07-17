// Last changed on 2014-06-11

#include <cassert>
#include <vector>

#include <TRTK/Signals.hpp>

#include "unit_test.hpp"


///////////////////////////////////////
//               Counter             //
///////////////////////////////////////

class Counter
{
public:

    Counter()
    {
        counter = 0;
    }

    void incCounter()

    {
        counter += 1;
    }

    const int getCounter() const
    {
        return counter;
    }

    void reset()
    {
        counter = 0;
    }

private:

    int counter;
};

Counter counter;


///////////////////////////////////////
//         Global Functions          //
///////////////////////////////////////

void GlobalFunction0()
{
    counter.incCounter();
}

void GlobalFunction1(int a)
{
    assert(a == 1);
    counter.incCounter();
}

void GlobalFunction2(int a, double b)
{
    assert(a == 1);
    assert(b == 2.2);
    counter.incCounter();
}

void GlobalFunction3(int a, double b, char c)
{
    assert(a == 1);
    assert(b == 2.2);
    assert(c == 'a');
    counter.incCounter();
}

void GlobalFunction4(int a, double b, char c, unsigned d)
{
    assert(a == 1);
    assert(b == 2.2);
    assert(c == 'a');
    assert(d == 4);
    counter.incCounter();
}


///////////////////////////////////////////////////////
//         Member Functions (No Inheritance)         //
///////////////////////////////////////////////////////

class TestMemberFunctions
{
public:
    void function0()
    {
        counter.incCounter();
    }

    void function1(int a)
    {
        assert(a == 1);
        counter.incCounter();
    }

    void function2(int a, double b)
    {
        assert(a == 1);
        assert(b == 2.2);
        counter.incCounter();
    }

    void function3(int a, double b, char c)
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        counter.incCounter();
    }

    void function4(int a, double b, char c, unsigned d)
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        assert(d == 4);
        counter.incCounter();
    }

    void constFunction0() const
    {
        counter.incCounter();
    }

    void constFunction1(int a) const
    {
        assert(a == 1);
        counter.incCounter();
    }

    void constFunction2(int a, double b) const
    {
        assert(a == 1);
        assert(b == 2.2);
        counter.incCounter();
    }

    void constFunction3(int a, double b, char c) const
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        counter.incCounter();
    }

    void constFunction4(int a, double b, char c, unsigned d) const
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        assert(d == 4);
        counter.incCounter();
    }
};


///////////////////////////////////////////////////////////
//         Member Functions (Single Inheritance)         //
///////////////////////////////////////////////////////////

class TestMemberFunctionsInterface
{
public:
    virtual ~TestMemberFunctionsInterface() {};

    virtual void function0() = 0;
    virtual void function1(int a) = 0;
    virtual void function2(int a, double b) = 0;
    virtual void function3(int a, double b, char c) = 0;
    virtual void function4(int a, double b, char c, unsigned d) = 0;

    virtual void constFunction0() const = 0;
    virtual void constFunction1(int a) const = 0;
    virtual void constFunction2(int a, double b) const = 0;
    virtual void constFunction3(int a, double b, char c) const = 0;
    virtual void constFunction4(int a, double b, char c, unsigned d) const = 0;
};

class TestMemberFunctionsSingleInheritance : public TestMemberFunctions
{
public:
    void function0()
    {
        counter.incCounter();
    }

    void function1(int a)
    {
        assert(a == 1);
        counter.incCounter();
    }

    void function2(int a, double b)
    {
        assert(a == 1);
        assert(b == 2.2);
        counter.incCounter();
    }

    void function3(int a, double b, char c)
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        counter.incCounter();
    }

    void function4(int a, double b, char c, unsigned d)
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        assert(d == 4);
        counter.incCounter();
    }

    void constFunction0() const
    {
        counter.incCounter();
    }

    void constFunction1(int a) const
    {
        assert(a == 1);
        counter.incCounter();
    }

    void constFunction2(int a, double b) const
    {
        assert(a == 1);
        assert(b == 2.2);
        counter.incCounter();
    }

    void constFunction3(int a, double b, char c) const
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        counter.incCounter();
    }

    void constFunction4(int a, double b, char c, unsigned d) const
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        assert(d == 4);
        counter.incCounter();
    }
};


/////////////////////////////////////////////////////////////
//         Member Functions (Multiple Inheritance)         //
/////////////////////////////////////////////////////////////

class Base
{
public:
    virtual ~Base() {};
};

class TestMemberFunctionsMultipleInheritance : public TestMemberFunctions, public Base
{
public:
    void function0()
    {
        counter.incCounter();
    }

    void function1(int a)
    {
        assert(a == 1);
        counter.incCounter();
    }

    void function2(int a, double b)
    {
        assert(a == 1);
        assert(b == 2.2);
        counter.incCounter();
    }

    void function3(int a, double b, char c)
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        counter.incCounter();
    }

    void function4(int a, double b, char c, unsigned d)
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        assert(d == 4);
        counter.incCounter();
    }

    void constFunction0() const
    {
        counter.incCounter();
    }

    void constFunction1(int a) const
    {
        assert(a == 1);
        counter.incCounter();
    }

    void constFunction2(int a, double b) const
    {
        assert(a == 1);
        assert(b == 2.2);
        counter.incCounter();
    }

    void constFunction3(int a, double b, char c) const
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        counter.incCounter();
    }

    void constFunction4(int a, double b, char c, unsigned d) const
    {
        assert(a == 1);
        assert(b == 2.2);
        assert(c == 'a');
        assert(d == 4);
        counter.incCounter();
    }
};


//////////////////////////////////////////////////
//    Member functions derived from Receiver    //
//////////////////////////////////////////////////

class TestReceiver : public TRTK::Receiver
{
public:
    TestReceiver()
    {
    }

    void function() const
    {
        counter.incCounter();
    }
};


class TestReceiver2 : public Base, virtual public TRTK::Receiver
{
public:
    TestReceiver2()
    {
    }

    void function() const
    {
        counter.incCounter();
    }
};


///////////////////////////////////////
//            Mutability             //
///////////////////////////////////////

class TestMutability
{
public:
    TestMutability()
    {
    }

    void notify() const
    {
        signal.send();
    }

    TRTK::Signal<> signal;
};


///////////////////////////////////////
//            Unit tests             //
///////////////////////////////////////

void unit_test_Signals()
{
    using namespace TRTK;


    HEADING(Signals)

        TestMemberFunctions testMemberFunctions;
        TestMemberFunctionsSingleInheritance testMemberFunctionsSingleInheritance;
        TestMemberFunctionsMultipleInheritance testMemberFunctionsMultipleInheritance;

        Signal<> s0;
        Signal<int> s1;
        Signal<int, double> s2;
        Signal<int, double, char> s3;
        Signal<int, double, char, unsigned> s4;


    SUBHEADING(Global Functions (SINGLE_CONNECTION))

        START_TEST
            counter.reset();
            s0.connect(&GlobalFunction0);
            s0.connect(&GlobalFunction0);
            s0.send();
            assert(counter.getCounter() == 1);

            s0.disconnect(&GlobalFunction0);
            s0.send();
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&GlobalFunction1);
            s1.connect(&GlobalFunction1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            s1.disconnect(&GlobalFunction1);
            s1.send(1);
            assert(counter.getCounter() == 1);
            counter.reset();
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&GlobalFunction2);
            s2.connect(&GlobalFunction2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            s2.disconnect(&GlobalFunction2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&GlobalFunction3);
            s3.connect(&GlobalFunction3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            s3.disconnect(&GlobalFunction3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&GlobalFunction4);
            s4.connect(&GlobalFunction4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            s4.disconnect(&GlobalFunction4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

    SUBHEADING(Global Functions (MULTIPLE_CONNECTIONS))

        START_TEST
            counter.reset();
            s0.connect(&GlobalFunction0);
            s0.connect(&GlobalFunction0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            s0.disconnect(&GlobalFunction0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&GlobalFunction1);
            s1.connect(&GlobalFunction1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);

            s1.disconnect(&GlobalFunction1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&GlobalFunction2);
            s2.connect(&GlobalFunction2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);

            s2.disconnect(&GlobalFunction2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&GlobalFunction3);
            s3.connect(&GlobalFunction3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);

            s3.disconnect(&GlobalFunction3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&GlobalFunction4);
            s4.connect(&GlobalFunction4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);

            s4.disconnect(&GlobalFunction4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);
        STOP_TEST

    SUBHEADING(Member Functions (SINGLE_CONNECTION, No Inheritance))

        // non-const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctions, &TestMemberFunctions::function0);
            s0.connect(&testMemberFunctions, &TestMemberFunctions::function0);
            s0.send();
            assert(counter.getCounter() == 1);

            s0.disconnect(&testMemberFunctions, &TestMemberFunctions::function0);
            s0.send();
            assert(counter.getCounter() == 1);

            // restore initial state
            s0.disconnect(&testMemberFunctions, &TestMemberFunctions::function0);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctions, &TestMemberFunctions::function1);
            s1.connect(&testMemberFunctions, &TestMemberFunctions::function1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            s1.disconnect(&testMemberFunctions, &TestMemberFunctions::function1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            // restore initial state
            s1.disconnect(&testMemberFunctions, &TestMemberFunctions::function1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctions, &TestMemberFunctions::function2);
            s2.connect(&testMemberFunctions, &TestMemberFunctions::function2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::function2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            // restore initial state
            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::function2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::function3);
            s3.connect(&testMemberFunctions, &TestMemberFunctions::function3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::function3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            // restore initial state
            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::function3);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::function4);
            s4.connect(&testMemberFunctions, &TestMemberFunctions::function4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::function4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            // restore initial state
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::function4);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s0.connect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s0.send();
            assert(counter.getCounter() == 1);

            s0.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s0.send();
            assert(counter.getCounter() == 1);

            // restore initial state
            s0.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s1.connect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            s1.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            // restore initial state
            s1.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
            s2.connect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            // restore initial state
            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::constFunction3);
            s3.connect(&testMemberFunctions, &TestMemberFunctions::constFunction3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            // restore initial state
            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction3);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::constFunction4);
            s4.connect(&testMemberFunctions, &TestMemberFunctions::constFunction4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            // restore initial state
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction4);
        STOP_TEST

    SUBHEADING(Member Functions (MULTIPLE_CONNECTIONS, No Inheritance))

        // non-const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctions, &TestMemberFunctions::function0);
            s0.connect(&testMemberFunctions, &TestMemberFunctions::function0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            s0.disconnect(&testMemberFunctions, &TestMemberFunctions::function0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctions, &TestMemberFunctions::function1);
            s1.connect(&testMemberFunctions, &TestMemberFunctions::function1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);

            s1.disconnect(&testMemberFunctions, &TestMemberFunctions::function1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctions, &TestMemberFunctions::function2);
            s2.connect(&testMemberFunctions, &TestMemberFunctions::function2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);

            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::function2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::function3);
            s3.connect(&testMemberFunctions, &TestMemberFunctions::function3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);

            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::function3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::function4);
            s4.connect(&testMemberFunctions, &TestMemberFunctions::function4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);

            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::function4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s0.connect(&testMemberFunctions, &TestMemberFunctions::constFunction0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            s0.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s1.connect(&testMemberFunctions, &TestMemberFunctions::constFunction1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);

            s1.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
            s2.connect(&testMemberFunctions, &TestMemberFunctions::constFunction2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);

            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::constFunction3);
            s3.connect(&testMemberFunctions, &TestMemberFunctions::constFunction3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);

            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::constFunction4);
            s4.connect(&testMemberFunctions, &TestMemberFunctions::constFunction4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);

            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);
        STOP_TEST

    SUBHEADING(Member Functions (SINGLE_CONNECTIONS, Single Inheritance))

        // non-const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function0);
            s0.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function0);
            s0.send();
            assert(counter.getCounter() == 1);

            s0.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function0);
            s0.send();
            assert(counter.getCounter() == 1);

            // restore initial state
            s0.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function0);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function1);
            s1.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            s1.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            // restore initial state
            s1.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function2);
            s2.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            s2.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            // restore initial state
            s2.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function3);
            s3.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            s3.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            // restore initial state
            s3.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function3);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function4);
            s4.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            s4.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            // restore initial state
            s4.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function4);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction0);
            s0.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction0);
            s0.send();
            assert(counter.getCounter() == 1);

            s0.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction0);
            s0.send();
            assert(counter.getCounter() == 1);

            // restore initial state
            s0.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction0);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction1);
            s1.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            s1.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            // restore initial state
            s1.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction2);
            s2.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            s2.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            // restore initial state
            s2.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction3);
            s3.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            s3.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            // restore initial state
            s3.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction3);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction4);
            s4.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            s4.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            // restore initial state
            s4.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction4);
        STOP_TEST

    SUBHEADING(Member Functions (MULTIPLE_CONNECTIONS, Single Inheritance))

        // non-const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function0);
            s0.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            s0.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function1);
            s1.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);

            s1.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function2);
            s2.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);

            s2.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function3);
            s3.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);

            s3.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function4);
            s4.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);

            s4.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::function4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction0);
            s0.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            s0.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction1);
            s1.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);

            s1.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction2);
            s2.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);

            s2.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction3);
            s3.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);

            s3.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction4);
            s4.connect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);

            s4.disconnect(&testMemberFunctionsSingleInheritance, &TestMemberFunctionsSingleInheritance::constFunction4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);
        STOP_TEST

    SUBHEADING(Member Functions (SINGLE_CONNECTIONS, Multiple Inheritance))

        // non-const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function0);
            s0.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function0);
            s0.send();
            assert(counter.getCounter() == 1);

            s0.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function0);
            s0.send();
            assert(counter.getCounter() == 1);

            // restore initial state
            s0.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function0);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function1);
            s1.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            s1.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            // restore initial state
            s1.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function2);
            s2.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            s2.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            // restore initial state
            s2.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function3);
            s3.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            s3.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            // restore initial state
            s3.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function3);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function4);
            s4.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            s4.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            // restore initial state
            s4.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function4);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction0);
            s0.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction0);
            s0.send();
            assert(counter.getCounter() == 1);

            s0.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction0);
            s0.send();
            assert(counter.getCounter() == 1);

            // restore initial state
            s0.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction0);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction1);
            s1.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            s1.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction1);
            s1.send(1);
            assert(counter.getCounter() == 1);

            // restore initial state
            s1.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction2);
            s2.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            s2.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction2);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);

            // restore initial state
            s2.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction3);
            s3.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            s3.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction3);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);

            // restore initial state
            s3.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction3);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction4);
            s4.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            s4.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction4);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);

            // restore initial state
            s4.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction4);
        STOP_TEST

    SUBHEADING(Member Functions (MULTIPLE_CONNECTIONS, Multiple Inheritance))

        // non-const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function0);
            s0.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            s0.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function1);
            s1.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);

            s1.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function2);
            s2.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);

            s2.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function3);
            s3.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);

            s3.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function4);
            s4.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);

            s4.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::function4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s0.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction0);
            s0.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            s0.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction0, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction1);
            s1.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);

            s1.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction1, MULTIPLE_CONNECTIONS);
            s1.send(1);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction2);
            s2.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);

            s2.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction2, MULTIPLE_CONNECTIONS);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction3);
            s3.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);

            s3.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction3, MULTIPLE_CONNECTIONS);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction4);
            s4.connect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);

            s4.disconnect(&testMemberFunctionsMultipleInheritance, &TestMemberFunctionsMultipleInheritance::constFunction4, MULTIPLE_CONNECTIONS);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 2);
        STOP_TEST

    SUBHEADING(Member Functions Derived from Receiver)

        START_TEST
            counter.reset();

            TestReceiver * testReceiver = new TestReceiver;

            s0.connect(testReceiver, &TestReceiver::function);
            s0.connect(testReceiver, &TestReceiver::function, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            delete testReceiver; // now all connections testReceiver should be disconnected

            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();

            TestReceiver2 * testReceiver2 = new TestReceiver2;

            s0.connect(testReceiver2, &TestReceiver2::function);
            s0.connect(testReceiver2, &TestReceiver2::function, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            delete testReceiver2; // now all connections testReceiver should be disconnected

            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

    SUBHEADING(Mutability) // connections to const objects

        START_TEST
            counter.reset();
            const TestMutability testMutability;

            testMutability.signal.connect(&GlobalFunction0);
            testMutability.notify();
            assert(counter.getCounter() == 1);

            testMutability.signal.disconnect(&GlobalFunction0);
            testMutability.notify();
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();

            const TestReceiver * testReceiver3 = new const TestReceiver;

            s0.connect(testReceiver3, &TestReceiver::function);
            s0.connect(testReceiver3, &TestReceiver::function, MULTIPLE_CONNECTIONS);
            s0.send();
            assert(counter.getCounter() == 2);

            delete testReceiver3; // now all connections testReceiver2 should be disconnected

            s0.send();
            assert(counter.getCounter() == 2);
        STOP_TEST

    SUBHEADING(Connections to Slots With a Smaller Number of Parameters Than the Signal)

        // non-const member functions

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::function0);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::function0);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::function1);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::function1);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::function2);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::function2);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::function3);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::function3);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&testMemberFunctions, &TestMemberFunctions::constFunction3);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction3);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        // global or a static member functions

        START_TEST
            counter.reset();
            s4.connect(&GlobalFunction0);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&GlobalFunction0);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&GlobalFunction1);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&GlobalFunction1);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&GlobalFunction2);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&GlobalFunction2);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s4.connect(&GlobalFunction3);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
            s4.disconnect(&GlobalFunction3);
            s4.send(1, 2.2, 'a', 4);
            assert(counter.getCounter() == 1);
        STOP_TEST


        // non-const member functions

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::function0);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::function0);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::function1);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::function1);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::function2);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::function2);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction2);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        // global or a static member functions

        START_TEST
            counter.reset();
            s3.connect(&GlobalFunction0);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&GlobalFunction0);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&GlobalFunction1);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&GlobalFunction1);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s3.connect(&GlobalFunction2);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
            s3.disconnect(&GlobalFunction2);
            s3.send(1, 2.2, 'a');
            assert(counter.getCounter() == 1);
        STOP_TEST


        // non-const member functions

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctions, &TestMemberFunctions::function0);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::function0);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctions, &TestMemberFunctions::function1);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::function1);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
            s2.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction1);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
        STOP_TEST

        // global or a static member functions

        START_TEST
            counter.reset();
            s2.connect(&GlobalFunction0);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
            s2.disconnect(&GlobalFunction0);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
        STOP_TEST

        START_TEST
            counter.reset();
            s2.connect(&GlobalFunction1);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
            s2.disconnect(&GlobalFunction1);
            s2.send(1, 2.2);
            assert(counter.getCounter() == 1);
        STOP_TEST


        // non-const member functions

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctions, &TestMemberFunctions::function0);
            s1.send(1);
            assert(counter.getCounter() == 1);
            s1.disconnect(&testMemberFunctions, &TestMemberFunctions::function0);
            s1.send(1);
            assert(counter.getCounter() == 1);
        STOP_TEST

        // const member functions

        START_TEST
            counter.reset();
            s1.connect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s1.send(1);
            assert(counter.getCounter() == 1);
            s1.disconnect(&testMemberFunctions, &TestMemberFunctions::constFunction0);
            s1.send(1);
            assert(counter.getCounter() == 1);
        STOP_TEST

        // global or a static member functions

        START_TEST
            counter.reset();
            s1.connect(&GlobalFunction0);
            s1.send(1);
            assert(counter.getCounter() == 1);
            s1.disconnect(&GlobalFunction0);
            s1.send(1);
            assert(counter.getCounter() == 1);
        STOP_TEST
}
