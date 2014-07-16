// Last changed on 2011-04-25.


#include <ctime>
#include <vector>

#include <TRTK/Timestamp.hpp>

#include "unit_test.hpp"


using std::cout;
using std::endl;


void wait(const double seconds)
{
    clock_t time = std::clock();

    while (double(std::clock() - time) / CLOCKS_PER_SEC < seconds)
    {
        // do nothing
    }
}

class TimestampTestClass
{
public:

    TimestampTestClass()
    {
        // Initialize the object dependencies.

        timestamp["B"].addDependency("A");

        timestamp["D"].addDependency("B");
        timestamp["D"].addDependency("C");
    }

    void A()
    {
        testString.push_back('A');
        wait(0.01);

        timestamp["A"].update();
    }

    void B()
    {
        if (timestamp["B"].dependenciesHaveChanged())
        {
            if (timestamp["B"].dependencyHasChanged("A")) A();

            testString.push_back('B');
            wait(0.01);

            timestamp["B"].update();
        }
    }

    void C()
    {
        testString.push_back('C');
        wait(0.01);

        timestamp["C"].update();
    }

    void D()
    {
        if (timestamp["D"].dependenciesHaveChanged())
        {
            if (timestamp["D"].dependencyHasChanged("B")) B();
            if (timestamp["D"].dependencyHasChanged("C")) C();

            testString.push_back('D');
            wait(0.01);

            timestamp["D"].update();
        }
    }

    const std::vector<char> & getTestString() const
    {
        return testString;
    }

private:

    TRTK::Timestamp timestamp;
    std::vector<char> testString;
};


void unit_test_Timestamp()
{
    HEADING(Timestamp)


        START_TEST

            TimestampTestClass timestampTestClass;

            timestampTestClass.D();
            timestampTestClass.D();
            timestampTestClass.A();
            timestampTestClass.D();
            timestampTestClass.D();

            assert(timestampTestClass.getTestString().size() == 8);
            assert(timestampTestClass.getTestString()[0] == 'A');
            assert(timestampTestClass.getTestString()[1] == 'B');
            assert(timestampTestClass.getTestString()[2] == 'C');
            assert(timestampTestClass.getTestString()[3] == 'D');
            assert(timestampTestClass.getTestString()[4] == 'A');
            assert(timestampTestClass.getTestString()[5] == 'A');
            assert(timestampTestClass.getTestString()[6] == 'B');
            assert(timestampTestClass.getTestString()[7] == 'D');

        STOP_TEST

        START_TEST
        {
            TRTK::Timestamp timestamp;
            timestamp["A"].addDependency("B");
            timestamp["A"].addDependency("C");
            timestamp["D"].addDependency("E");
            timestamp["D"].addDependency("F");
            timestamp["D"].addDependency("G");
            timestamp["G"].addDependency("H");

            timestamp["D"].update();
            wait(0.01);
            timestamp["A"].update();
            assert(timestamp["A"].isMoreRecentThan("D"));
        }
        STOP_TEST

        START_TEST
        {
            TRTK::Timestamp timestamp;
            timestamp["A"].addDependency("B");
            timestamp["A"].addDependency("C");
            timestamp["D"].addDependency("E");
            timestamp["D"].addDependency("F");
            timestamp["D"].addDependency("G");
            timestamp["G"].addDependency("H");

            timestamp["D"].update();
            wait(0.01);
            timestamp["A"].update();
            wait(0.01);
            timestamp["E"].update();
            assert(timestamp["A"].isMoreRecentThan("D", false)); // do not check dependencies
            assert(!timestamp["A"].isMoreRecentThan("D")); // check dependencies
        }
        STOP_TEST
}
