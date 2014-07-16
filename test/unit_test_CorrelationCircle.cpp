// Last changed on 2011-06-06.


#include <cv.h>
#include <highgui.h>
#include <TRTK/CorrelationCircle.hpp>

#include "unit_test.hpp"


class Counter
{
public:
    Counter() : counter(0) {}
    void increment() {++counter;}
    void increment(int) {++counter;}
    const int & getCounter() const {return counter;}
    void reset() {counter = 0;}

private:
    int counter;
};


void unit_test_CorrelationCircle()
{
    using namespace cv;
    using namespace TRTK;

    // Generate some test data.

    Mat image(256, 256, CV_8U, Scalar(0));

    circle(image, Point(100, 200), 20, 255);
    circle(image, Point(100, 120), 40, 255);
    circle(image, Point( 30,  70), 80, 255);

    const IplImage & img = image;

    Counter counter;


    HEADING(CorrelationCircle)


    SUBHEADING(Constructors)


        START_TEST
            CorrelationCircle correlationCircle1;
            assert(correlationCircle1.getCorrelations().size() == 0);
        STOP_TEST


        START_TEST
            CorrelationCircle correlationCircle2(&img);
            assert(correlationCircle2.getCorrelations().size() == 0);
        STOP_TEST


    SUBHEADING(setImage())


        START_TEST
            correlationCircle1.setImage(&img);
        STOP_TEST


    SUBHEADING(setRadii())


        START_TEST
            correlationCircle1.setRadii(30, 50);
        STOP_TEST


        START_TEST
            correlationCircle2.setRadii(30, 50);
        STOP_TEST


    SUBHEADING(setThickness())


        START_TEST
            correlationCircle1.setThickness(3);
        STOP_TEST


        START_TEST
            correlationCircle2.setThickness(1);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            correlationCircle1.compute();
            correlationCircle1.compute(); // check for reentrancy
        STOP_TEST


        START_TEST
            correlationCircle2.compute();
        STOP_TEST


    SUBHEADING(findSingleCircle())


        START_TEST
            Circle circle1 = correlationCircle1.findSingleCircle();
            assert(std::abs(circle1.x      - 100) < 2);
            assert(std::abs(circle1.y      - 120) < 2);
            assert(std::abs(circle1.radius -  40) < 2);
        STOP_TEST


        START_TEST
            Circle circle2 = correlationCircle2.findSingleCircle();
            assert(std::abs(circle2.x      - 100) < 2);
            assert(std::abs(circle2.y      - 120) < 2);
            assert(std::abs(circle2.radius -  40) < 2);
        STOP_TEST


    SUBHEADING(Real World Data)


        START_TEST
            Mat image2 = imread("./res/ultrasound_table_tennis_ball.jpg");
            const IplImage & img2 = image2;

            CorrelationCircle correlationCircle(&img2);
            correlationCircle.setRadii(100, 110);
            correlationCircle.setThickness(3);
            correlationCircle.compute();

            Circle circle = correlationCircle.findSingleCircle();

            assert(std::abs(circle.x - 462) < 2);
            assert(std::abs(circle.y - 256) < 2);
            assert(std::abs(circle.radius - 104) < 2);
        STOP_TEST


        START_TEST
            counter.reset();

            correlationCircle1.setRadii(30, 50);
            void (Counter::*function)(void) = &Counter::increment;
            correlationCircle1.progress.connect(&counter, function);
            correlationCircle1.compute();

            assert(counter.getCounter() == 21 || counter.getCounter() == 22);
        STOP_TEST


        START_TEST
            counter.reset();
            circle = correlationCircle1.findSingleCircle();
            assert(counter.getCounter() == 21 || counter.getCounter() == 22);
        STOP_TEST
}
