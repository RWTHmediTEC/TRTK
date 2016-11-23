// Last changed on 2011-06-06.


#include <TRTK/CorrelationSphere.hpp>

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


void unit_test_CorrelationSphere()
{
    using namespace TRTK;
    using namespace cimg_library;

    // Generate some test data.

    CImg<unsigned char> * image = new CImg<unsigned char>(256, 256, 1, 3, 0);
    const unsigned char color[] = {255, 255, 255};

    image->draw_circle(100, 200, 20, color, 1, 0);
    image->draw_circle(100, 120, 40, color, 1, 0);
    image->draw_circle(30, 70, 80, color, 1, 0);

    Counter counter;


    HEADING(CorrelationSphere)


    SUBHEADING(Constructors)


        START_TEST
            CorrelationSphere<unsigned char> correlationSphere1;
            assert(correlationSphere1.getCorrelations().size() == 0);
        STOP_TEST


        START_TEST
            CorrelationSphere<unsigned char> correlationSphere2(image);
            assert(correlationSphere2.getCorrelations().size() == 0);
        STOP_TEST


    SUBHEADING(setImage())


        START_TEST
            correlationSphere1.setImage(image);
        STOP_TEST


    SUBHEADING(setRadii())


        START_TEST
            correlationSphere1.setRadii(30, 50);
        STOP_TEST


        START_TEST
            correlationSphere2.setRadii(30, 50);
        STOP_TEST


    SUBHEADING(setThickness())


        START_TEST
            correlationSphere1.setThickness(3);
        STOP_TEST


        START_TEST
            correlationSphere2.setThickness(1);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            correlationSphere1.compute();
            correlationSphere1.compute(); // check for reentrancy
        STOP_TEST


        START_TEST
            correlationSphere2.compute();
        STOP_TEST


    SUBHEADING(findSingleSphere())


        START_TEST
            Sphere sphere1 = correlationSphere1.findSingleSphere();
            assert(std::abs(sphere1.x      - 100) < 2);
            assert(std::abs(sphere1.y      - 120) < 2);
            assert(std::abs(sphere1.radius -  40) < 2);
        STOP_TEST


        START_TEST
            Sphere sphere2 = correlationSphere2.findSingleSphere();
            assert(std::abs(sphere2.x      - 100) < 2);
            assert(std::abs(sphere2.y      - 120) < 2);
            assert(std::abs(sphere2.radius -  40) < 2);
        STOP_TEST


    SUBHEADING(Real World Data)


        START_TEST
            CImg<unsigned char> img2("./res/ultrasound_table_tennis_ball.bmp");

            CorrelationSphere<unsigned char> correlationSphere(&img2);
            correlationSphere.setRadii(70, 110);
            correlationSphere.setThickness(3);
            correlationSphere.compute();

            Sphere sphere = correlationSphere.findSingleSphere();

			assert(std::abs(sphere.x - 491) < 2);
			assert(std::abs(sphere.y - 246) < 2);
			assert(std::abs(sphere.radius - 94) < 2);
        STOP_TEST


        START_TEST
            counter.reset();

            correlationSphere1.setRadii(30, 50);
            void (Counter::*function)(void) = &Counter::increment;
            correlationSphere1.progress.connect(&counter, function);
            correlationSphere1.compute();

            assert(counter.getCounter() == 21 || counter.getCounter() == 22);
        STOP_TEST


        START_TEST
            counter.reset();
            sphere = correlationSphere1.findSingleSphere();
            assert(counter.getCounter() == 21 || counter.getCounter() == 22);
        STOP_TEST
}
