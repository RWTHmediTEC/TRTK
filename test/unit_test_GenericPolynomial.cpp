// Last changed on 2013-06-01.

#include <cmath>
#include <vector>

#include <TRTK/GenericPolynomial.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


using namespace std;
using namespace TRTK;
using namespace TRTK::Tools;

typedef TRTK::GenericPolynomial<double>::Point Point;
typedef TRTK::GenericPolynomial<double>::MatrixXT MatrixXT;


////////////////////////////////////////////////////////////////////////////
//                              Test Classes                              //
////////////////////////////////////////////////////////////////////////////


class TestMP
{
public:

    TestMP(double sigma = 0.0) // noise level
    {
        // Generate two point sets that relate to each other via a
        // four-dimensional bivariate cubic polynomial function f.

        for (unsigned i = 0; i < 100; ++i)
        {
            Point source_point(0, 0);

            source_point.x() = rand(-10.0, 10.0);
            source_point.y() = rand(-10.0, 10.0);

            Point target_point = f(source_point);

            // Add noise.

            Point noise(sigma * randn(), sigma * randn(), sigma * randn(), sigma * randn());
            target_point += noise;

            double weight = 10 * exp(-noise.squaredNorm());
            weights.push_back(weight);

            double weight2 = 1 / (noise.norm() + 1);
            weights2.push_back(weight2);

            source_points.push_back(source_point);
            target_points.push_back(target_point);
        }
    }

    static Point f(const Point & point)
    {
        double x = point.x();
        double y = point.y();

        return Point(-x, x + 20 * y, 5 * x * y, -x * y * y);
    };

public:

    vector<Point> source_points;
    vector<Point> target_points;
    vector<double> weights;
    vector<double> weights2;
};


////////////////////////////////////////////////////////////////////////////
//                               Unit Tests                               //
////////////////////////////////////////////////////////////////////////////

void unit_test_GenericPolynomial()
{
    HEADING(Generic Polynomial)

    GenericPolynomial<double> polynomial(2, 4, 3);

    Point point;
    Point result;

    double residual = 0;
    double rmse = 0;

    MatrixXT coefficients(4, 10);
    coefficients << 0, -1,  0, 0, 0, 0, 0, 0,  0, 0,
                    0,  1, 20, 0, 0, 0, 0, 0,  0, 0,
                    0,  0,  0, 0, 5, 0, 0, 0,  0, 0,
                    0,  0,  0, 0, 0, 0, 0, 0, -1, 0;


    SUBHEADING(Constructor)

        // Don't do anything to polynomial before this section!

        START_TEST
            // Constructor
            point = Point(1, 2);
            result = polynomial * point; // polynomial is the identity function
            residual = (result - (point, 0, 0)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
             // Constructor and assingment operator
            GenericPolynomial<double> polynomial2(coefficients, 2, 4, 3);
            polynomial = polynomial2;
            point = Point(1, 2);
            result = polynomial * point;
            residual = (result - TestMP::f(point)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
            // Copy constructor
            GenericPolynomial<double> polynomial3 = polynomial2;
            point = Point(1, 2);
            result = polynomial3 * point;
            residual = (result - TestMP::f(point)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
            // Templated copy constructor
            GenericPolynomial<float> polynomial4(coefficients.cast<float>(), 2, 4, 3);
            GenericPolynomial<double> polynomial5(polynomial4, 2, 4, 3);
            point = Point(1, 2);
            result = polynomial5 * point;
            residual = (result - TestMP::f(point)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


    SUBHEADING(estimate())

        START_TEST
            TestMP test1;
            rmse = polynomial.estimate(make_iterator(test1.source_points.begin()),
                                       make_iterator(test1.source_points.end()),
                                       make_iterator(test1.target_points.begin()),
                                       make_iterator(test1.target_points.end()));
            assert(abs(rmse - 0.0) < 1e-10);
        STOP_TEST


        START_TEST
            TestMP test2(0.1);
            rmse = polynomial.estimate(make_iterator(test2.source_points.begin()),
                                       make_iterator(test2.source_points.end()),
                                       make_iterator(test2.target_points.begin()),
                                       make_iterator(test2.target_points.end()));
            assert(rmse < 0.2);
        STOP_TEST


        START_TEST
            TestMP test3(0.1);
            rmse = polynomial.estimate(make_range(test3.source_points),
                                       make_range(test3.target_points));
            assert(rmse < 0.2);
        STOP_TEST


    SUBHEADING(transform())

        START_TEST
            polynomial.reset(); // polynomial = id
            point = Point(1, 2);
            result = polynomial.transform(point);
            residual = (result - (point, 0, 0)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
            polynomial.estimate(make_iterator(test1.source_points.begin()),
                                make_iterator(test1.source_points.end()),
                                make_iterator(test1.target_points.begin()),
                                make_iterator(test1.target_points.end()));
            point = Point(1, 2);
            result = polynomial.transform(point); // polynomial = f
            residual = (result - TestMP::f(point)).norm();
            assert(abs(residual - 0.0) < 1e-10);
        STOP_TEST


    SUBHEADING(operator*())

        START_TEST
            polynomial.reset(); // polynomial = id
            point = Point(1, 2);
            result = polynomial * point;
            residual = (result - (point, 0, 0)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
            polynomial.estimate(make_iterator(test1.source_points.begin()),
                                make_iterator(test1.source_points.end()),
                                make_iterator(test1.target_points.begin()),
                                make_iterator(test1.target_points.end()));
            point = Point(1, 2);
            result = polynomial * point; // polynomial = f
            residual = (result - TestMP::f(point)).norm();
            assert(abs(residual - 0.0) < 1e-10);
        STOP_TEST


    SUBHEADING(reset())

        START_TEST
            polynomial.estimate(make_iterator(test1.source_points.begin()),
                                make_iterator(test1.source_points.end()),
                                make_iterator(test1.target_points.begin()),
                                make_iterator(test1.target_points.end()));
            polynomial.reset(); // polynomial = id
            point = Point(1, 2);
            result = polynomial * point;
            residual = (result - (point,0 ,0)).norm();
            assert(abs(residual - 0.0) < 1e-12);
        STOP_TEST


    SUBHEADING(getCoefficients())

        START_TEST
            polynomial.estimate(make_iterator(test1.source_points.begin()),
                                make_iterator(test1.source_points.end()),
                                make_iterator(test1.target_points.begin()),
                                make_iterator(test1.target_points.end()));
            assert(coefficients.isApprox(polynomial.getCoefficients()));
        STOP_TEST


    SUBHEADING(Weigthed Least Squares)

        START_TEST
            // see above: Test test2(0.1)
            rmse = polynomial.estimate(make_iterator(test2.source_points.begin()),
                                       make_iterator(test2.source_points.end()),
                                       make_iterator(test2.target_points.begin()),
                                       make_iterator(test2.target_points.end()));

            vector<double> weights(test2.weights.size(), 1.0);
            double rmse2 = polynomial.estimate(make_iterator(test2.source_points.begin()),
                                               make_iterator(test2.source_points.end()),
                                               make_iterator(test2.target_points.begin()),
                                               make_iterator(test2.target_points.end()),
                                               make_iterator(weights.begin()),
                                               make_iterator(weights.end()));
            assert(isEqual(rmse, rmse2));
        STOP_TEST


        START_TEST
            rmse = polynomial.estimate(make_iterator(test2.source_points.begin()),
                                       make_iterator(test2.source_points.end()),
                                       make_iterator(test2.target_points.begin()),
                                       make_iterator(test2.target_points.end()));

            rmse2 = polynomial.estimate(make_iterator(test2.source_points.begin()),
                                        make_iterator(test2.source_points.end()),
                                        make_iterator(test2.target_points.begin()),
                                        make_iterator(test2.target_points.end()),
                                        make_iterator(test2.weights.begin()),
                                        make_iterator(test2.weights.end()));

            double mean = 0;
            for (unsigned i = 0; i < test2.weights.size(); ++i)
            {
                mean += test2.weights[i] / test2.weights.size();
            }

            assert(rmse2 / sqrt(mean) < rmse);
            assert(coefficients.isApprox(polynomial.getCoefficients(), 0.1));
        STOP_TEST

}
