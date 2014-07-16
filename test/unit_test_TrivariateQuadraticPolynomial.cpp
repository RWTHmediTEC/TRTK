// Last changed on 2013-03-26.

#include <cmath>
#include <vector>

#include <TRTK/TrivariateQuadraticPolynomial.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


using namespace std;
using namespace TRTK;
using namespace TRTK::Tools;

typedef TRTK::TrivariateQuadraticPolynomial<double>::Point Point;
typedef TRTK::TrivariateQuadraticPolynomial<double>::MatrixXT MatrixXT;


////////////////////////////////////////////////////////////////////////////
//                              Test Classes                              //
////////////////////////////////////////////////////////////////////////////


class TestTQP
{
public:

    TestTQP(double sigma = 0.0) // noise level
    {
        // Generate two point sets that relate to each other via the
        // trivariate quadratic polynomial function f.

        for (unsigned i = 0; i < 100; ++i)
        {
            Point source_point(0, 0, 0);

            source_point.x() = rand(-10.0, 10.0);
            source_point.y() = rand(-10.0, 10.0);
            source_point.z() = rand(-10.0, 10.0);

            Point target_point = f(source_point);

            // Add noise.

            Point noise(sigma * randn(), sigma * randn(), sigma * randn());
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
        double z = point.z();

        return Point(-x, x + 20 * y, z + 5 * x * z);
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

void unit_test_TrivariateQuadraticPolynomial()
{
    HEADING(Trivariate Quadratic Polynomial)

    TrivariateQuadraticPolynomial<double> polynomial;

    Point point;
    Point result;

    double residual = 0;
    double rmse = 0;

    MatrixXT coefficients(3, 10);
    coefficients << 0, -1,  0, 0, 0, 0, 0, 0, 0, 0,
                    0,  1, 20, 0, 0, 0, 0, 0, 0, 0,
                    0,  0,  0, 1, 0, 0, 5, 0, 0, 0;


    SUBHEADING(Constructor)

        // Don't do anything to polynomial before this section!

        START_TEST
            // Default constructor
            point = Point(1, 2, 3);
            result = polynomial * point;
            residual = (result - point).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
             // Constructor and assingment operator
            TrivariateQuadraticPolynomial<double> polynomial2(coefficients);
            polynomial = polynomial2;
            point = Point(1, 2, 3);
            result = polynomial * point;
            residual = (result - TestTQP::f(point)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
            // Copy constructor
            TrivariateQuadraticPolynomial<double> polynomial3 = polynomial2;
            point = Point(1, 2, 3);
            result = polynomial3 * point;
            residual = (result - TestTQP::f(point)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
            // Templated copy constructor
            TrivariateQuadraticPolynomial<float> polynomial4(coefficients.cast<float>());
            TrivariateQuadraticPolynomial<double> polynomial5 = polynomial4;
            point = Point(1, 2, 3);
            result = polynomial5 * point;
            residual = (result - TestTQP::f(point)).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


    SUBHEADING(estimate())

        START_TEST
            TestTQP test1;
            rmse = polynomial.estimate(make_iterator(test1.source_points.begin()),
                                       make_iterator(test1.source_points.end()),
                                       make_iterator(test1.target_points.begin()),
                                       make_iterator(test1.target_points.end()));
            assert(abs(rmse - 0.0) < 1e-12);
        STOP_TEST


        START_TEST
            TestTQP test2(0.1);
            rmse = polynomial.estimate(make_iterator(test2.source_points.begin()),
                                       make_iterator(test2.source_points.end()),
                                       make_iterator(test2.target_points.begin()),
                                       make_iterator(test2.target_points.end()));
            assert(rmse < 0.2);
        STOP_TEST


        START_TEST
            TestTQP test3(0.1);
            rmse = polynomial.estimate(make_range(test3.source_points),
                                       make_range(test3.target_points));
            assert(rmse < 0.2);
        STOP_TEST


    SUBHEADING(transform())

        START_TEST
            polynomial.reset(); // polynomial = id
            point = Point(1, 2, 3);
            result = polynomial.transform(point);
            residual = (result - point).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
            polynomial.estimate(make_iterator(test1.source_points.begin()),
                                make_iterator(test1.source_points.end()),
                                make_iterator(test1.target_points.begin()),
                                make_iterator(test1.target_points.end()));
            point = Point(1, 2, 3);
            result = polynomial.transform(point); // polynomial = f
            residual = (result - TestTQP::f(point)).norm();
            assert(abs(residual - 0.0) < 1e-12);
        STOP_TEST


    SUBHEADING(operator*())

        START_TEST
            polynomial.reset(); // polynomial = id
            point = Point(1, 2, 3);
            result = polynomial * point;
            residual = (result - point).norm();
            assert(isEqual(residual, 0.0));
        STOP_TEST


        START_TEST
            polynomial.estimate(make_iterator(test1.source_points.begin()),
                                make_iterator(test1.source_points.end()),
                                make_iterator(test1.target_points.begin()),
                                make_iterator(test1.target_points.end()));
            point = Point(1, 2, 3);
            result = polynomial * point; // polynomial = f
            residual = (result - TestTQP::f(point)).norm();
            assert(abs(residual - 0.0) < 1e-12);
        STOP_TEST


    SUBHEADING(reset())

        START_TEST
            polynomial.estimate(make_iterator(test1.source_points.begin()),
                                make_iterator(test1.source_points.end()),
                                make_iterator(test1.target_points.begin()),
                                make_iterator(test1.target_points.end()));
            polynomial.reset(); // polynomial = id
            point = Point(1, 2, 3);
            result = polynomial * point;
            residual = (result - point).norm();
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
