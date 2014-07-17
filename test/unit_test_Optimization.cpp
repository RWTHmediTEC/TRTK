// Last changed on 2012-10-23.

#include <cmath>
#include <vector>

#include <TRTK/Optimization.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


using namespace std;
using namespace TRTK;
using namespace TRTK::Optimization;
using namespace TRTK::Tools;


////////////////////////////////////////////////////////////////////////////
//                      Helper Classes and Functions                      //
////////////////////////////////////////////////////////////////////////////

// f has a zero at sqrt(2)

double f(double x)
{
    return x * x - 2;
}


// f2 has a zero at (x, y) = (2, 0)

Coordinate<double> f2(const Coordinate<double> & arg)
{
    assert(arg.size() == 2);

    const double x = arg.x();
    const double y = arg.y();

    Coordinate<double> result(0, 0);

    result.x() = (x - 2) * (y - 1);
    result.y() = -3 * y;

    return result;
}


// f3 has a zero at (x, y) = (2, 1)

double f3(const Coordinate<double> & arg)
{
    assert(arg.size() == 2);

    const double x = arg.x();
    const double y = arg.y();

    return (x - 2) * (x - 2)  + (y - 1) * (y - 1) - 9;
}


// Rosenbrock function

Coordinate<double> rosenbrock(const Coordinate<double> & arg)
{
    assert(arg.size() == 2);

    const double x = arg.x();
    const double y = arg.y();

    double result = (1 - x) * (1 - x)  + 100 * (y - x * x) * (y - x * x);

    return Coordinate<double>(result);
}


////////////////////////////////////////////////////////////////////////////
//                              Test Classes                              //
////////////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////
//                               Unit Tests                               //
////////////////////////////////////////////////////////////////////////////

void unit_test_Optimization()
{
    HEADING(Optimization Module)

    double result_ = 0;
    double residual = 0;

    Options<double> options;
    Coordinate<double> arg;
    Result<double> result;
    Eigen::MatrixXd matrix;


    SUBHEADING(makeGradient())

        START_TEST
            arg = Coordinate<double>(100, -2);
            result = solveNewtonRaphsonMethod(makeGradient<double>(f3), arg);
            assert(result.error < 1e-9);
            residual = (result.root - Coordinate<double>(2, 1)).norm();
            assert(residual < 1e-9);
        STOP_TEST

        START_TEST
            options = Options<double>();
            options.max_number_iterations = 100;
            options.spacing = 0.001;
            options.error_tolerance = 1e-8;
            arg = Coordinate<double>(100, -2);
            result = solveNewtonRaphsonMethod(makeGradient<double>(f3), arg, options);
            residual = (result.root - Coordinate<double>(2, 1)).norm();
            assert(result.error < 1e-8);
            assert(residual < 1e-8);
            assert(result.number_of_iterations < 10);
        STOP_TEST


    SUBHEADING(jacobian())

        START_TEST
            options = Options<double>();
            arg = Coordinate<double>(5, 5);
            matrix = jacobian(f2, arg);
            assert(abs(matrix(0, 0) -  4) < 1e-8);
            assert(abs(matrix(0, 1) -  3) < 1e-8);
            assert(abs(matrix(1, 0) -  0) < 1e-8);
            assert(abs(matrix(1, 1) - -3) < 1e-8);
        STOP_TEST


    SUBHEADING(solve())

        START_TEST
            arg = Coordinate<double>(100, -2);
            result = solve(f2, arg);
            assert(result.error < 1e-15);
            residual = (result.root - Coordinate<double>(2, 0)).norm();
            assert(residual < 1e-15);
        STOP_TEST


        START_TEST
            options = Options<double>();
            options.algorithm = Options<double>::NEWTON_RAPHSON;
            arg = Coordinate<double>(100, -2);
            result = solve(f2, arg, options);
            assert(result.error < 1e-15);
            residual = (result.root - Coordinate<double>(2, 0)).norm();
            assert(residual < 1e-15);
        STOP_TEST


    SUBHEADING(solve1D())

        START_TEST
            result_ = solve1D(f, 1.0); // returns sqrt(2)
            residual = abs(result_ - sqrt(2.0));
            assert(residual < 1e-15);
        STOP_TEST

        START_TEST
            result_ = solve1D(f, 1.0, 7);
            residual = abs(result_ - sqrt(2.0));
            assert(residual < 1e-15);
        STOP_TEST


    SUBHEADING(solveLevenbergMarquardt())

        START_TEST
            arg = Coordinate<double>(100, -2);
            result = solveLevenbergMarquardt(f2, arg);
            assert(result.error < 1e-15);
            residual = (result.root - Coordinate<double>(2, 0)).norm();
            assert(residual < 1e-15);
        STOP_TEST

        START_TEST
            options = Options<double>();
            options.spacing = 0.1;
            arg = Coordinate<double>(100, -2);
            result = solveLevenbergMarquardt(f2, arg, options);
            assert(result.error < 1e-15);
            residual = (result.root - Coordinate<double>(2, 0)).norm();
            assert(residual < 1e-15);
        STOP_TEST

        START_TEST
            arg = Coordinate<double>(0.5, 2);
            result = solveLevenbergMarquardt(rosenbrock, arg);
            assert(result.error < 0.1);
            residual = (result.root - Coordinate<double>(1, 1)).norm();
            assert(residual < 0.3);
        STOP_TEST


    SUBHEADING(solveNewtonRaphsonMethod())

        START_TEST
            arg = Coordinate<double>(100, -2);
            result = solveNewtonRaphsonMethod(f2, arg);
            assert(result.error < 1e-15);
            residual = (result.root - Coordinate<double>(2, 0)).norm();
            assert(residual < 1e-15);
        STOP_TEST

        START_TEST
            options = Options<double>();
            options.spacing = 0.1;
            arg = Coordinate<double>(100, -2);
            result = solveNewtonRaphsonMethod(f2, arg, options);
            assert(result.error < 1e-15);
            residual = (result.root - Coordinate<double>(2, 0)).norm();
            assert(residual < 1e-15);
        STOP_TEST

        START_TEST
            options = Options<double>();
            options.error_tolerance = 0.01;
            arg = Coordinate<double>(100, -2);
            result = solveNewtonRaphsonMethod(f2, arg, options);
            assert(result.error < 0.01);
            residual = (result.root - Coordinate<double>(2, 0)).norm();
            assert(residual < 0.01);
        STOP_TEST

        START_TEST
            options = Options<double>();
            options.max_number_iterations = 7;
            arg = Coordinate<double>(100, -2);
            result = solveNewtonRaphsonMethod(f2, arg, options);
            assert(result.error < 1e-10);
            residual = (result.root - Coordinate<double>(2, 0)).norm();
            assert(residual < 1e-10);
        STOP_TEST
}
