// Last changed on 2012-08-06.

#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS

#include <cmath>
#include <fstream>
#include <vector>

#include <TRTK/EstimateRigidTransformation3D.hpp>
#include <TRTK/EstimateSimilarityTransformation3D.hpp>
#include <TRTK/Icp.hpp>
#include <TRTK/Transform3D.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


using namespace std;
using namespace TRTK;
using namespace TRTK::Tools;


////////////////////////////////////////////////////////////////////////////
//                             Helper Classes                             //
////////////////////////////////////////////////////////////////////////////

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

    void incCounter(int)
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


////////////////////////////////////////////////////////////////////////////
//                              Test Classes                              //
////////////////////////////////////////////////////////////////////////////

class Test3D
{
public:
    typedef Coordinate<double> Point;
    typedef vector<Point> Points;

    const Points & getSourcePoints() const
    {
        return source_points;
    }

    const Points & getTargetPoints() const
    {
        return target_points;
    }

    const Transform3D<double> & getTransform() const
    {
        return transform;
    }

    double getRMS()
    {
        double rms = 0;
        unsigned N = source_points.size();

        for (unsigned i = 0; i < N; ++i)
        {
            rms += (source_points[i] - target_points[i]).squaredNorm();
        }

        return sqrt(rms / N);
    }

protected:
    Points source_points;
    Points target_points;
    Transform3D<double> transform;
};


class Test3D_1 : public Test3D
{
public:
    Test3D_1(double sigma = 0.0)
    {
        transform.rotateZ(10.0 / 180.0 * 3.1416).translate(0.1, 0.2, 0.3);

        source_points.push_back(Point(0, 0, 0));
        source_points.push_back(Point(1, 0, 0));
        source_points.push_back(Point(1, 1, 0));
        source_points.push_back(Point(0, 1, 0));

        for (unsigned i = 0; i < source_points.size(); ++i)
        {
            Point target_point = transform * source_points[i] + sigma * Point(randn(), randn(), randn());
            target_points.push_back(target_point);
        }
    }
};


class Test3D_2 : public Test3D
{
public:
    Test3D_2(double sigma = 0.0)
    {
        // Generate points lying on the surface of a hyperboloid.

        /* GNU Octave code for the graphs in the documentation:
         * N = 500;
         * s =2 *  rand(1, N) - 1;
         * r =2 * pi *  rand(1, N);
         * x = sqrt(s.^2 + 0.2) .* cos(r);
         * y = sqrt(s.^2 + 0.2) .* sin(r);
         * z = s;
         * plot3(x, y, z, 'o');
         */

        double pi = 3.1416;

        unsigned number_source_points = 1000;
        unsigned number_target_points = 1000;

        source_points.reserve(number_source_points);
        target_points.reserve(number_target_points);

        for (unsigned i = 0; i < number_source_points; ++i)
        {
            double s = rand<double>(-1.0, 1.0);
            double t = rand<double>() * 2 * pi;
            double x = sqrt(s * s + 0.2) * cos(t);
            double y = sqrt(s * s + 0.2) * sin(t);
            double z = s;

            source_points.push_back(Point(x, y, z));
        }

        transform.rotateAxis(10.0 / 180.0 * pi, Point(1, 1, 10)).translate(0.1, 0.2, 0.3);

        for (unsigned i = 0; i < number_target_points; ++i)
        {
            double s = rand<double>(-1.0, 1.0);
            double t = rand<double>() * 2 * pi;
            double x = sqrt(s * s + 0.2) * cos(t);
            double y = sqrt(s * s + 0.2) * sin(t);
            double z = s;

            Point target_point = transform * Point(x, y, z);
            target_point += sigma * Point(randn(), randn(), randn());

            target_points.push_back(target_point);
        }
    }
};


class Test3D_3 : public Test3D
{
public:
    Test3D_3()
    {
        // Load the model and data points (anatomic landmarks) to perform a pre-registration.

        ifstream file_stream("./res/ICP_hip_pre-registration_model_points.txt");

        while (file_stream)
        {
            double x, y, z;
            file_stream >> x >> y >> z;
            target_points.push_back(Point(x, y, z));
        }

        file_stream.close();

        file_stream.open("./res/ICP_hip_pre-registration_data_points.txt");

        while (file_stream)
        {
            double x, y, z;
            file_stream >> x >> y >> z;
            source_points.push_back(Point(x, y, z));
        }

        file_stream.close();

        // Perform the pre-registration.

        EstimateSimilarityTransformation3D<double> estimator(source_points, target_points);
        estimator.compute();

        transform = Transform3D<double>(estimator.getTransformationMatrix());

        // Load the model and data points.

        file_stream.open("./res/ICP_hip_model_points.txt");

        target_points.clear();

        unsigned number_of_points = unsigned(fileLength(file_stream) / 22.0);

        target_points.reserve(number_of_points);

        while (file_stream)
        {
            double x, y, z;
            file_stream >> x >> y >> z;
            target_points.push_back(Point(x, y, z));
        }

        file_stream.close();

        file_stream.open("./res/ICP_hip_data_points.txt");

        source_points.clear();

        while (file_stream)
        {
            double x, y, z;
            file_stream >> x >> y >> z;
            source_points.push_back(Point(x, y, z));
        }

        file_stream.close();
    }
};


////////////////////////////////////////////////////////////////////////////
//                               Unit Tests                               //
////////////////////////////////////////////////////////////////////////////

void unit_test_Icp()
{
    HEADING(Iterative Closest Point)

    Counter counter;


    SUBHEADING(Icp3D)

    Icp3D<double> icp3d;
    void (Counter::*fp_incCounter)(void) = &Counter::incCounter;
    icp3d.progress.connect(&counter, fp_incCounter);

    // EstimateRigidTransformation3D<double> estimator;
    EstimateSimilarityTransformation3D<double> estimator;

    double rmse = 0.0;
    double residual = 0.0;

        START_TEST

            Test3D_1 test3d_1a;

            icp3d.setSourcePoints(test3d_1a.getSourcePoints());
            icp3d.setTargetPoints(test3d_1a.getTargetPoints());
            icp3d.setEstimationAlgorithm(estimator);
            icp3d.setMaximumNumberIterations(20);
            icp3d.setMaximumRMSE(1e-14);

            rmse = icp3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_1a.getTransform().getTransformationMatrix()).norm();
            assert(residual < 1e-15);
            assert(counter.getCounter() == 2);
        STOP_TEST

        START_TEST
            counter.reset();

			Test3D_1 test3d_1b(0.01);

            icp3d.setSourcePoints(test3d_1b.getSourcePoints());
            icp3d.setTargetPoints(test3d_1b.getTargetPoints());
            icp3d.setInitialEstimation(Eigen::Matrix4d::Identity());
            icp3d.setEstimationAlgorithm(estimator);
            icp3d.setMaximumNumberIterations(10);
            icp3d.setMaximumRMSE(1e-3);

            rmse = icp3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_1b.getTransform().getTransformationMatrix()).norm();
            assert(residual < 0.05);
            assert(counter.getCounter() == 10);
        STOP_TEST

        START_TEST
            counter.reset();

            Test3D_2 test3d_2a(0.01);

            icp3d.setSourcePoints(test3d_2a.getSourcePoints());
            icp3d.setTargetPoints(test3d_2a.getTargetPoints());
            icp3d.setInitialEstimation(Eigen::Matrix4d::Identity());
            icp3d.setEstimationAlgorithm(estimator);
            icp3d.setMaximumNumberIterations(10);
            icp3d.setMaximumRMSE(0.07);

            rmse = icp3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_2a.getTransform().getTransformationMatrix()).norm();
            assert(residual < 0.5);
            assert(counter.getCounter() < 10);
        STOP_TEST


	SUBHEADING(IcpTrimmed3D)

	IcpTrimmed3D<double> icpTrimmed3d;
    icpTrimmed3d.progress.connect(&counter, fp_incCounter);

		START_TEST
            counter.reset();

			Test3D_1 test3d_1a2;

			icpTrimmed3d.setSourcePoints(test3d_1a2.getSourcePoints());
            icpTrimmed3d.setTargetPoints(test3d_1a2.getTargetPoints());
            icpTrimmed3d.setEstimationAlgorithm(estimator);
			icpTrimmed3d.setMaximumNumberIterations(20);
            icpTrimmed3d.setMaximumRMSE(1e-14);
			icpTrimmed3d.setMinimumOverlap(1);

            rmse = icpTrimmed3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_1a2.getTransform().getTransformationMatrix()).norm();
            assert(residual < 1e-15);
            assert(counter.getCounter() <= 4);
        STOP_TEST

		START_TEST
            counter.reset();

            Test3D_1 test3d_1b2(0.01);

            icpTrimmed3d.setSourcePoints(test3d_1b2.getSourcePoints());
            icpTrimmed3d.setTargetPoints(test3d_1b2.getTargetPoints());
            icpTrimmed3d.setInitialEstimation(Eigen::Matrix4d::Identity());
            icpTrimmed3d.setEstimationAlgorithm(estimator);
            icpTrimmed3d.setMaximumNumberIterations(10);
            icpTrimmed3d.setMaximumRMSE(1e-3);
			icpTrimmed3d.setMinimumOverlap(1);

            rmse = icpTrimmed3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_1b2.getTransform().getTransformationMatrix()).norm();
            assert(residual < 0.05);
            assert(counter.getCounter() == 10);
        STOP_TEST

		START_TEST
            counter.reset();

            Test3D_2 test3d_2a2(0.01);

            icpTrimmed3d.setSourcePoints(test3d_2a2.getSourcePoints());
            icpTrimmed3d.setTargetPoints(test3d_2a2.getTargetPoints());
            icpTrimmed3d.setInitialEstimation(Eigen::Matrix4d::Identity());
            icpTrimmed3d.setEstimationAlgorithm(estimator);
            icpTrimmed3d.setMaximumNumberIterations(10);
            icpTrimmed3d.setMaximumRMSE(0.07);
			icpTrimmed3d.setMinimumOverlap(1);

            rmse = icpTrimmed3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_2a2.getTransform().getTransformationMatrix()).norm();
            assert(residual < 0.5);
            assert(counter.getCounter() < 10);
        STOP_TEST

    SUBHEADING(RandomSampleIcp3D)

    RandomSampleIcp3D<double> rsicp3d;
    rsicp3d.progress.connect(&counter, fp_incCounter);

    // EstimateRigidTransformation3D<double> estimator; // see above

        START_TEST
            counter.reset();

            Test3D_1 test3d_1c;

            rsicp3d.setSourcePoints(test3d_1c.getSourcePoints());
            rsicp3d.setTargetPoints(test3d_1c.getTargetPoints());
            rsicp3d.setEstimationAlgorithm(estimator);
            rsicp3d.setMaximumNumberIterationsFirstStage(20);
            rsicp3d.setMaximumNumberIterationsSecondStage(20);
            rsicp3d.setPercentage(100);
            rsicp3d.setMaximumRMSE(1e-14);

            rmse = rsicp3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_1c.getTransform().getTransformationMatrix()).norm();
            assert(residual < 1e-15);
            assert(counter.getCounter() <= 4);
        STOP_TEST

        START_TEST
            counter.reset();

            Test3D_1 test3d_1d(0.01);

            rsicp3d.setSourcePoints(test3d_1d.getSourcePoints());
            rsicp3d.setTargetPoints(test3d_1d.getTargetPoints());
            rsicp3d.setInitialEstimation(Eigen::Matrix4d::Identity());
            rsicp3d.setEstimationAlgorithm(estimator);
            rsicp3d.setMaximumNumberIterationsFirstStage(10);
            rsicp3d.setMaximumNumberIterationsSecondStage(10);
            rsicp3d.setPercentage(100);
            rsicp3d.setMaximumRMSE(1e-3);

            rmse = rsicp3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_1d.getTransform().getTransformationMatrix()).norm();
            assert(residual < 0.075);
            assert(counter.getCounter() == 20);
        STOP_TEST

        START_TEST
            counter.reset();

            Test3D_2 test3d_2b(0.01);

            rsicp3d.setSourcePoints(test3d_2b.getSourcePoints());
            rsicp3d.setTargetPoints(test3d_2b.getTargetPoints());
            rsicp3d.setInitialEstimation(Eigen::Matrix4d::Identity());
            rsicp3d.setEstimationAlgorithm(estimator);
            rsicp3d.setMaximumNumberIterationsFirstStage(20);
            rsicp3d.setMaximumNumberIterationsSecondStage(20);
            rsicp3d.setPercentage(50);
            rsicp3d.setMaximumRMSE(0.07);

            rmse = rsicp3d.compute();

            residual = (estimator.getTransformationMatrix() - test3d_2b.getTransform().getTransformationMatrix()).norm();
            assert(residual < 0.5);
            assert(counter.getCounter() < 10);
        STOP_TEST

        START_TEST
            counter.reset();

            cout << "[loading the data set...] ";
            Test3D_3 test3d_3a;

            rsicp3d.setSourcePoints(test3d_3a.getSourcePoints());
            rsicp3d.setTargetPoints(test3d_3a.getTargetPoints());
            rsicp3d.setInitialEstimation(test3d_3a.getTransform().getTransformationMatrix());
            rsicp3d.setEstimationAlgorithm(estimator);
            rsicp3d.setMaximumNumberIterationsFirstStage(20);
            rsicp3d.setMaximumNumberIterationsSecondStage(20);
            rsicp3d.setPercentage(50);
            rsicp3d.setMaximumRMSE(0.1);

            cout << "[performing the computation...] ";
            rmse = rsicp3d.compute();

            assert(rmse < 1.0);
            assert(counter.getCounter() < 50);
        STOP_TEST
}
