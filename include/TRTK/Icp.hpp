/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.3.0 (2014-07-04)
*/

/** \file Icp.hpp
  * \brief This file contains the \ref TRTK::Icp "ICP" class.
  */

#ifndef ICP_HPP_7908314317
#define ICP_HPP_7908314317


#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <flann/flann.hpp>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "EstimateTransformation3D.hpp"
#include "Signals.hpp"
#include "Tools.hpp"
#include "Transform2D.hpp"
#include "Transform3D.hpp"


namespace TRTK
{


////////////////////////////////////////////////////////////////////////
//                            IcpInterface                            //
////////////////////////////////////////////////////////////////////////


/** \tparam ValueType   Scalar type (must be a floating point type).
  *
  * \brief Iterative Closest Point.
  *
  * The iterative closest point (ICP) algorithm allows solving a registration
  * problem where some data points (source points) shall be geometrically
  * aligned with a given model. The model is also given by a set of points
  * (target points).
  *
  * \image html ICP.png "Left: Model points (blue) and unaligned data points (red). Right: Registered with the ICP algorithm."
  *
  * This header file contains several variants of the ICP algorithm, first
  * and foremost the original algorithm of Chen and Medioni [1] or Besl and
  * McKay [2] for 2D and 3D point clouds.
  *
  * The general concept of the ICP algorithm is always as follows:
  *
  * Given some model and data points and an initial transformation between both
  * point sets (from the data coordinate system to the model coordinate system)
  *
  *  - (1) For each point in the data set search for a corresponding point in
  *        the model set by using the current transformation estimate (this
  *        can be done by transforming all source points and performing a simple
  *        nearest neighbor search).
  *  - (2) Estimate a new transformation between both sets from the above
  *        correspondencies.
  *  - (3) Iterate until convergence is achieved or until some other stop
  *        criterion is met.
  *
  * Internally the query for point correspondencies is done using an approximate
  * nearest neighbor search which can significantly improve the query time. By
  * default no approximation is done (eps-approximate of zero). To do an approximate
  * search, set the eps value to a value greater than zero using the function
  * setEpsApproximate(). Note, the search structure is only built up from the set
  * of target points which is assumed to be bigger than the set of data points.
  *
  * The transformation estimator can be an arbitrary estimator. See
  * EstimateTransformation and its subclasses for more details.
  *
  * The current state of computation is provided by the signal \c progress.
  * The computation can be canceled at each point in time by calling the
  * function abortComputation().
  *
  * \b Example:
  *
  * In the following example, the point clouds seen in the figure above are
  * registered. Each of the clouds consists of points lying on the surface of a
  * hyperboloid. The blue points are the undistorted model points, the red ones
  * are the noisy data points which shall be aligned with the model points.
  *
  * First, we will construct these two point sets. Then we will set an initial
  * transformation estimation and finally we will run the ICP algorithm and
  * put out the results.
  *
  * \code
  * #include <iostream>
  *
  * #include <TRTK/Icp.hpp>
  * #include <TRTK/EstimateRigidTransformation3D.hpp>
  * #include <TRTK/Tools.hpp>
  * #include <TRTK/Transform3D.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
  *
  * void printProgress(unsigned value)
  * {
  *     cout << "Progress: " << value << endl;
  * }
  *
  * int main()
  * {
  *     typedef Coordinate<double> Point;
  *     typedef vector<Point> Points;
  *     typedef TRTK::Icp3D<double> ICP;
  *
  *     double pi = 3.1416;
  *
  *     // Generate some model points.
  *
  *     Points target_points;
  *     unsigned number_target_points = 1000;
  *     target_points.reserve(number_target_points);
  *
  *     for (unsigned i = 0; i < number_target_points; ++i)
  *     {
  *         // points lying on the surface of a hyperboloid
  *
  *         double s = rand<double>(-1.0, 1.0);
  *         double t = rand<double>() * 2 * pi;
  *         double x = sqrt(s * s + 0.2) * cos(t);
  *         double y = sqrt(s * s + 0.2) * sin(t);
  *         double z = s;
  *
  *         target_points.push_back(Point(x, y, z));
  *     }
  *
  *     // Generate some noisy data points. These points shall also be unaligned
  *     // with the model points.
  *
  *     Transform3D<double> transform;
  *     transform.rotateAxis(10.0 / 180.0 * pi, Point(1, 1, 10)).translate(0.1, 0.2, 0.3);
  *
  *     Points source_points;
  *     unsigned number_source_points = 1000;
  *     source_points.reserve(number_source_points);
  *
  *     for (unsigned i = 0; i < number_source_points; ++i)
  *     {
  *         double s = rand<double>(-1.0, 1.0);
  *         double t = rand<double>() * 2 * pi;
  *         double x = sqrt(s * s + 0.2) * cos(t);
  *         double y = sqrt(s * s + 0.2) * sin(t);
  *         double z = s;
  *
  *         Point source_point = transform * Point(x, y, z); // displace
  *         source_point += sigma * Point(randn(), randn(), randn()); // add noise
  *
  *         source_points.push_back(source_point);
  *     }
  *
  *     // Set an initial transformation estimation (pre-registration).
  *
  *     ICP::Matrix initial_transformation = Eigen::Matrix4d::Identity();
  *
  *     // Register the two point sets using a rigid transformation.
  *
  *     EstimateRigidTransformation3D<double> estimator;
  *
  *     ICP icp;
  *
  *     icp.setSourcePoints(source_points);
  *     icp.setTargetPoints(target_points);
  *     icp.setInitialEstimation(initial_transformation); // by default identity
  *     icp.setEstimationAlgorithm(estimator);
  *     icp.setMaximumNumberIterations(10);
  *     icp.setMaximumRMSE(0.07);
  *
  *     icp.progress.connect(&printProgress);
  *
  *     cout << "Start computation.\n";
  *
  *     double rmse = icp.compute();
  *
  *     cout << endl << "The RMSE is " << rmse << endl << "and the sought transformation is:"
  *          << endl << icp.getTransformationMatrix();
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * ...
  * \endcode
  *
  * \see IcpBase
  *
  * \references
  *
  * [1] Yang Chen and Gerard Medioni, "Object Modelling by Registration of Multiple
  *     Range Images", Image and Vision Computing, 1991
  *
  * [2] Paul J. Besl and Neil D. McKay, "A Method for Registration of 3-D Shapes",
  *     IEEE Transactions on Pattern Analysis and Machine Intelligence, 1992
  *
  * [3] Rusinkiewicz and Levoy, "Efficient Variants of the ICP Algorithm",
  *     International Conference on 3-D Digital Imaging and Modeling, 2001
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2012-08-07
  */

template <class ValueType>
class IcpInterface
{
public:
    typedef Eigen::Matrix<ValueType, Eigen::Dynamic, Eigen::Dynamic> Matrix;

    virtual ~IcpInterface() = 0;

    virtual void setSourcePoints(const std::vector<Coordinate<ValueType> > & source_points) = 0;    //!< Set the data points as decribed above.
    virtual void setTargetPoints(const std::vector<Coordinate<ValueType> > & target_points) = 0;    //!< Set the model points as decribed above.
    virtual void setInitialEstimation(const Matrix & transformation) = 0;                           //!< Set the initial transformation from the source to the target point coordinate system.
    virtual void setEstimationAlgorithm(EstimateTransformation3D<ValueType> & estimator) = 0;       //!< Set the transformation estimator.
    virtual void setMaximumNumberIterations(unsigned number) = 0;                                   //!< Set the maximum number of iterations performed by an algorithm (default is 20).
    virtual void setMaximumRMSE(ValueType value) = 0;                                               //!< Set an upper bound of the root mean square error; an algorithm terminates if the error of the current estimation falls below this value (default is 0).
    virtual void setEpsApproximate(float value) = 0;                                                //!< Set the value for the eps-approximate neighbor search. The higher the value the faster the search (but also the less accurate) (default is 0).

    virtual ValueType compute() = 0;                                                                //!< Call this function to perform the transformation estimation. The RMSE is returned.

    virtual void abortComputation() = 0;                                                            //!< Call this function to abort a running algorithm.

    virtual const Matrix & getTransformation() = 0;                                                 //!< Returns the estimated transformation in the form of a homogeneous matrix.
};


template<class ValueType>
IcpInterface<ValueType>::~IcpInterface()
{
}


////////////////////////////////////////////////////////////////////////
//                              IcpBase                               //
////////////////////////////////////////////////////////////////////////


/** \relates IcpInterface
  *
  * \tparam ValueType   Scalar type (must be a floating point type).
  *
  * \brief Base class for all ICP implementations.
  *
  * For a more detailed description, please have a look at the \ref IcpInterface "interface class".
  *
  * \see IcpInterface
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2012-08-07
  */

template<class ValueType>
class IcpBase : public IcpInterface<ValueType>
{
public:
    typedef IcpInterface<ValueType> super;
    typedef typename super::Matrix Matrix;

    IcpBase(unsigned dimension);
    ~IcpBase();

    void setSourcePoints(const std::vector<Coordinate<ValueType> > & source_points);    //!< Set the data points as decribed \ref IcpInterface "here".
    void setTargetPoints(const std::vector<Coordinate<ValueType> > & target_points);    //!< Set the model points as decribed \ref IcpInterface "here".
    void setInitialEstimation(const Matrix & transformation);
    void setEstimationAlgorithm(EstimateTransformation3D<ValueType> & estimator);
    void setMaximumNumberIterations(unsigned number);
    void setMaximumRMSE(ValueType value);
    void setEpsApproximate(float value);

    virtual ValueType compute() = 0; // returns the RMSE

    void abortComputation();

    const Matrix & getTransformation();

    Signal<unsigned> progress;  //!< Provides the current state of computation (from 0 to 100).

protected:
    unsigned dimension;                                         //!< Dimension of the source and target points.

    const std::vector<Coordinate<ValueType> > * source_points;  //!< Reference to the data points given by the user. An algorithm might store a local copy which is transformed during the registration procedure.

    flann::Matrix<ValueType> target_points;                     //!< Internal copy of the model points given by the user.
    flann::Matrix<ValueType> query_point;                       //!< Query point used to search the kd-tree.
    flann::Matrix<ValueType> distances;                         //!< Distance to the nearest neighbor obtained by a nearest neighbor search.
    flann::Matrix<int> indices;                                 //!< Index of the nearest neighbor in \ref target_points obtained by a nearest neighbor search.
    flann::Index<flann::L2<ValueType> > * tree;                 //!< Data structure of the search index (kd-tree).
    float eps;                                                  //!< Eps-approximate value for the nearest neighbor search.

    EstimateTransformation3D<ValueType> * estimator;            //!< Reference to the user-provided estimator.

    unsigned maximum_number_iterations;                         //!< Maximum possible number of iterations during the computation.

    ValueType maximum_rms_error;                                //!< Iteration stop criterion. If the current RMSE falls below this bound, an algorithm might terminate.

    Matrix transformation;                                      //!< Internally stored (homogeneous) transformation.

    volatile bool abort;                                        //!< This flag must be checked during iterations; can be set by other threads through abortComputation().
};


template<class ValueType>
IcpBase<ValueType>::IcpBase(unsigned dimension) :
    dimension(dimension),
    tree(NULL),
    eps(0),
    estimator(NULL),
    maximum_number_iterations(20),
    maximum_rms_error(0),
    abort(false)
{
}


template<class ValueType>
IcpBase<ValueType>::~IcpBase()
{
    if (tree != NULL) delete tree;

    delete[] target_points.ptr();
    delete[] query_point.ptr();
    delete[] distances.ptr();
    delete[] indices.ptr();
}


template<class ValueType>
void IcpBase<ValueType>::abortComputation()
{
    abort = true;
}


template<class ValueType>
inline const typename IcpBase<ValueType>::Matrix & IcpBase<ValueType>::getTransformation()
{
    return transformation;
}


template<class ValueType>
void IcpBase<ValueType>::setEpsApproximate(float value)
{
    assert(eps >= 0);
    eps = value;
}


template<class ValueType>
void IcpBase<ValueType>::setEstimationAlgorithm(EstimateTransformation3D<ValueType> & estimator)
{
    this->estimator = &estimator;
}


template<class ValueType>
void IcpBase<ValueType>::setInitialEstimation(const Matrix & transformation)
{
    this->transformation = transformation;
}


template<class ValueType>
void IcpBase<ValueType>::setMaximumNumberIterations(unsigned number)
{
    maximum_number_iterations = number;
}


template<class ValueType>
void IcpBase<ValueType>::setMaximumRMSE(ValueType value)
{
    maximum_rms_error = value;
}


/** Only a reference is stored. */

template<class ValueType>
void IcpBase<ValueType>::setSourcePoints(const std::vector<Coordinate<ValueType> > & source_points)
{
    this->source_points = &source_points;
}


/** The targe points are copied.  */

template<class ValueType>
void IcpBase<ValueType>::setTargetPoints(const std::vector<Coordinate<ValueType> > & target_points)
{
    if (target_points.size() == 0)
    {
        throw std::runtime_error("IcpBase::setTargetPoints(): The input data may not be emtpy.");
    }

    // Copy the target points.

    const unsigned number_points = target_points.size();

    delete[] this->target_points.ptr();
    this->target_points = flann::Matrix<ValueType>(new ValueType[number_points * dimension], number_points, dimension);

    for (unsigned i = 0; i < number_points; ++i)
    {
        for (unsigned j = 0; j < dimension; ++j)
        {
            this->target_points[i][j] = target_points[i][j];
        }
    }

    // Setup tree.

    if (tree != NULL) delete tree;

    tree = new flann::Index<flann::L2<ValueType> >(this->target_points, flann::KDTreeSingleIndexParams());
    tree->buildIndex();
}


////////////////////////////////////////////////////////////////////////
//                               Icp3D                                //
////////////////////////////////////////////////////////////////////////


/** \relates IcpInterface
  *
  * \tparam ValueType   Scalar type (must be a floating point type).
  *
  * \brief ICP algorithm for 3D point clouds as described by Besl and McKay [1].
  *
  * For a more detailed description and how to use this class, please have a
  * look at the \ref IcpInterface "interface class".
  *
  * \see IcpInterface and IcpBase
  *
  * \references
  *
  * [1] Paul J. Besl and Neil D. McKay, "A Method for Registration of 3-D Shapes",
  * IEEE Transactions on Pattern Analysis and Machine Intelligence, 1992
  *
  * \author Christoph Haenisch
  * \version 0.1.1
  * \date last changed on 2012-08-07
  */

template <class ValueType>
class Icp3D : public IcpBase<ValueType>
{
public:
    Icp3D();
    ValueType compute();

private:
    typedef IcpBase<ValueType> super;

    using super::abort;
    using super::distances;
    using super::estimator;
    using super::indices;
    using super::maximum_number_iterations;
    using super::maximum_rms_error;
    using super::query_point;
    using super::source_points;
    using super::target_points;
    using super::transformation;
    using super::tree;
};


template<class ValueType>
Icp3D<ValueType>::Icp3D() : IcpBase<ValueType>(3)
{
    transformation = Transform3D<ValueType>().getTransformationMatrix();

    distances = flann::Matrix<ValueType>(new ValueType[1], 1, 1);
    indices = flann::Matrix<int>(new int[1], 1, 1);
    query_point = flann::Matrix<ValueType>(new ValueType[3], 1, 3);
}


/** \exception range_error A range error exception is thrown if no model points are given.
  *
  * \note The given estimator might throw an exception as well.
  */

template<class ValueType>
ValueType Icp3D<ValueType>::compute()
{
    if (tree->size() == 0)
    {
        throw std::range_error("Icp3D::compute(): No target points are given.");
    }

    // Initialize some variables.

    using std::ceil;
    using std::max;
    using std::min;

    abort = false;

    const unsigned size = source_points->size();

    std::vector<Coordinate<ValueType> > transformed_source_points;
    std::vector<Coordinate<ValueType> > target_points;

    Coordinate<ValueType> target_point(0, 0, 0);

    estimator->setSourcePoints(*source_points);

    std::vector<ValueType> rmse(maximum_number_iterations); // used to compute the progress

    // Iterate.

    unsigned i = 0;

    while (i < maximum_number_iterations && !abort)
    {
        transformed_source_points.clear();
        transformed_source_points.reserve(size);

        target_points.clear();
        target_points.reserve(size);

        // Transform the source points with the actual transformation estimation
        // and search for the corresponding points in the target set.

        Transform3D<ValueType> transform(transformation);

        for (unsigned n = 0; n < size; ++n)
        {
            Coordinate<ValueType> transformed_source_point = transform * (*source_points)[n];

            transformed_source_points.push_back(transformed_source_point);

            const Coordinate<ValueType> & point = transformed_source_points[n];

            query_point[0][0] = point.x();
            query_point[0][1] = point.y();
            query_point[0][2] = point.z();

            tree->knnSearch(query_point, indices, distances, 1, flann::SearchParams(128, super::eps));

            target_point.x() = super::target_points[indices[0][0]][0];
            target_point.y() = super::target_points[indices[0][0]][1];
            target_point.z() = super::target_points[indices[0][0]][2];

            target_points.push_back(target_point);
        }

        // Estimate a new transformation.

        estimator->setTargetPoints(target_points);
        estimator->compute();

        transformation = estimator->getTransformationMatrix();

        // Compute the actual progress [0, ..., 100] using a simple heuristic.
        //
        // 1.  A lower bound is given by the current iteration and the maximum
        //     number of iterations.
        //
        // 2.  The convergence rate of the ICP is assumed to be linear. Estimate
        //     the iteration N when the RMS will be smaller than the given root
        //     mean square error bound. Note: N cannot be greater than the
        //     maximum number of iterations.

        unsigned progress = 0;

        unsigned lower_bound = unsigned(100.0 * (i + 1) / maximum_number_iterations);

        rmse[i] = estimator->getRMS();

        if (i > 0)
        {
            // rmse_bound > y = m * x + b,   m := (rmse[i] - rmse[0]) / i,   m < 0,   b := rmse[0]
            // ==> N := ceil(x) = ceil((rmse_bound - rmse[0]) / ((rmse[i] - rmse[0]) / i))

            using std::numeric_limits;
            ValueType epsilon = numeric_limits<ValueType>::epsilon();

            ValueType m = (rmse[i] - rmse[0]) / i - epsilon;
            ValueType N = (maximum_rms_error - rmse[0]) / m;
            N = N > 0 ? ceil(N) : 0;

            progress = N > 0 ? unsigned(100.0 * i / N) : 100; //if the current RMS is lower than the given error bound (last case), we are done (progress = 100%)
            progress = min<unsigned>(progress, 100); // progress can at most be 100
        }

        progress = max(lower_bound, progress);

        super::progress.send(progress);

        // Iterate + stop criterion.

        if (rmse[i++] < maximum_rms_error)
        {
            if (progress < 100)
            {
                super::progress.send(100);
            }

            break;
        }
    }

    return estimator->getRMS();
}


////////////////////////////////////////////////////////////////////////
//                       IcpTrimmed3D                                 //
////////////////////////////////////////////////////////////////////////



/** \relates IcpInterface
  *
  * \tparam ValueType   Scalar type (must be a floating point type).
  *
  * \brief Trimmed ICP algorithm for 3D point clouds.
  *
  * This algorithm implements a variant of the iterative closest point
  * algorithm which tries to solve the problem of geometrically aligning
  * two roughly pre-registered, partially overlapping, rigid, noisy 3D
  * point sets.
  *
  * The new algorithm is based on the consistent use of the Least Trimmed
  * Squares approach in all phases of the operation. TrICP is applicable
  * to overlaps under 50%, shall be robust to erroneous and incomplete
  * measurements. ICP is a special case of TrICP when the overlap parameter
  * is 100%. [cmp. Chetverikov 2005]
  *
  * For some additional information and an example of how to use this
  * class, please have a look at the \ref IcpInterface "interface class".
  *
  * \see IcpInterface and IcpBase
  *
  * \references
  *
  * Chetverikov et al., "Robust Euclidean alignment of 3D point sets - The
  * trimmed iterative closest point algorithm", Image and Vision Computing,
  * Volume 23, Number 3, Pages 299 - 309, 2005, Elsevier
  *
  * \author Juliana Hsu (with code from Christoph Haenisch)
  * \version 0.1.1
  * \date last changed on 2014-07-04
  */

template <class ValueType>
class IcpTrimmed3D : public IcpBase<ValueType>
{
public:
    IcpTrimmed3D();
    ValueType compute();

	void setMinimumOverlap(ValueType value);


private:
    typedef IcpBase<ValueType> super;

    using super::abort;
    using super::distances;
    using super::estimator;
    using super::indices;
    using super::maximum_number_iterations;
    using super::maximum_rms_error;
    using super::query_point;
    using super::source_points;
    using super::target_points;
    using super::transformation;
    using super::tree;

	ValueType m_minimumOverlap;
};

template<class ValueType>
IcpTrimmed3D<ValueType>::IcpTrimmed3D() :
    IcpBase<ValueType>(3),
	m_minimumOverlap(0.3)
{
    transformation = Transform3D<ValueType>().getTransformationMatrix();

    distances = flann::Matrix<ValueType>(new ValueType[1], 1, 1);
    indices = flann::Matrix<int>(new int[1], 1, 1);
    query_point = flann::Matrix<ValueType>(new ValueType[3], 1, 3);
}


template<class ValueType>
ValueType IcpTrimmed3D<ValueType>::compute()
{
    if (tree->size() == 0)
    {
        throw std::range_error("IcpTrimmed3D::compute(): No target points are given.");
    }

    // Initialize some variables.

    using std::ceil;
    using std::max;
    using std::min;

    abort = false;

    const unsigned size = source_points->size();

    std::vector<Coordinate<ValueType> > transformed_source_points;
	std::vector<Coordinate<ValueType> > trimmed_source_points;
    std::vector<Coordinate<ValueType> > target_points;
	std::vector<Coordinate<ValueType> > trimmed_target_points;
	std::vector<ValueType>				individual_distances;

    Coordinate<ValueType> target_point(0, 0, 0);

    //estimator->setSourcePoints(*source_points);

    std::vector<ValueType> rmse(maximum_number_iterations); // used to compute the progress

    // Iterate.

    unsigned i = 0;

    while (i < maximum_number_iterations && !abort)
    {
        transformed_source_points.clear();
        transformed_source_points.reserve(size);

		trimmed_source_points.clear();
		trimmed_source_points.reserve(size);

        target_points.clear();
        target_points.reserve(size);

		individual_distances.clear();
        individual_distances.reserve(size);

        // Transform the source points with the actual transformation estimation
        // and search for the corresponding points in the target set.

        Transform3D<ValueType> transform(transformation);

        for (unsigned n = 0; n < size; ++n)
        {
            Coordinate<ValueType> transformed_source_point = transform * (*source_points)[n];

            transformed_source_points.push_back(transformed_source_point);

            const Coordinate<ValueType> & point = transformed_source_points[n];

            query_point[0][0] = point.x();
            query_point[0][1] = point.y();
            query_point[0][2] = point.z();

            tree->knnSearch(query_point, indices, distances, 1, flann::SearchParams(128, super::eps));

            target_point.x() = super::target_points[indices[0][0]][0];
            target_point.y() = super::target_points[indices[0][0]][1];
            target_point.z() = super::target_points[indices[0][0]][2];

			// compute distance of transformed source and target
			ValueType d = (transformed_source_point - target_point).squaredNorm();
			individual_distances.push_back(d);
			trimmed_source_points.push_back((*source_points)[n]);
			target_points.push_back(target_point);
        }

		estimator->setSourcePoints(trimmed_source_points);
		estimator->setTargetPoints(target_points);

		// sorting takes too long for large point clouds
		// use approximation instead

		if (m_minimumOverlap < 1)
		{
			// find maximumAllowedDistance:
			// find minimum and maximum
			double maxDistance = 0;
			double minDistance = std::numeric_limits<double>::max();

			if (m_minimumOverlap > 0.5)
			{
				for (unsigned n = 0; n < size; ++n)
				{
					if (individual_distances[n]>maxDistance) maxDistance = individual_distances[n];
					if (individual_distances[n]<minDistance) minDistance = individual_distances[n];
				}
			}
			else
			{
				// improve accuracy: consider distances smaller than the average only
				for (unsigned n = 0; n < size; ++n)
				{
					maxDistance += individual_distances[n];
					if (individual_distances[n]<minDistance) minDistance = individual_distances[n];
				}
				maxDistance /= size;
			}

			// find maximum allowed distance
			double maximumAllowedDistance;
			double minimumNumPoints = m_minimumOverlap*size;

			double distanceDistribution[10000];
			for (int i = 0; i<10000; i++)
            {
				distanceDistribution[i] = 0;
            }
			for (unsigned n = 0; n < size; ++n)
			{
				if (individual_distances[n]<=maxDistance)
                {
					distanceDistribution[(int) ((individual_distances[n]-minDistance)/(maxDistance-minDistance)*9999)]++;
                }
			}
			double distanceDistributionAccumulated[10000];
			distanceDistributionAccumulated[0] = distanceDistribution[0];
			int i;
			for (i = 1; i<10000; i++)
			{
				distanceDistributionAccumulated[i] = distanceDistributionAccumulated[i-1] + distanceDistribution[i];
				if (distanceDistributionAccumulated[i] >= minimumNumPoints)
					break;
			}
			maximumAllowedDistance = ((double) i+0.5)/9999 * (maxDistance-minDistance) + minDistance;

			trimmed_target_points.clear();
			trimmed_target_points.reserve(size);

			trimmed_source_points.clear();

			for (unsigned n = 0; n < size; ++n)
			{
				if (individual_distances[n]<=maximumAllowedDistance)
				{
					trimmed_source_points.push_back((*source_points)[n]);
					trimmed_target_points.push_back(target_points[n]);
				}
			}

			estimator->setSourcePoints(trimmed_source_points);
			estimator->setTargetPoints(trimmed_target_points);
		}

		 // Estimate a new transformation.
        estimator->compute();

        transformation = estimator->getTransformationMatrix();

        // Compute the actual progress [0, ..., 100] using a simple heuristic.
        //
        // 1.  A lower bound is given by the current iteration and the maximum
        //     number of iterations.
        //
        // 2.  The convergence rate of the ICP is assumed to be linear. Estimate
        //     the iteration N when the RMS will be smaller than the given root
        //     mean square error bound. Note: N cannot be greater than the
        //     maximum number of iterations.

        unsigned progress = 0;

        unsigned lower_bound = unsigned(100.0 * (i + 1) / maximum_number_iterations);

        rmse[i] = estimator->getRMS();

        if (i > 0)
        {
            // rmse_bound > y = m * x + b,   m := (rmse[i] - rmse[0]) / i,   m < 0,   b := rmse[0]
            // ==> N := ceil(x) = ceil((rmse_bound - rmse[0]) / ((rmse[i] - rmse[0]) / i))

            using std::numeric_limits;
            ValueType epsilon = numeric_limits<ValueType>::epsilon();

            ValueType m = (rmse[i] - rmse[0]) / i - epsilon;
            ValueType N = (maximum_rms_error - rmse[0]) / m;
            N = N > 0 ? ceil(N) : 0;

            progress = N > 0 ? unsigned(100.0 * i / N) : 100; //if the current RMS is lower than the given error bound (last case), we are done (progress = 100%)
            progress = min<unsigned>(progress, 100); // progress can at most be 100
        }

        progress = max(lower_bound, progress);

        super::progress.send(progress);

        // Iterate + stop criterion.

        if (rmse[i++] < maximum_rms_error)
        {
            if (progress < 100)
            {
                super::progress.send(100);
            }

            break;
        }
    }

    return estimator->getRMS();
}


/** \brief Sets the minimum overlap between the data points and the model points.*/

template<class ValueType>
void IcpTrimmed3D<ValueType>::setMinimumOverlap(ValueType value)
{
    m_minimumOverlap = value;
}


////////////////////////////////////////////////////////////////////////
//                         RandomSampleIcp3D                          //
////////////////////////////////////////////////////////////////////////


/** \relates IcpInterface
  *
  * \tparam ValueType   Scalar type (must be a floating point type).
  *
  * \brief Random Sample ICP algorithm for 3D point clouds.
  *
  * This algorithm implements a variant of the iterative closest point
  * algorithm.
  *
  * Basically, it proceeds in two stages. First, the classical iterative
  * closest point algorithm is computed from a subset of data points. This
  * subset is newly created in each iteration by randomly selecting points
  * from the data points set. Since the subset is constantly changing, the
  * convergence is not monotonicly decreasing anymore and the root mean
  * square error might become greater. Hence, after each iteration the
  * current estimate is saved if it is better than a previous estimate
  * (measured by the RMSE). Second, the classical iterative closest point
  * algorithm is run where its intial estimte is set to the best estimate
  * from the first stage.
  *
  * The maximum number of iterations in the first stage can be set by
  * setMaximumNumberIterationsFirstStage(). The maximum number of iterations
  * in the second stage by setMaximumNumberIterationsSecondStage() or by
  * setMaximumNumberIterations(). The portion of randomly selected samples
  * in the first stage can be set by setPercentage().
  *
  * For some additional information and an example of how to use this
  * class, please have a look at the \ref IcpInterface "interface class".
  *
  * \see IcpInterface and IcpBase
  *
  * \references
  *
  * Fieten et al., "Fast and Accurate Registration of Cranial CT Images With
  * A-mode Ultrasound", International Journal of Computer Assisted Radiology
  * and Surgery, 2009
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2012-08-16
  */

template <class ValueType>
class RandomSampleIcp3D : public IcpBase<ValueType>
{
public:
    RandomSampleIcp3D();
    ValueType compute();                                            //!< Call this function to perform the transformation estimation. The RMSE is returned.

    void setMaximumNumberIterationsFirstStage(unsigned value);      //!< Sets the maximum number of iterations in the first stage (default is 20).
    void setMaximumNumberIterationsSecondStage(unsigned value);     //!< Sets the maximum number of iterations in the second stage (default is 20).
    void setPercentage(unsigned value);                             //!< Sets the portion of randomly selected samples in the first stage. 0 means no samples and 100 means all samples (default is 50).

private:
    typedef IcpBase<ValueType> super;
    typedef typename super::Matrix Matrix;

    using super::abort;
    using super::distances;
    using super::estimator;
    using super::indices;
    using super::maximum_number_iterations;
    using super::maximum_rms_error;
    using super::query_point;
    using super::source_points;
    using super::target_points;
    using super::transformation;
    using super::tree;

    unsigned maximum_number_iterations_first_stage;
    unsigned percentage;
};


template<class ValueType>
RandomSampleIcp3D<ValueType>::RandomSampleIcp3D() :
    IcpBase<ValueType>(3),
    maximum_number_iterations_first_stage(20),
    percentage(50)
{
    transformation = Transform3D<ValueType>().getTransformationMatrix();

    distances = flann::Matrix<ValueType>(new ValueType[1], 1, 1);
    indices = flann::Matrix<int>(new int[1], 1, 1);
    query_point = flann::Matrix<ValueType>(new ValueType[3], 1, 3);

    maximum_number_iterations = 20;
}


/** \exception range_error A range error exception is thrown if no model points are given.
  *
  * \note The given estimator might throw an exception as well.
  */

template<class ValueType>
ValueType RandomSampleIcp3D<ValueType>::compute()
{
    if (tree->size() == 0)
    {
        throw std::range_error("RandomSampleIcp3D::compute(): No target points are given.");
    }

    // Initialize some variables.

    using std::ceil;
    using std::max;
    using std::min;
    using std::numeric_limits;

    abort = false;

    const unsigned size = source_points->size();

    std::vector<Coordinate<ValueType> > source_points; // note, this hides the class member
    std::vector<Coordinate<ValueType> > transformed_source_points;
    std::vector<Coordinate<ValueType> > target_points;

    Coordinate<ValueType> target_point(0, 0, 0);

    Matrix best_estimate = transformation;

    ValueType least_rmse = numeric_limits<ValueType>::max();

    unsigned maximum_number_iterations_per_stage = max(maximum_number_iterations, maximum_number_iterations_first_stage);
    unsigned maximum_number_iterations_overall = maximum_number_iterations + maximum_number_iterations_first_stage;

    std::vector<ValueType> rmse(maximum_number_iterations_per_stage); // used to compute the progress

    ValueType ratio = ValueType(maximum_number_iterations_first_stage) / ValueType(maximum_number_iterations_overall);

    // PHASE 1 --> Randomly selected subsets are taken to "feed" the classical ICP algorithm.

    // Iterate.

    unsigned i = 0;

    while (i < maximum_number_iterations_first_stage && !abort)
    {
        source_points.clear();
        source_points.reserve(size);

        transformed_source_points.clear();
        transformed_source_points.reserve(size);

        target_points.clear();
        target_points.reserve(size);

        // Transform the source points with the actual transformation estimation
        // and search for the corresponding points in the target set.

        Transform3D<ValueType> transform(transformation);

        for (unsigned n = 0; n < size; ++n)
        {
            if (Tools::rand(0, 100) > percentage) continue; // randomly select points

            Coordinate<ValueType> source_point = (*this->source_points)[n];

            source_points.push_back(source_point);

            Coordinate<ValueType> transformed_source_point = transform * source_point;

            transformed_source_points.push_back(transformed_source_point);

            const Coordinate<ValueType> & point = transformed_source_point;

            query_point[0][0] = point.x();
            query_point[0][1] = point.y();
            query_point[0][2] = point.z();

            tree->knnSearch(query_point, indices, distances, 1, flann::SearchParams(128, super::eps));

            target_point.x() = super::target_points[indices[0][0]][0];
            target_point.y() = super::target_points[indices[0][0]][1];
            target_point.z() = super::target_points[indices[0][0]][2];

            target_points.push_back(target_point);
        }

        // Estimate a new transformation.

        estimator->setSourcePoints(source_points);
        estimator->setTargetPoints(target_points);
        estimator->compute();

        transformation = estimator->getTransformationMatrix();

        // Store the estimation if its RMSE is lesser than a previous one.

        if (estimator->getRMS() < least_rmse)
        {
            least_rmse = estimator->getRMS();
            best_estimate = estimator->getTransformationMatrix();
        }

        // Compute the actual progress [0, ..., 100] using a simple heuristic.
        //
        // 1.  A lower bound is given by the current iteration and the maximum
        //     number of iterations.
        //
        // 2.  The convergence rate of the ICP is assumed to be linear. Estimate
        //     the iteration N when the RMS will be smaller than the given root
        //     mean square error bound. Note: N cannot be greater than the
        //     maximum number of iterations.

        unsigned progress = 0;

        unsigned lower_bound = unsigned(100.0 * (i + 1) / maximum_number_iterations_first_stage);

        rmse[i] = estimator->getRMS();

        if (i > 0)
        {
            // rmse_bound > y = m * x + b,   m := (rmse[i] - rmse[0]) / i,   m < 0,   b := rmse[0]
            // ==> N := ceil(x) = ceil((rmse_bound - rmse[0]) / ((rmse[i] - rmse[0]) / i))

            ValueType epsilon = numeric_limits<ValueType>::epsilon();

            ValueType m = (rmse[i] - rmse[0]) / i - epsilon;
            ValueType N = (maximum_rms_error - rmse[0]) / m;
            N = N > 0 ? ceil(N) : 0;

            progress = N > 0 ? unsigned(100.0 * i / N) : 100; //if the current RMS is lower than the given error bound (last case), we are done (progress = 100%)
            progress = min<unsigned>(progress, 100); // progress can at most be 100
        }

        progress = max(lower_bound, progress);

        super::progress.send(unsigned(progress * ratio));

        // Iterate + stop criterion.

        if (rmse[i++] < maximum_rms_error)
        {
            if (progress < 100)
            {
                super::progress.send(unsigned(100 * ratio));
            }

            break;
        }
    }

    // Use the best transformation estimate for the next phase.

    transformation = best_estimate;

    // Clean up some no longer used variables.

    source_points.clear();
    source_points.reserve(0);

    // PHASE 2

    estimator->setSourcePoints(*this->source_points);

    // Iterate.

    i = 0;

    while (i < maximum_number_iterations && !abort)
    {
        transformed_source_points.clear();
        transformed_source_points.reserve(size);

        target_points.clear();
        target_points.reserve(size);

        // Transform the source points with the actual transformation estimation
        // and search for the corresponding points in the target set.

        Transform3D<ValueType> transform(transformation);

        for (unsigned n = 0; n < size; ++n)
        {
            Coordinate<ValueType> transformed_source_point = transform * (*this->source_points)[n];

            transformed_source_points.push_back(transformed_source_point);

            const Coordinate<ValueType> & point = transformed_source_points[n];

            query_point[0][0] = point.x();
            query_point[0][1] = point.y();
            query_point[0][2] = point.z();

            tree->knnSearch(query_point, indices, distances, 1, flann::SearchParams(128, super::eps));

            target_point.x() = super::target_points[indices[0][0]][0];
            target_point.y() = super::target_points[indices[0][0]][1];
            target_point.z() = super::target_points[indices[0][0]][2];

            target_points.push_back(target_point);
        }

        // Estimate a new transformation.

        estimator->setTargetPoints(target_points);
        estimator->compute();

        transformation = estimator->getTransformationMatrix();

        // Compute the actual progress [0, ..., 100] using a simple heuristic.
        //
        // 1.  A lower bound is given by the current iteration and the maximum
        //     number of iterations.
        //
        // 2.  The convergence rate of the ICP is assumed to be linear. Estimate
        //     the iteration N when the RMS will be smaller than the given root
        //     mean square error bound. Note: N cannot be greater than the
        //     maximum number of iterations.

        unsigned progress = 0;

        unsigned lower_bound = unsigned(100.0 * (i + 1) / maximum_number_iterations);

        rmse[i] = estimator->getRMS();

        if (i > 0)
        {
            // rmse_bound > y = m * x + b,   m := (rmse[i] - rmse[0]) / i,   m < 0,   b := rmse[0]
            // ==> N := ceil(x) = ceil((rmse_bound - rmse[0]) / ((rmse[i] - rmse[0]) / i))

            ValueType epsilon = numeric_limits<ValueType>::epsilon();

            ValueType m = (rmse[i] - rmse[0]) / i - epsilon;
            ValueType N = (maximum_rms_error - rmse[0]) / m;
            N = N > 0 ? ceil(N) : 0;

            progress = N > 0 ? unsigned(100.0 * i / N) : 100; //if the current RMS is lower than the given error bound (last case), we are done (progress = 100%)
            progress = min<unsigned>(progress, 100); // progress can at most be 100
        }

        progress = max(lower_bound, progress);

        super::progress.send(unsigned(100 * ratio + progress * (1 - ratio)));

        // Iterate + stop criterion.

        if (rmse[i++] < maximum_rms_error)
        {
            if (progress < 100)
            {
                super::progress.send(100);
            }

            break;
        }
    }

    return estimator->getRMS();
}


template<class ValueType>
void RandomSampleIcp3D<ValueType>::setMaximumNumberIterationsFirstStage(unsigned value)
{
    maximum_number_iterations_first_stage = value;
}


template<class ValueType>
void RandomSampleIcp3D<ValueType>::setMaximumNumberIterationsSecondStage(unsigned value)
{
    maximum_number_iterations = value;
}


template<class ValueType>
void RandomSampleIcp3D<ValueType>::setPercentage(unsigned value)
{
    assert(percentage <= 100);
    percentage = std::min(value, 100u);
}


} // namespace TRTK


#endif // ICP_HPP_7908314317
