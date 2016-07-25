/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.0 (2012-03-23)
*/

/** \file Ransac.hpp
  * \brief This file contains the \ref TRTK::Ransac "Ransac" class.
  */

#ifndef RANSAC_HPP_1147104378
#define RANSAC_HPP_1147104378


#include <cstddef>
#include <cmath>
#include <limits>
#include <vector>
#include <set>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"


namespace TRTK
{


/** \tparam T           Scalar type (must be a floating point).
  * \tparam DataType    Type of the data points.
  *
  * \brief Robust model fitting.
  *
  * The RANSAC algorithm works by taking initial data sets (where a set
  * contains as few points as feasible, just to support the model) and by
  * enlarging these sets with consistent data. Finally the bigest consensus
  * set is taken and a new refined model is computed from these data points.
  * This algorithm conform to the original algorithm described by Fischler
  * and Bolles. However, this class also provides several variations of this
  * algorithm.
  *
  * All algorithms assume that there are two sources of error. First,
  * measurement errors which in general are small and normally distributed;
  * these average out while doing a regression analysis. Second, gross
  * errors which are quite big and arbitrarily distributed. All algorithms
  * assume that you define some error bound (which constitutes the deviation
  * of a datum to an estimated model) which should be chosen such that
  * all gross errors are neglected but as many as possible good points are
  * taken into consideration. Since measurement errors are quite small,
  * this is often easily possible.
  *
  * If no maximum number of attempts (\ref setTrials() "trials") to find a
  * consensus set is given, it is automatically computed from the minimum
  * number of samples needed to compute a model as well as from the
  * \ref setProbability() "probability" (the default value is 0.2) that any
  * selected data point for such a minimal set is within the given
  * \ref setErrorTolerance "error tolerance". If no \ref setThreshold() "threshold"
  * is given, it is set to 8/10 of the size of \p data. The default algorithm
  * is "RANSAC".
  *
  * The class works according to the strategy pattern; the class as is
  * constitutes the context and all models used form the strategies. A
  * regression model (e.g. a line fitting model) must conform to a certain
  * interfacen, namely \ref Model. See the example below.
  *
  * Example:
  *
  * \code
  * #include <iostream>
  *
  * #include <TRTK/Ransac.hpp>
  * #include <TRTK/RansacModelFitLine.hpp>
  * #include <TRTK/Tools.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
  *
  *
  * int main()
  * {
  *     // Construct some points lying on a line and add some noise and outliers.
  *
  *     vector<Coordinate<double> > points;
  *
  *     double slope = 0.7;
  *     double y_intercept = -3;
  *
  *     for (int i = -10; i < 10; ++i)
  *     {
  *         // Noisy measurement.
  *
  *         double x = i + randn(0.0, 0.1);
  *         double y = i * slope + y_intercept + randn(0.0, 0.1);
  *
  *         Coordinate<double> point(x, y);
  *
  *         points.push_back(point);
  *     }
  *
  *     for (int i = 0; i < 5; ++i)
  *     {
  *         // Gros outliers.
  *
  *         double x = rand(-10.0, 10.0);
  *         double y = rand(-10.0, 10.0);
  *
  *         Coordinate<double> point(x, y);
  *
  *         points.push_back(point);
  *     }
  *
  *     // Estimate the line parameters using ordinary least sqares.
  *
  *     FitLine<double> fitLine;
  *     fitLine.setPoints(points);
  *     fitLine.compute();
  *
  *     cout << "Slope: " << fitLine.getSlope() << endl;
  *     cout << "Y-intercept: " << fitLine.getYIntercept() << endl;
  *     cout << "Direction Vector: " << fitLine.getDirectionVector() << endl;
  *     cout << "Distance from origin: " << fitLine.getDistanceFromOrigin() << endl;
  *     cout << "RMSE: " << fitLine.getRMS() << endl << endl;
  *
  *     // Estimate the line parameters using RANSAC.
  *
  *     Ransac<double> ransac;
  *     RansacModelFitLine<double> model;
  *
  *     ransac.setModel(model);
  *     ransac.setData(points);
  *     ransac.setErrorTolerance(0.2);
  *
  *     unsigned number_of_samples_used = ransac.compute();
  *
  *     cout << "Slope: " << model.getSlope() << endl;
  *     cout << "Y-intercept: " << model.getYIntercept() << endl;
  *     cout << "Direction Vector: " << model.getDirectionVector() << endl;
  *     cout << "Distance from origin: " << model.getDistanceFromOrigin() << endl;
  *     cout << "Number of samples used: " << number_of_samples_used << endl;
  *     cout << "RMSE: " << model.getRMS() << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Slope: 0.824709
  * Y-intercept: -2.97947
  * Direction Vector: (-0.771483, -0.63625)
  * Distance from origin: 2.29861
  * RMSE: 2.27904
  *
  * Slope: 0.691347
  * Y-intercept: -3.00877
  * Direction Vector: (-0.822562, -0.568676)
  * Distance from origin: 2.4749
  * Number of samples used: 19
  * RMSE: 0.0712661
  * \endcode
  *
  * \references
  *
  * "Random Sample Consensus: A Paradigm for Model Fitting with Applications
  * to Image Analysis and Automated Cartography", Fischler and Bolles, 1981,
  * Communications of the ACM
  *
  * \see Model
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2012-03-23
  */

template <class T, class DataType = Coordinate<T> >
class Ransac
{
public:

    class Model
    {
    public:
        virtual void compute() = 0;                                     ///< Estimate the model parameters.
        virtual T getDeviation(const DataType & datum) const = 0;       ///< Return the amount of how much a datum deviates from the model.
        virtual unsigned getMinimumNumberOfItems() const = 0;           ///< Return the minimum number of items required to compute the model.
        virtual T getRMSE() const = 0;                                  ///< Return the root mean square error of the estimated regression model.
        virtual void setData(const std::vector<DataType> & data) = 0;   ///< Set the sample data. No data is copied but a reference is stored.
    };                                                                  ///< Interface class for models to be used with the Ransac class.

    enum Algorithm
    {
        RANSAC,                             ///< RANSAC algorithm as described by Fischler and Bolles. If a consensus set is found whose size is greater than \p threshold then this set is taken to compute a new model and the algorithm is terminated. Otherwise a new model is computed from the biggest consensus set found.
        RANSAC_BIGGEST_CONSESUS_SET,        ///< A new model is always computed from the biggest consensus set.
        RANSAC_SMALLEST_RMSE                ///< The model with the smallest RMSE is taken.
    };

    enum Error
    {
        NO_DATA_AVAILABLE,                  ///< No data points were given.
        NO_MODEL_AVAILABLE,                 ///< No regression model was set.
        NOT_ENOUGH_DATA_POINTS,             ///< There are not enough data points to compute the model.
        UNKNOWN_ALGORITHM,                  ///< The algorithm does not exist or is not yet implemented.
        UNKNOWN_ERROR                       ///< An unknown error occurred.
    };

    Ransac();
    virtual ~Ransac();

    unsigned compute();

    void setAlgorithm(Algorithm algorithm = RANSAC);
    void setData(const std::vector<DataType> & data);
    void setErrorTolerance(T error_tolerance);
    void setMinimumSetSize(unsigned threshold);
    void setModel(Model & model);
    void setProbability(T probability);
    void setThreshold(unsigned threshold);
    void setTrials(unsigned number);

protected:
    const std::vector<DataType> * data;
    Model * model;

    T error_tolerance;
    unsigned threshold;
    unsigned trials;
    T probability;                          // Probability that any selected data point of a minimal subset of data is within the error tolerance of the model.
    unsigned minimumSetSize;

    unsigned (Ransac::*algorithm)();

    unsigned algorithmRansac();
    unsigned algorithmRansacBiggestConsensusSet();
    unsigned algorithmRansacSmallestRmse();
};


template <class T, class DataType>
Ransac<T, DataType>::Ransac() :
    data(NULL),
    model(NULL),
    algorithm(&Ransac::algorithmRansac),
    error_tolerance(0),
    threshold(0),
    trials(0),
    probability(0.2),
    minimumSetSize(0)
{
}


template <class T, class DataType>
Ransac<T, DataType>::~Ransac()
{
}


/** \brief RANSAC algorithm as described by Fischler and Bolles.
  *
  * If a consensus set is found whose size is greater than a given
  * threshold then this set is taken to compute a new model and the
  * algorithm is terminated. Otherwise a new model is computed from
  * the biggest consensus set found.
  *
  * \return Returns the number of items used to compute the model.
  */

template <class T, class DataType>
unsigned Ransac<T, DataType>::algorithmRansac()
{
    using namespace std;
    using namespace Tools;

    const unsigned maximum_number_of_items = data->size();
    const unsigned minimum_number_of_items = model->getMinimumNumberOfItems();

    vector<DataType> initial_data_set(minimum_number_of_items);
    vector<DataType> consensus_set;
    vector<DataType> best_consensus_set;

    // For each trial...

    for (unsigned i = 0; i < trials; ++i)
    {
        // Randomly select items from the data set and use them
        // as an initial data set for the model fitting.

        set<int> indices;

        for (unsigned j = 0; j < minimum_number_of_items; ++j)
        {
            // Avoid using the same data point several times.

            unsigned index = rand(0u, maximum_number_of_items - 1);;

            while (indices.find(index) != indices.end())
            {
                index = rand(0u, maximum_number_of_items - 1);
            }

            indices.insert(index);

            // Store the randomly selected data point.

            initial_data_set[j] = (*data)[index];
        }

        // Estimate a model from the initial data set.

        model->setData(initial_data_set);
        model->compute();

        // Construct a new consensus set and add those items from the
        // original data set which lie within some error tolerance.

        consensus_set.clear();
        consensus_set.reserve(maximum_number_of_items);

        for (unsigned j = 0; j < maximum_number_of_items; ++j)
        {
            if (model->getDeviation((*data)[j]) < error_tolerance)
            {
                consensus_set.push_back((*data)[j]);
            }
        }

        if (consensus_set.size() <= minimum_number_of_items)
        {
            // Actually, this should never happen!
            // But you never know...

            consensus_set = initial_data_set;
        }

        // If the size of the consensus set is greater than the given
        // threshold, then compute a new model from this set and exit.

        if (consensus_set.size() > threshold)
        {
            model->setData(consensus_set);
            model->compute();

            return consensus_set.size();
        }

        // The size of the consensus set is *not* greater than the given
        // threshold, so randomly select a new subset. Save the old
        // consensus set in case no better one is found.

        if (best_consensus_set.size() < consensus_set.size())
        {
            best_consensus_set = consensus_set;
        }
    }

    // Use the consensus set to compute a new model.

    model->setData(best_consensus_set);
    model->compute();

    return best_consensus_set.size();
}


/** \brief Modified version of the RANSAC algorithm as described by Fischler and Bolles.
  *
  * A new model is always computed from the biggest consensus set.
  *
  * \return Returns the number of items used to compute the model.
  */

template <class T, class DataType>
unsigned Ransac<T, DataType>::algorithmRansacBiggestConsensusSet()
{
    using namespace std;
    using namespace Tools;

    const unsigned maximum_number_of_items = data->size();
    const unsigned minimum_number_of_items = model->getMinimumNumberOfItems();

    vector<DataType> initial_data_set(minimum_number_of_items);
    vector<DataType> consensus_set;
    vector<DataType> best_consensus_set;

    // For each trial...

    for (unsigned i = 0; i < trials; ++i)
    {
        // Randomly select items from the data set and use them
        // as an initial data set for the model fitting.

        set<int> indices;

        for (unsigned j = 0; j < minimum_number_of_items; ++j)
        {
            // Avoid using the same data point several times.

            unsigned index = rand(0u, maximum_number_of_items - 1);;

            while (indices.find(index) != indices.end())
            {
                index = rand(0u, maximum_number_of_items - 1);
            }

            indices.insert(index);

            // Store the randomly selected data point.

            initial_data_set[j] = (*data)[index];
        }

        // Estimate a model from the initial data set.

        model->setData(initial_data_set);
        model->compute();

        // Construct a new consensus set and add those items from the
        // original data set which lie within some error tolerance.

        consensus_set.clear();
        consensus_set.reserve(maximum_number_of_items);

        for (unsigned j = 0; j < maximum_number_of_items; ++j)
        {
            if (model->getDeviation((*data)[j]) < error_tolerance)
            {
                consensus_set.push_back((*data)[j]);
            }
        }

        if (consensus_set.size() <= minimum_number_of_items)
        {
            // Actually, this should never happen!
            // But you never know...

            consensus_set = initial_data_set;
        }

        // Randomly select a new subset. Save the old consensus set
        // in case no better one is found.

        if (best_consensus_set.size() < consensus_set.size())
        {
            best_consensus_set = consensus_set;
        }
    }

    // Use the biggest consensus set to compute a new model.

    model->setData(best_consensus_set);
    model->compute();

    return best_consensus_set.size();
}


/** \brief Modified version of the RANSAC algorithm as described by Fischler and Bolles.
  *
  * The model with the smallest RMSE is taken. The consensus set must be bigger
  * than a given threshold (see \ref setMinimumSetSize()), otherwise no model
  * is estimated (i.e. the algorithm returns 0).
  *
  * \return Returns the number of items used to compute the model.
  */

template <class T, class DataType>
unsigned Ransac<T, DataType>::algorithmRansacSmallestRmse()
{
    using namespace std;
    using namespace Tools;

    const unsigned maximum_number_of_items = data->size();
    const unsigned minimum_number_of_items = model->getMinimumNumberOfItems();

    vector<DataType> initial_data_set(minimum_number_of_items);
    vector<DataType> consensus_set;
    vector<DataType> best_consensus_set;

    T rmse = numeric_limits<T>::max();

    // For each trial...

    for (unsigned i = 0; i < trials; ++i)
    {
        // Randomly select items from the data set and use them
        // as an initial data set for the model fitting.

        set<int> indices;

        for (unsigned j = 0; j < minimum_number_of_items; ++j)
        {
            // Avoid using the same data point several times.

            unsigned index = rand(0u, maximum_number_of_items - 1);;

            while (indices.find(index) != indices.end())
            {
                index = rand(0u, maximum_number_of_items - 1);
            }

            indices.insert(index);

            // Store the randomly selected data point.

            initial_data_set[j] = (*data)[index];
        }

        // Estimate a model from the initial data set.

        model->setData(initial_data_set);
        model->compute();

        // Construct a new consensus set and add those items from the
        // original data set which lie within some error tolerance.

        consensus_set.clear();
        consensus_set.reserve(maximum_number_of_items);

        for (unsigned j = 0; j < maximum_number_of_items; ++j)
        {
            if (model->getDeviation((*data)[j]) < error_tolerance)
            {
                consensus_set.push_back((*data)[j]);
            }
        }

        if (consensus_set.size() <= minimum_number_of_items)
        {
            // Actually, this should never happen!
            // But you never know...

            consensus_set = initial_data_set;
        }

        // Compute the model with the current consensus set.

        model->setData(consensus_set);
        model->compute();

        T rmse_current_model = model->getRMSE();

        // Randomly select a new subset. Save the old consensus set
        // in case no better one is found.

        if (rmse_current_model < rmse && consensus_set.size() >= minimumSetSize)
        {
            best_consensus_set = consensus_set;
        }
    }

    // Use the biggest consensus set to compute a new model.

    if (best_consensus_set.size() > minimum_number_of_items)
    {
        model->setData(best_consensus_set);
        model->compute();

        return best_consensus_set.size();
    }
    else
    {
        return 0;
    }
}


/** \brief Runs the model fitting algorithm.
  *
  * If no maximum number of attempts (trials) to find a consensus
  * set is given, it is automatically computed from the minimum number
  * of samples needed to compute a model as well as from the
  * probability that any selected data point for such a minimal set
  * is within the given error tolerance. See the references for more
  * details. If no threshold is given, it is set to 8/10 of the size
  * of \ref data.
  *
  * \throw ErrorObj     If the regression model is not set, an error
  *                     object is thrown and its error code set to
  *                     \c NO_MODEL_AVAILABLE.
  *
  * \throw ErrorObj     If no data is given, an error obejct is
  *                     thrown and its error code set to
  *                     \c NO_DATA_AVAILABLE.
  *
  * \throw ErrorObj     If there are not enough data points to compute
  *                     the model, an error object is thrown and its
  *                     error code set to \c NOT_ENOUGH_DATA_POINTS.
  *
  * \return Returns the number of items used to compute the model.
  */

template <class T, class DataType>
unsigned Ransac<T, DataType>::compute()
{
    // Checking some assumptions.

    if (!model)
    {
        ErrorObj error;
        error.setClassName("Ransac");
        error.setFunctionName("compute");
        error.setErrorMessage("Regression model was not set.");
        error.setErrorCode(NO_MODEL_AVAILABLE);
        throw error;
    }

    if (!data)
    {
        ErrorObj error;
        error.setClassName("Ransac");
        error.setFunctionName("compute");
        error.setErrorMessage("No data available.");
        error.setErrorCode(NO_DATA_AVAILABLE);
        throw error;
    }

    if (data->size() < model->getMinimumNumberOfItems())
    {
        ErrorObj error;
        error.setClassName("Ransac");
        error.setFunctionName("compute");
        error.setErrorMessage("Not enough data points to compute the model.");
        error.setErrorCode(NOT_ENOUGH_DATA_POINTS);
        throw error;
    }

    using namespace std;

    // Compute the maximum number of attempts (trials) to find a consensus
    // set if not given. See the paper of Fischler and Bolles.

    unsigned trials_backup = trials;

    if (trials == 0)
    {
        int n = model->getMinimumNumberOfItems();
        trials = unsigned(ceil(3.0 * pow(probability, -T(n))));
    }

    // If no threshold is given, set it to 8/10 of the size of "data".

    unsigned threshold_backup = threshold;

    if (threshold == 0)
    {
        threshold = unsigned(ceil(0.8 * data->size()));
    }

    // If the minimum set size is greater than the size of "data" shrink
    // it to #data - model->getMinimumNumberOfItems().

    unsigned minimumSetSize_backup = minimumSetSize;

    if (minimumSetSize > data->size() && data->size() > model->getMinimumNumberOfItems())
    {
        minimumSetSize = data->size() - model->getMinimumNumberOfItems();
    }

    // Start the computation.

    unsigned number_of_items_used = (this->*algorithm)();

    // Restore the changed variables.

    minimumSetSize = minimumSetSize_backup;
    threshold = threshold_backup;
    trials = trials_backup;

    return number_of_items_used;
}


/** \brief Sets the algorithm used to compute the consensus set. */

template <class T, class DataType>
void Ransac<T, DataType>::setAlgorithm(Algorithm algorithm)
{
    switch(algorithm)
    {
        case RANSAC:
        {
            this->algorithm = &Ransac::algorithmRansac;
            break;
        }

        case RANSAC_BIGGEST_CONSESUS_SET:
        {
            this->algorithm = &Ransac::algorithmRansacBiggestConsensusSet;
            break;
        }

        case RANSAC_SMALLEST_RMSE:
        {
            this->algorithm = &Ransac::algorithmRansacSmallestRmse;
            break;
        }

        default:
        {
            ErrorObj error;
            error.setClassName("Ransac");
            error.setFunctionName("setAlgorithm");
            error.setErrorMessage("Unknown algorithm.");
            error.setErrorCode(UNKNOWN_ALGORITHM);
            throw error;
        }
    }
}


/** \brief Sets the sample data. */

template <class T, class DataType>
void Ransac<T, DataType>::setData(const std::vector<DataType> & data)
{
    this->data = &data;
}


/** \brief Sets the error tolerance.
  *
  * Sets the error tolerance for establishing the datum-model-compatibility,
  * i.e. the amount of how much a data point is allowed to deviate from the
  * model.
  */

template <class T, class DataType>
void Ransac<T, DataType>::setErrorTolerance(T error_tolerance)
{
    this->error_tolerance = error_tolerance;
}


/** \brief Sets the minimal size a consensus set must have (not used by all alogrithms). */

template <class T, class DataType>
void Ransac<T, DataType>::setMinimumSetSize(unsigned size)
{
    minimumSetSize = size;
}


/** \brief Sets the model. */

template <class T, class DataType>
void Ransac<T, DataType>::setModel(Model & model)
{
    this->model = &model;
}


/** \brief Sets the probability that...
  *
  * ... any selected data point in a minimal set for computing
  * the model is within the given error tolerance.
  */

template <class T, class DataType>
void Ransac<T, DataType>::setProbability(T probability)
{
    this->probability = probability;
}


/** \brief Sets a threshold...
  *
  * ... denoting a bound concerning the size of the consensus
  * set after which some algorithms might stop looking for further
  * consensus sets and instead use the current set to compute a new
  * model.
  */

template <class T, class DataType>
void Ransac<T, DataType>::setThreshold(unsigned threshold)
{
    this->threshold = threshold;
}


/** \brief Sets the maximum number of attempts to find a consensus set. */

template <class T, class DataType>
void Ransac<T, DataType>::setTrials(unsigned number)
{
    trials = number;
}


} // namespace TRTK


#endif // RANSAC_HPP_1147104378
