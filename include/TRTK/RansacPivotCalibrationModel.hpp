/*
    Copyright (C) 2010 - 2016 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.0 (2016-07-06)
*/

/** \file RansacPivotCalibrationModel.hpp
  * \brief This file contains the \ref TRTK::RansacGenericFittingModel "RansacGenericFittingModel" class.
  */

#ifndef RANSAC_PIVOT_CALIBRATION_HPP_2432433439
#define RANSAC_PIVOT_CALIBRATION_HPP_2432433439


#include <list>
#include <utility>

#include <Eigen/Core>

#include "PivotCalibration.hpp"
#include "Ransac.hpp"


namespace TRTK
{

/** \tparam T Scalar type (must be a floating point).
  *
  * \brief This class implements the Ransac::Model interface for all PivotCalibration classes.
  *
  * Here is an example that shows how to use this class:
  *
  * \code
  * #include <iostream>
  *
  * #include <TRTK/PivotCalibration.hpp>
  * #include <TRTK/RansacPivotCalibrationModel.hpp>
  * #include <TRTK/Tools.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
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
  *     FitLine<double> fitLine(points);
  *
  *     fitLine.compute();
  *
  *     cout << "Slope: " << fitLine.getSlope() << endl;
  *     cout << "Y-intercept: " << fitLine.getYIntercept() << endl;
  *     cout << "Direction Vector: " << fitLine.getDirectionVector() << endl;
  *     cout << "Distance from origin: " << fitLine.getDistanceFromOrigin() << endl;
  *     cout << "RMS: " << fitLine.getRMS() << endl << endl;
  *
  *     // Estimate the line parameters using RANSAC.
  *
  *     RansacGenericFittingModel<double> model(fitLine);
  *
  *     Ransac<double> ransac;
  *
  *     ransac.setModel(model);
  *     ransac.setData(points);
  *     ransac.setErrorTolerance(0.2);
  *
  *     unsigned number_of_samples_used = ransac.compute();
  *
  *     cout << "Slope: " << fitLine.getSlope() << endl;
  *     cout << "Y-intercept: " << fitLine.getYIntercept() << endl;
  *     cout << "Direction Vector: " << fitLine.getDirectionVector() << endl;
  *     cout << "Distance from origin: " << fitLine.getDistanceFromOrigin() << endl;
  *     cout << "Number of samples used: " << number_of_samples_used << endl;
  *     cout << "RMS: " << model.getRMS() << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * RMS: ...
  * \endcode
  *
  * For further information, please have a look at the documentation of the particular
  * \ref PivotCalibration "Pivot calibration class" and the \ref Ransac class, respectively.
  *
  * \see Ransac, Ransac::Model, PivotCalibration
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2016-07-06
  */


template <class T>
class RansacPivotCalibrationModel : public Ransac<T, typename PivotCalibration<T>::DataType>::Model
{
public:

    typedef T value_type;
    typedef typename PivotCalibration<T>::Vector3T Vector3T;
    typedef typename PivotCalibration<T>::Matrix3T Matrix3T;
    typedef typename PivotCalibration<T>::DataType DataType;

    RansacPivotCalibrationModel(PivotCalibration<T> & estimator);
    virtual ~RansacPivotCalibrationModel();

    void compute();                                             ///< Estimates the model parameters.
    T getDeviation(const DataType & datum) const;               ///< Returns the amount of how much a datum deviates from the model.
    unsigned getMinimumNumberOfItems() const;                   ///< Returns the minimum number of items required to compute the model.
    T getRMSE() const;                                          ///< Returns the root mean square error of the estimated regression model.
    void setData(const std::vector<DataType> & data);           ///< Sets the sample data.
    void setEstimator(PivotCalibration<T> & estimator);         ///< Sets the \ref PivotCalibration "pivot calibration instance".

private:

    PivotCalibration<T> * estimator;
};


/** \brief Constructs an instance of RansacGenericFittingModel.
  *
  * \param [in] model   A \ref Fit "fitting class".
  */

template<class T>
RansacPivotCalibrationModel<T>::RansacPivotCalibrationModel(PivotCalibration<T> & estimator) :
    estimator(&estimator)
{
}


template<class T>
RansacPivotCalibrationModel<T>::~RansacPivotCalibrationModel()
{
}


template<class T>
void RansacPivotCalibrationModel<T>::compute()
{
    estimator->compute();
}


template<class T>
T RansacPivotCalibrationModel<T>::getDeviation(const DataType & datum) const
{
    Vector3T location = datum.first;
    Matrix3T rotation = datum.second;
    Vector3T noisy_pivot_point = rotation * estimator->getLocalPivotPoint() + location;
    return (estimator->getPivotPoint() - noisy_pivot_point).norm();
}


template<class T>
unsigned RansacPivotCalibrationModel<T>::getMinimumNumberOfItems() const
{
    return estimator->getNumberItemsRequired();
}


template<class T>
T RansacPivotCalibrationModel<T>::getRMSE() const
{
    return estimator->getRMSE();
}


template<class T>
void RansacPivotCalibrationModel<T>::setData(const std::vector<DataType> & data)
{
    std::list<DataType::first_type> locations;
    std::list<DataType::second_type> rotations;

    for (size_t i = 0; i < data.size(); ++i)
    {
        locations.push_back(data[i].first);
        rotations.push_back(data[i].second);
    }

    estimator->setLocations(make_range(locations));
    estimator->setRotations(make_range(rotations));
}


template<class T>
void RansacPivotCalibrationModel<T>::setEstimator(PivotCalibration<T> & estimator)
{
    this->estimator = &estimator;
}


} // namespace TRTK


#endif // RANSAC_PIVOT_CALIBRATION_HPP_2432433439
