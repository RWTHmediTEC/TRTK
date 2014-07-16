/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.0 (2012-03-29)
*/

/** \file RansacGenericFittingModel.hpp
  * \brief This file contains the \ref TRTK::RansacGenericFittingModel "RansacGenericFittingModel" class.
  */

#ifndef RANSAC_GENERIC_FITTING_MODEL_HPP_2437670324
#define RANSAC_GENERIC_FITTING_MODEL_HPP_2437670324


#include "Fit.hpp"
#include "Ransac.hpp"


namespace TRTK
{

/** \tparam T           Scalar type (must be a floating point).
  *
  * \brief This class implements the Ransac::Model interface for all fitting classes.
  *
  * Here is an example that shows how to use this class:
  *
  * \code
  * #include <iostream>
  *
  * #include <TRTK/FitLine.hpp>
  * #include <TRTK/RansacGenericFittingModel.hpp>
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
  * Slope: 0.824709
  * Y-intercept: -2.97947
  * Direction Vector: (-0.771483, -0.63625)
  * Distance from origin: 2.29861
  * RMS: 2.27904
  *
  * Slope: 0.691347
  * Y-intercept: -3.00877
  * Direction Vector: (-0.822562, -0.568676)
  * Distance from origin: 2.4749
  * Number of samples used: 19
  * RMS: 0.0712661
  * \endcode
  *
  * For further information, please have a look at the documentation of the particular
  * \ref Fit "fitting class" and the \ref Ransac class, respectively.
  *
  * \see Ransac, Ransac::Model, Fit
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2012-03-23
  */

template <class T>
class RansacGenericFittingModel : public Ransac<T>::Model
{
public:

    RansacGenericFittingModel(Fit<T> & model);
    virtual ~RansacGenericFittingModel();

    void compute();                                             ///< Estimates the model parameters.
    T getDeviation(const Coordinate<T> & datum) const;          ///< Returns the amount of how much a datum deviates from the model.
    unsigned getMinimumNumberOfItems() const;                   ///< Returns the minimum number of items required to compute the model.
    T getRMS() const;                                           ///< Returns the root mean square error of the estimated regression model.
    void setData(const std::vector<Coordinate<T> > & data);     ///< Sets the sample data.
    void setFittingModel(Fit<T> & model);                       ///< Sets the \ref Fit "fitting model".

private:

    Fit<T> * model;
};


/** \brief Constructs an instance of RansacGenericFittingModel.
  *
  * \param [in] model   A \ref Fit "fitting class".
  */

template<class T>
RansacGenericFittingModel<T>::RansacGenericFittingModel(Fit<T> & model) :
    model(&model)
{
}


template<class T>
RansacGenericFittingModel<T>::~RansacGenericFittingModel()
{
}


template<class T>
void RansacGenericFittingModel<T>::compute()
{
    model->compute();
}


template<class T>
T RansacGenericFittingModel<T>::getDeviation(const Coordinate<T> & datum) const
{
    return model->getDistanceTo(datum);
}


template<class T>
unsigned RansacGenericFittingModel<T>::getMinimumNumberOfItems() const
{
    return model->getNumberPointsRequired();
}


template<class T>
T RansacGenericFittingModel<T>::getRMS() const
{
    return model->getRMS();
}


template<class T>
void RansacGenericFittingModel<T>::setData(const std::vector<Coordinate<T> > & data)
{
    model->setPoints(data);
}


template<class T>
void RansacGenericFittingModel<T>::setFittingModel(Fit<T> & model)
{
    this->model = &model;
}


} // namespace TRTK


#endif // RANSAC_GENERIC_FITTING_MODEL_HPP_2437670324
