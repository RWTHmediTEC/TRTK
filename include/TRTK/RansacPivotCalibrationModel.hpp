/*
    Copyright (C) 2010 - 2016 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.0 (2016-07-25)
*/

/** \file RansacPivotCalibrationModel.hpp
  * \brief This file contains the \ref TRTK::RansacPivotCalibrationModel "RansacPivotCalibrationModel" class.
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
  * #include <cmath>
  * #include <cstdlib>
  * #include <iostream>
  * #include <iomanip>
  * #include <utility>
  * #include <vector>
  * 
  * #include<Eigen/StdVector>
  * 
  * #include <TRTK/Clock.hpp>
  * #include <TRTK/Coordinate.hpp>
  * #include <TRTK/Tools.hpp>
  * #include <TRTK/Transform3D.hpp>
  * #include <TRTK/PivotCalibration.hpp>
  * #include <TRTK/RansacPivotCalibrationModel.hpp>
  * 
  * 
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
  * 
  * double pi = 3.1415926535;
  * 
  * typedef PivotCalibration<double> Calibration;
  * typedef Calibration::value_type value_type;
  * typedef Calibration::Matrix3T Matrix;
  * typedef Calibration::Vector3T Vector;
  * 
  * 
  * namespace
  * {
  * 
  *     struct GenerateTestData
  *     {
  *         GenerateTestData(double sigma = 0)
  *         {
  *             // Generate some (noisy) test data.
  * 
  *             // We assume, there is a tool which is aligned with the x-axis. Its tool
  *             // tip is located at the (global) point p (pivot point), and the center of
  *             // the tool coordinate system is located at the point t. The local tool tip
  *             // position is defined to be (a, 0, 0).
  *             //
  *             // The relation between the local coordinate system and the global coordinate
  *             // system is given by p_global = R * p_local + t. The initial rotation R of
  *             // the tool coordinate system is a rotation of pi/2 in the x-y plane.
  *             //
  *             // Now the tool is rotated, where the tool tip remains at the same position.
  * 
  *             Matrix R;
  *             R << cos(pi/2), -sin(pi/2), 0,
  *                  sin(pi/2),  cos(pi/2), 0,
  *                          0,          0, 1;
  * 
  *             double a = 70;
  *             p_local = Vector(a, 0, 0);
  *             p = Vector(1, 1, 1);
  *             Vector t = p - R * p_local;
  * 
  *             for (double theta = -0.8 * pi / 2; theta < 0.8 * pi / 2; theta += pi / 20)
  *             {
  *                 for (double phi = -0.8 * pi / 2; phi < 0.8 * pi / 2; phi += pi / 20)
  *                 {
  *                     // Rotate around pivot point P.
  * 
  *                     const double x = p.x();
  *                     const double y = p.y();
  *                     const double z = p.z();
  * 
  *                     Transform3D<double> transform;
  *                     transform.translate(-x, -y, -z).rotateZ(phi).rotateY(theta).translate(x, y, z);
  * 
  *                     Vector location = transform * t;
  *                     Matrix rotation = transform.getTransformationMatrix().block(0, 0, 3, 3) * R;
  * 
  *                     // Add some noise
  * 
  *                     using TRTK::Tools::randn;
  *                     location += sigma * Vector(randn(), randn(), randn());
  * 
  *                     // Store the results
  * 
  *                     locations.push_back(location);
  *                     rotations.push_back(rotation);
  *                 }
  *             }
  *         }
  * 
  *         vector<Matrix> rotations;
  *         vector<Vector> locations;
  *         Vector p;
  *         Vector p_local;
  *     };
  * 
  * } // end of anonymous namespace
  * 
  * 
  * int main()
  * {
  *     cout << setprecision(4);
  *     cout << fixed;
  * 
  *     cout << "Pivot calibration example" << endl;
  *     cout << "-------------------------" << endl << endl;
  * 
  *     GenerateTestData test_data;
  *     cout << "Ground truth" << endl;
  *     cout << "Pivot point: " << test_data.p.transpose() << endl;
  *     cout << "Local pivot point: " << test_data.p_local.transpose() << endl << endl << endl;
  * 
  *     {
  *         Clock clock;
  *         srand(0);
  *         GenerateTestData test_data;
  * 
  *         PivotCalibrationTwoStep<double> calibration;
  *         calibration.setLocations(make_range(test_data.locations));
  *         calibration.setRotations(make_range(test_data.rotations));
  *         double rmse = calibration.compute();
  * 
  *         cout << "No noise" << endl;
  *         cout << "RMSE: " << rmse << endl;
  *         cout << "Global pivot point: " << calibration.getPivotPoint().transpose() << "\t";
  *         cout << "Error: " << (calibration.getPivotPoint() - test_data.p).norm() << endl;
  *         cout << "Local pivot point: " << calibration.getLocalPivotPoint().transpose() << "\t";
  *         cout << "Error: " << (calibration.getLocalPivotPoint() - test_data.p_local).norm() << endl;
  *         cout << clock << endl;
  *     }
  * 
  *     {
  *         Clock clock;
  *         srand(0);
  *         GenerateTestData test_data(0.1);
  * 
  *         PivotCalibrationTwoStep<double> calibration;
  *         calibration.setLocations(make_range(test_data.locations));
  *         calibration.setRotations(make_range(test_data.rotations));
  *         double rmse = calibration.compute();
  * 
  *         cout << "With noise" << endl;
  *         cout << "RMSE: " << rmse << endl;
  *         cout << "Global pivot point: " << calibration.getPivotPoint().transpose() << "\t";
  *         cout << "Error: " << (calibration.getPivotPoint() - test_data.p).norm() << endl;
  *         cout << "Local pivot point: " << calibration.getLocalPivotPoint().transpose() << "\t";
  *         cout << "Error: " << (calibration.getLocalPivotPoint() - test_data.p_local).norm() << endl;
  *         cout << clock << endl;
  *     }
  * 
  *     {
  *         Clock clock;
  *         srand(0);
  *         GenerateTestData test_data(0.1);
  *         vector<pair<Vector, Matrix> > data = zip(test_data.locations, test_data.rotations);
  * 
  *         PivotCalibrationTwoStep<double> calibration;
  *         RansacPivotCalibrationModel<double> model(calibration);
  *         Ransac<double, PivotCalibration<double>::DataType> ransac;
  * 
  *         ransac.setModel(model);
  *         ransac.setData(data);
  *         ransac.setErrorTolerance(0.2);
  * 
  *         unsigned number_of_samples_used = ransac.compute();
  * 
  *         cout << "RANSAC and noise" << endl;
  *         cout << "RMSE: " << model.getRMSE() << endl;
  *         cout << "Number of samples used: " << number_of_samples_used << endl;
  *         cout << "Global pivot point: " << calibration.getPivotPoint().transpose() << "\t";
  *         cout << "Error: " << (calibration.getPivotPoint() - test_data.p).norm() << endl;
  *         cout << "Local pivot point: " << calibration.getLocalPivotPoint().transpose() << "\t";
  *         cout << "Error: " << (calibration.getLocalPivotPoint() - test_data.p_local).norm() << endl;
  *         cout << clock << endl;
  *     }
  * 
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \verbatim
  * Pivot calibration example
  * -------------------------
  * 
  * Ground truth
  * Pivot point: 1.0000 1.0000 1.0000
  * Local pivot point: 70.0000  0.0000  0.0000
  * 
  * 
  * No noise
  * RMSE: 0.0000
  * Global pivot point: 1.0000 1.0000 1.0000        Error: 0.0000
  * Local pivot point: 70.0000 -0.0000  0.0000      Error: 0.0000
  * Elapsed time: 0.0390 seconds.
  * 
  * With noise
  * RMSE: 0.1752
  * Global pivot point: 0.9918 1.0086 1.0184        Error: 0.0219
  * Local pivot point: 70.0125  0.0018  0.0190      Error: 0.0228
  * Elapsed time: 0.0350 seconds.
  * 
  * RANSAC and noise
  * RMSE: 0.1320
  * Number of samples used: 202
  * Global pivot point: 0.9856 1.0058 1.0158        Error: 0.0221
  * Local pivot point: 70.0144  0.0271  0.0202      Error: 0.0368
  * Elapsed time: 0.6570 seconds.
* \endverbatim
  *
  * For further information, please have a look at the documentation of the particular
  * \ref PivotCalibration "Pivot calibration class" and the \ref Ransac class, respectively.
  *
  * \see Ransac, Ransac::Model, PivotCalibration
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2016-07-25
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


/** \brief Constructs an instance of RansacPivotCalibrationModel.
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
