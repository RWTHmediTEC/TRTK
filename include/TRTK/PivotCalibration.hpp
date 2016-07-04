/*
    This class performs a pivot calibration providing a defined translation
    between a tool tip and its sensor system.

    Copyright (C) 2010 - 2016 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.2.0 (2012-02-03)
*/

/** \file PivotCalibration.hpp
  * \brief This file contains the \ref TRTK::PivotCalibration
  *        "PivotCalibration" class.
  */


#ifndef PIVOTCALIBRATION_HPP_3123933941
#define PIVOTCALIBRATION_HPP_3123933941

#include <vector>

#include<Eigen/StdVector>

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "FitSphere.hpp"
#include "Range.hpp"
#include "Transform3D.hpp"


namespace TRTK
{


////////////////////////////////////////////////////////////////////////////////
//                                 Interface                                  //
////////////////////////////////////////////////////////////////////////////////


template <class T>
class PivotCalibration
{
public:
    enum Error {
        NOT_ENOUGH_INPUT_DATA,
        UNEQUAL_CARDINALITY_OF_INPUT_SETS,
        UNKNOWN_ERROR
    };

    typedef T value_type;
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Matrix<T, 4, 1> Vector4T;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXT;
    typedef Eigen::Matrix<T, 3, 3> Matrix3T;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXT;
    typedef Coordinate<T> Point;

    PivotCalibration() {};
    virtual ~PivotCalibration() {};

    virtual T compute() = 0; ///< Returns the RMSE.
    virtual T getRMSE() const = 0;
    virtual const Vector3T & getTranslation() const = 0;
    virtual void setLocations(Range<Vector3T> locations) = 0;
    virtual void setRotations(Range<Matrix3T> rotations) = 0;
};


////////////////////////////////////////////////////////////////////////////////
//                          PivotCalibrationTwoStep                           //
////////////////////////////////////////////////////////////////////////////////


template <class T>
class PivotCalibrationTwoStep : public PivotCalibration<T>
{
private:
    typedef PivotCalibration<T> super;

public:
    typedef T value_type;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::Matrix3T Matrix3T;
    typedef typename super::MatrixXT MatrixXT;
    typedef typename super::Point Point;

    PivotCalibrationTwoStep();
    ~PivotCalibrationTwoStep();

    T compute(); ///< Returns the RMSE.
    T getRMSE() const;
    const Vector3T & getTranslation() const;
    void setLocations(Range<Vector3T> locations);
    void setRotations(Range<Matrix3T> rotations);

private:
    std::vector<Vector3T> locations;
    std::vector<Matrix3T> rotations;
    Vector3T translation;
    T rmse;
};


template <class T>
PivotCalibrationTwoStep<T>::PivotCalibrationTwoStep() : rmse(T(0))
{
}


template <class T>
PivotCalibrationTwoStep<T>::~PivotCalibrationTwoStep()
{
}


template <class T>
T PivotCalibrationTwoStep<T>::compute()
{
    /*

    Explanation of the algorithm

    As described in the details section, during the calibration procedure,
    the tool is moved around its tip. In doing so, the tool tip always resides
    a the same point, namely the pivot point. At the same time the tool's
    sensor moves along the surface of a fictive sphere. Now, in a first step,
    the center point (= pivot point) as well as the radius of this sphere are
    estimated by a simple least square scheme (in global coordinates).

    Now, the location of the tip in the local sensor coordinate system can
    be easily computed from a single measurement (step two). It holds, that a
    point p in the local sensor coordinate system corresponds to the point p'
    in global coordinates by

        p' = R * p + t

    where R is the rotation and t the location of the sensor in the global
    coordinate system. Since the pivot point is known in global coordinates
    (let it be p') and the tool tip resides at this location, the location
    of the tip in local coordinates is

        p = R^(-1) * (p' - t)

    Using a single measurement (R_i, t_i) and computing the sought translation,
    we were actually done. But due to noise, we should take the mean of several
    estimated translations.

    */

    // Estimate sphere parameters

    FitSphere<T> fitSphere(locations);
    fitSphere.compute();
    Vector3T centerPoint = fitSphere.getCenterPoint().toArray();

    // Compute the difference vector, transform it into the sensor system and average it

    translation = Vector3T(0, 0, 0);

    const int n = rotations.size();

    for (int i = 0; i < n; ++i)
    {
        Vector3T differenceVectorGlobal = centerPoint - locations[i];

        // Transform the vector into the local sensor coordinate system
        Vector3T differenceVectorLocal = rotations[i].inverse() * differenceVectorGlobal;

        // Average the found translation vector (part 1)
        translation += differenceVectorLocal;
    }

    // Average the found translation vector (part 2)
    translation = translation / n;

    // Compute the RMSE

    rmse = T(0);
    for (int i = 0; i < n; ++i)
    {
        Vector3T centerPointLocal = rotations[i].inverse() * (centerPoint - locations[i]);
        T squaredError = (translation - centerPointLocal).squaredNorm();
        rmse += squaredError;
    }
    using std::sqrt;
    rmse = sqrt(rmse / n);

    return rmse;
}


template <class T>
T PivotCalibrationTwoStep<T>::getRMSE() const
{
    return rmse;
}


template <class T>
const typename PivotCalibrationTwoStep<T>::Vector3T & PivotCalibrationTwoStep<T>::getTranslation() const
{
    return translation;
}


template <class T>
void PivotCalibrationTwoStep<T>::setLocations(Range<Vector3T> locations)
{
    this->locations.resize(locations.size());
    size_t i = 0;

    while(!locations.isDone())
    {
        this->locations[i++] = locations.currentItem();
        locations.next();
    }
}


template <class T>
void PivotCalibrationTwoStep<T>::setRotations(Range<Matrix3T> rotations)
{
    this->rotations.resize(rotations.size());
    size_t i = 0;

    while(!rotations.isDone())
    {
        this->rotations[i++] = rotations.currentItem();
        rotations.next();
    }
}


} // namespace TRTK


#endif // PIVOTCALIBRATION_HPP_3123933941
