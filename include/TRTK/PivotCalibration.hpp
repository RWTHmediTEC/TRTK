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

#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

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
    typedef Eigen::Matrix<T, 4, 4> Matrix4T;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXT;
    typedef Coordinate<T> Point;
    typedef std::pair<Vector3T, Matrix3T> DataType;


    PivotCalibration() {};
    virtual ~PivotCalibration() {};

    virtual size_t getNumberItemsRequired() const = 0;              ///< Returns the minimum number of locations and rotations needed for the algorithm.
    virtual void setLocations(Range<Vector3T> locations) = 0;
    virtual void setRotations(Range<Matrix3T> rotations) = 0;
    virtual T compute() = 0;                                        ///< Returns the RMSE.
    virtual T getRMSE() const = 0;
    virtual const Vector3T & getLocalPivotPoint() const = 0;        ///< Returns the pivot point (or tool tip) in local coordinates.
    virtual const Vector3T & getPivotPoint() const = 0;             ///< Returns the pivot point (or tool tip) in world coordinates.
};


////////////////////////////////////////////////////////////////////////////////
//                    PivotCalibrationCombinatorialApproach                   //
////////////////////////////////////////////////////////////////////////////////


template <class T>
class PivotCalibrationCombinatorialApproach : public PivotCalibration<T>
{
private:
    typedef PivotCalibration<T> super;

public:
    typedef T value_type;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::Matrix3T Matrix3T;
    typedef typename super::Matrix4T Matrix4T;
    typedef typename super::MatrixXT MatrixXT;
    typedef typename super::Point Point;
    typedef typename super::DataType DataType;

    PivotCalibrationCombinatorialApproach();
    ~PivotCalibrationCombinatorialApproach();

    size_t getNumberItemsRequired() const;              ///< Returns the minimum number of locations and rotations needed for the algorithm.
    void setLocations(Range<Vector3T> locations);
    void setRotations(Range<Matrix3T> rotations);
    T compute();                                        ///< Returns the RMSE.
    T getRMSE() const;
    const Vector3T & getPivotPoint() const;             ///< Returns the pivot point (or tool tip) in world coordinates.
    const Vector3T & getLocalPivotPoint() const;        ///< Returns the pivot point (or tool tip) in local coordinates.

private:
    std::vector<Vector3T> locations;
    std::vector<Matrix3T> rotations;
    Vector3T global_pivot_point;                        ///< Pivot point (or tool tip) in world coordinates.
    Vector3T local_pivot_point;                         ///< Pivot point (or tool tip) in local coordinates.
    T rmse;
};


template <class T>
PivotCalibrationCombinatorialApproach<T>::PivotCalibrationCombinatorialApproach() : rmse(T(0))
{
}


template <class T>
PivotCalibrationCombinatorialApproach<T>::~PivotCalibrationCombinatorialApproach()
{
}


template <class T>
T PivotCalibrationCombinatorialApproach<T>::compute()
{
    /*

    Explanation of the algorithm

    If a tool touches the pivot point with its tip, the pivot point's location M
    in global coordinates is

    M = R_i * t + t_i

    where 'R_i' is the rotation and 't_i' the location of the tool sensor in the world or
    tracking coordinate system. 't' is the sought location of the tool tip in the sensor's
    local coordinate system. Note, that 't' remains the same for all measurements! Now,
    using two different measurements, the pivot point M can be eliminated

    R_i * t + t_i = R_j * t + t_j

    This can be rearranged to

    (R_i - R_j) * t = (t_j - t_i)

    Doing this for all combinations of all measurements yields a system of equations
    Ax = b (with x = t) which can be solved for the sought translation 't'.

    | R1 - R2   |         | t2   - t1 |
    | R1 - R3   |         | t3   - t1 |
    | R1 - R4   | * t  =  | t4   - t1 |      <==>    At = b
    |   ...     |         |     ...   |
    | Rn - Rn-1 |         | tn-1 - tn |

    t = [A^T * A]^(-1) * A^T * B        (Moore–Penrose pseudoinverse)

    Note: Not all combinations are taken into account. For instance, the case i == j
    is omitted.

    */

    const int n = rotations.size();
    const int max_number_of_combinations = n * (n - 1);
    int current_index = 0; // i-th entry (or "row") in A and b

    // Build up A and b and solve the system of equations

    MatrixXT A(3 * max_number_of_combinations, 3);
    MatrixXT b(3 * max_number_of_combinations, 1);

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; j++)
        {
            if (i == j || rotations[i].isApprox(rotations[j]))
            {
                continue;
            }

            A.block(current_index * 3, 0, 3, 3) = rotations[i] - rotations[j];
            b.block(current_index * 3, 0, 3, 1) = locations[j] - locations[i];

            ++current_index;
        }

    }

    A.resize(current_index * 3, 3);
    b.resize(current_index * 3, 1);

    local_pivot_point = (A.transpose() * A).inverse() * (A.transpose() * b);

    // Compute an averaged global pivot point

    global_pivot_point = Vector3T(0, 0, 0);

    for (int i = 0; i < n; ++i)
    {
        global_pivot_point += rotations[i] * local_pivot_point + locations[i];
    }

    global_pivot_point /= n;

    // Compute the RMSE

    using std::sqrt;
    rmse = T(0);

    for (int i = 0; i < n; ++i)
    {
        Vector3T noisy_center_point = rotations[i] * local_pivot_point + locations[i];
        rmse += (noisy_center_point - global_pivot_point).squaredNorm();
    }

    rmse = sqrt(rmse / n);

    return rmse;
}


template <class T>
const typename PivotCalibrationCombinatorialApproach<T>::Vector3T & PivotCalibrationCombinatorialApproach<T>::getLocalPivotPoint() const
{
    return local_pivot_point;
}


template <class T>
size_t PivotCalibrationCombinatorialApproach<T>::getNumberItemsRequired() const
{
    return 4;
}


template <class T>
const typename PivotCalibrationCombinatorialApproach<T>::Vector3T & PivotCalibrationCombinatorialApproach<T>::getPivotPoint() const
{
    return global_pivot_point;
}


template <class T>
T PivotCalibrationCombinatorialApproach<T>::getRMSE() const
{
    return rmse;
}


template <class T>
void PivotCalibrationCombinatorialApproach<T>::setLocations(Range<Vector3T> locations)
{
    this->locations.resize(locations.size());
    size_t i = 0;

    while (!locations.isDone())
    {
        this->locations[i++] = locations.currentItem();
        locations.next();
    }
}


template <class T>
void PivotCalibrationCombinatorialApproach<T>::setRotations(Range<Matrix3T> rotations)
{
    this->rotations.resize(rotations.size());
    size_t i = 0;

    while (!rotations.isDone())
    {
        this->rotations[i++] = rotations.currentItem();
        rotations.next();
    }
}


////////////////////////////////////////////////////////////////////////////////
//                            PivotCalibrationPATM                            //
////////////////////////////////////////////////////////////////////////////////


template <class T>
class PivotCalibrationPATM : public PivotCalibration<T>
{
private:
    typedef PivotCalibration<T> super;

public:
    typedef T value_type;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::Vector4T Vector4T;
    typedef typename super::VectorXT VectorXT;
    typedef typename super::Matrix3T Matrix3T;
    typedef typename super::Matrix4T Matrix4T;
    typedef typename super::MatrixXT MatrixXT;
    typedef typename super::Point Point;
    typedef typename super::DataType DataType;

    typedef Eigen::Matrix<std::complex<T>, 3, 3> Matrix3cT;
    typedef Eigen::Matrix<std::complex<T>, 4, 4> Matrix4cT;

    PivotCalibrationPATM();
    ~PivotCalibrationPATM();

    size_t getNumberItemsRequired() const;              ///< Returns the minimum number of locations and rotations needed for the algorithm.
    void setLocations(Range<Vector3T> locations);
    void setRotations(Range<Matrix3T> rotations);
    void setNumberIterations(int count);                ///< Sets the number of fixed-point iterations. the default value is 500.
    T compute();                                        ///< Returns the RMSE.
    T getRMSE() const;
    const Vector3T & getPivotPoint() const;             ///< Returns the pivot point (or tool tip) in world coordinates.
    const Vector3T & getLocalPivotPoint() const;        ///< Returns the pivot point (or tool tip) in local coordinates.

private:
    unsigned number_of_iterations;
    std::vector<Vector3T> locations;
    std::vector<Matrix3T> rotations;
    Vector3T global_pivot_point;                        ///< Pivot point (or tool tip) in world coordinates.
    Vector3T local_pivot_point;                         ///< Pivot point (or tool tip) in local coordinates.
    T rmse;
};


template <class T>
PivotCalibrationPATM<T>::PivotCalibrationPATM() : number_of_iterations(500), rmse(T(0))
{
}


template <class T>
PivotCalibrationPATM<T>::~PivotCalibrationPATM()
{
}


template <class T>
T PivotCalibrationPATM<T>::compute()
{
    /*

    Explanation of the algorithm

    If a tool touches the pivot point with its tip, the pivot point's location M
    in global coordinates is

    M = R_i * t + t_i

    where 'R_i' is the rotation and 't_i' the location of the tool sensor in the world or
    tracking coordinate system. 't' is the sought location of the tool tip in the sensor's
    local coordinate system. Note, that 't' remains the same for all measurements! Now,
    using two different measurements, the pivot point M can be eliminated

    R_i * t + t_i = R_j * t + t_j

    This can be rearranged to

    t = R_i^T * R_j * t + R_i^T * (t_j - t_i) =: phi(t)                               (*)

    This is actually the form of a fixed-point iteration, i.e. t_i+1 = phi(t_i). However,
    in this form phi is not a contraction mapping since the Lipschitz constant L is not
    strictly less than 1:

    || phi(x) - phi(y) || = || R_i^T * R_j * (x - y) || = L * || x - y||,   L = 1

    This can be changed by averaging at least two equations (*) with different
    measurements. Thus we construct phi(t) as follows

    phi(t) := R * t + p                                                              (**)

    where R := 1/n * sum_i R_i^T * R_(i+1),   p := 1/n * sum_i R_i^T * (t_(i+1) - t_i),
    and n is the number of measurements. Since the sum goes up to n, R_(n+1) and t_(n1+1)
    are set to R_1 and t_1, respectively.

    The equation (**) can also rewritten as

             | R  p |
    phi(t) = |      | * t = A * t
             | 0  1 |

    and thus t_i+1 = phi(t_i) = A * t_i = A^i * t0.

    */

    // Build up R and p

    Matrix3T R = Matrix3T::Zero();
    Vector3T p = Vector3T::Zero();

    const int n = rotations.size();

    for (int i = 0; i < n; ++i)
    {
        int k = i % n; // 0, 1, 2, ..., n-1, n
        int l = (i + 1) % n; // 1, 2, 3, ..., n, 0

        R += rotations[k].transpose() * rotations[l];
        p += rotations[k].transpose() * (locations[l] - locations[k]);
    }

    R /= n;
    p /= n;

    Matrix4T A = Matrix4T::Identity();
    A.block<3, 3>(0, 0) = R;
    A.block<3, 1>(0, 3) = p;

    // Carry out the fixed-point iterations

    Vector4T t = Vector4T(1, 1, 1, 1);

    if (number_of_iterations > 10)
    {
        // Compute the power of A
        Eigen::EigenSolver<MatrixXT> solver(A);
        Matrix4cT D = solver.eigenvalues().asDiagonal();
        Matrix4cT V = solver.eigenvectors();
        A = (V * D.array().pow(number_of_iterations).matrix() * V.inverse()).real();
        t = A * t;
    }
    else
    {
        for (unsigned i = 0; i < number_of_iterations; ++i)
        {
            t = A * t;
        }
    }

    local_pivot_point = t.head<3>();

    // Compute an averaged global pivot point

    global_pivot_point = Vector3T(0, 0, 0);

    for (int i = 0; i < n; ++i)
    {
        global_pivot_point += rotations[i] * local_pivot_point + locations[i];
    }

    global_pivot_point /= n;

    // Compute the RMSE

    using std::sqrt;
    rmse = T(0);

    for (int i = 0; i < n; ++i)
    {
        Vector3T noisy_center_point = rotations[i] * local_pivot_point + locations[i];
        rmse += (noisy_center_point - global_pivot_point).squaredNorm();
    }

    rmse = sqrt(rmse / n);

    return rmse;
}


template <class T>
const typename PivotCalibrationPATM<T>::Vector3T & PivotCalibrationPATM<T>::getLocalPivotPoint() const
{
    return local_pivot_point;
}


template <class T>
size_t PivotCalibrationPATM<T>::getNumberItemsRequired() const
{
    return 3;
}


template <class T>
const typename PivotCalibrationPATM<T>::Vector3T & PivotCalibrationPATM<T>::getPivotPoint() const
{
    return global_pivot_point;
}


template <class T>
T PivotCalibrationPATM<T>::getRMSE() const
{
    return rmse;
}


template <class T>
void PivotCalibrationPATM<T>::setLocations(Range<Vector3T> locations)
{
    this->locations.resize(locations.size());
    size_t i = 0;

    while (!locations.isDone())
    {
        this->locations[i++] = locations.currentItem();
        locations.next();
    }
}


template <class T>
void PivotCalibrationPATM<T>::setNumberIterations(int count)
{
    number_of_iterations = count;
}


template <class T>
void PivotCalibrationPATM<T>::setRotations(Range<Matrix3T> rotations)
{
    this->rotations.resize(rotations.size());
    size_t i = 0;

    while (!rotations.isDone())
    {
        this->rotations[i++] = rotations.currentItem();
        rotations.next();
    }
}


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
    typedef typename super::DataType DataType;

    PivotCalibrationTwoStep();
    ~PivotCalibrationTwoStep();

    size_t getNumberItemsRequired() const;              ///< Returns the minimum number of locations and rotations needed for the algorithm.
    void setLocations(Range<Vector3T> locations);
    void setRotations(Range<Matrix3T> rotations);
    T compute();                                        ///< Returns the RMSE.
    T getRMSE() const;
    const Vector3T & getPivotPoint() const;             ///< Returns the pivot point (or tool tip) in world coordinates.
    const Vector3T & getLocalPivotPoint() const;        ///< Returns the pivot point (or tool tip) in local coordinates.

private:
    std::vector<Vector3T> locations;
    std::vector<Matrix3T> rotations;
    Vector3T global_pivot_point;                        ///< Pivot point (or tool tip) in world coordinates.
    Vector3T local_pivot_point;                         ///< Pivot point (or tool tip) in local coordinates.
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
    an object is moved around a pivot point. For example a tool is moved in
    such a way that the tool tip remains stationary at a particular position
    (which is the pivot point). While moving the object (e.g. the tool) the
    object's sensor moves along a circular arc (or in three dimensions on a
    calotte of a sphere). Now, in a first step, the center point (= pivot
    point) as well as the radius of this sphere are estimated by a simple
    least square scheme (that's done in global coordinates).

    Provided the object is still positioned as described above (e.g. the tool
    tip still is stationary), using a single measurement (R_i, t_i) the
    global pivot point p' can be easily transformed into a local coordinate
    p via the relation

        p' = R * p + t    ==>    p = R^(-1) * (p' - t)

    where R is the rotation and t the location of the sensor in the global
    coordinate system. However, due to noise, various results for p should
    be averaged.

    Implementation details:

    Instead of recording a new set of pairs of (R_i, t_i) after having
    determined the virtual sphere, the former measurement is used.

    */

    // Estimate sphere parameters

    FitSphere<T> fitSphere(locations);
    fitSphere.compute();
    global_pivot_point = fitSphere.getCenterPoint().toArray();

    // Compute the local pivot point

    local_pivot_point = Vector3T(0, 0, 0);

    const int n = rotations.size();

    for (int i = 0; i < n; ++i)
    {
        Vector3T differenceVectorLocal = rotations[i].inverse() * (global_pivot_point - locations[i]);
        local_pivot_point += differenceVectorLocal; // average (part 1)
    }

    local_pivot_point = local_pivot_point / n; // average (part 2)

    // Compute the RMSE

    using std::sqrt;
    rmse = T(0);

    for (int i = 0; i < n; ++i)
    {
        Vector3T noisy_center_point = rotations[i] * local_pivot_point + locations[i];
        rmse += (noisy_center_point - global_pivot_point).squaredNorm();
    }

    rmse = sqrt(rmse / n);

    return rmse;
}


template <class T>
const typename PivotCalibrationTwoStep<T>::Vector3T & PivotCalibrationTwoStep<T>::getLocalPivotPoint() const
{
    return local_pivot_point;
}


template <class T>
size_t PivotCalibrationTwoStep<T>::getNumberItemsRequired() const
{
    return 4;
}


template <class T>
const typename PivotCalibrationTwoStep<T>::Vector3T & PivotCalibrationTwoStep<T>::getPivotPoint() const
{
    return global_pivot_point;
}


template <class T>
T PivotCalibrationTwoStep<T>::getRMSE() const
{
    return rmse;
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
