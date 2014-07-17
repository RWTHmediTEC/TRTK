/*
    This class performs a pivot calibration providing a defined translation
    between a tool tip and its sensor system.

    Copyright (C) 2010 - 2014 Christoph Haenisch

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
#include "Transform3D.hpp"


namespace TRTK
{


/** \tparam T scalar type
  *
  * \brief This class performs a pivot calibration providing a defined
  *        translation between a tool tip and its sensor system.
  *
  * This class provides means to determine the location of a tool tip within
  * the instrument's local sensor coordinate system. The calibration procedure
  * to obtain the location is as follows: The tool is rotated around a pivot
  * point always touching this point with its tip. The location as well as the
  * rotation of the sensor system is saved for each sampling instance. Then
  * this list is given to PivotCalibration and the sought location/translation
  * is computed.
  * 
  * \note We always assume, that the transformations map from a local coordinate
  *       system to the global coordinate system. That is, given a rotation matrix
  *       \f$  R \f$  and a translation vector \f$ t \f$ describing the orientation
  *       and the location of a sensor system, respectively, the vector \f$ p' \f$
  *       in global coordinates corresponds to the vector \f$ p \f$ in local
  *       coordinates by this relation:
  *       \f[
  *       p' = Rp + t
  *       \f]
  *
  * The following two \ref Algorithm "algorithms" are available:
  *  - \ref TWO_STEP_PROCEDURE (default)
  *  - \ref COMBINATORICAL_APPROACH
  *
  * Here is an example of how to use the class:
  *
  * \code
  * typedef TRTK::PivotCalibration<double> Calibration;
  * typedef TRTK::Calibration::Matrix3T Matrix;
  * typedef TRTK::Calibration::Vector3T Vector;
  *
  * // These rotation matrices and translation vectors describe the local
  * // sensor coordinate systems in global coordinates
  *
  * std::vector<Matrix> rotations;
  * std::vector<Vector> locations;
  *
  * while (true)
  * {
  *     // record the data and break if enough data was collected
  *
  *     // ...
  *
  *     // save the rotation matrix and the translation vector...
  *
  *     Matrix R;
  *     R << r11, r12, r13, r21, r22, r23, r31, r32, r33;
  *
  *     Vector t(t1, t2, t3);
  *
  *     rotations.push_back(R);
  *     locations.push_back(t);
  * }
  *
  * Calibration pivotCalibration(rotations, locations);
  *
  * pivotCalibration.compute();
  *
  * Vector translation = pivotCalibration.getTranslation(); // in local coordinates
  *
  * // ...
  *
  * // Determinte the current tool tip position in global coordinates.
  * Vector tool_tip_position = current_rotation * translation + current_position;
  * \endcode
  *
  * \note See http://eigen.tuxfamily.org/ for how to use the Matrix class.
  *
  * \see Coordinate and Transform3D
  *
  * \author Christoph Haenisch
  * \version 0.2.0
  * \date last changed on 2012-02-03
  */

template <class T>
class PivotCalibration
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum Algorithm {
        TWO_STEP_PROCEDURE,         //!< First, estimate the pivot point in global coordinates. Second, estimate the sought translation by computing the difference vector between the pivot point and the center of the sensor system; then take the mean.\n This algorithm is advantageous in the case of zero-mean noise.
        COMBINATORICAL_APPROACH     //!< Setup n equations that yield the pivot point from a single measurement using the sought (but unknown) translation. Eliminate the pivot point in the system of equations and finally solve for the sought translation. \n This algorithm is advantageous in the case of non-zero-mean noise (e.g. a systematic error).
    };

    enum Error {
        NOT_ENOUGH_INPUT_DATA,
        UNEQUAL_CARDINALITY_OF_SETS,
        UNKNOWN_ALGORITHM,
        UNKNOWN_ERROR,
        WRONG_VECTOR_SIZE
    };

    typedef T value_type;
    typedef Eigen::Matrix<T, 3, 1> Vector3T;
    typedef Eigen::Matrix<T, 4, 1> Vector4T;
    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXT;
    typedef Eigen::Matrix<T, 3, 3> Matrix3T;
    typedef Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> MatrixXT;

    PivotCalibration();
    PivotCalibration(const std::vector<Matrix3T> & rotations, const std::vector<Vector3T> & locations);
    PivotCalibration(const std::vector<Transform3D<T>, Eigen::aligned_allocator<Transform3D<T> > > & transformations);

    virtual ~PivotCalibration();

    void compute();

    const Vector3T & getTranslation() const;

    void setAlgorithm(Algorithm algorithm);

    void setLocations(const std::vector<Coordinate<T> > & locations);
    void setLocations(const std::vector<Vector3T> & locations);
    void setLocations(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & locations);

    void setRotations(const std::vector<Matrix3T> & matrices);

    void setTransformations(const std::vector<Transform3D<T>, Eigen::aligned_allocator<Transform3D<T> > > & transformations);

private:
    void compute_two_step_procedure();
    void compute_combinatorical_approach();

    Vector3T translation;
    std::vector<Matrix3T> rotations;
    std::vector<Vector3T> locations;
    Algorithm algorithm;
};


/** \tparam T scalar type
  *
  * Constructs an empty PivotCalibration object.
  */

template <class T>
PivotCalibration<T>::PivotCalibration() :
    algorithm(TWO_STEP_PROCEDURE)
{
}


/** \tparam T scalar type
  *
  * \param [in] rotations rotations of the sensor systems
  * \param [in] locations locations of the sensor systems
  *
  * Constructs a PivotCalibration object.
  *
  * \see setRotations() and setLocations
  */

template <class T>
PivotCalibration<T>::PivotCalibration(const std::vector<Matrix3T> & rotations,
                                      const std::vector<Vector3T> & locations) :
    algorithm(TWO_STEP_PROCEDURE)
{
    this->rotations = rotations;
    this->locations = locations;
}


/** \tparam T scalar type
  *
  * \param [in] transformations rotations and locations of the sensor systems
  *
  * Constructs a PivotCalibration object.
  *
  * \see setTransformations()
  */

template <class T>
PivotCalibration<T>::PivotCalibration(const std::vector<Transform3D<T>, Eigen::aligned_allocator<Transform3D<T> > > & transformations) :
    algorithm(TWO_STEP_PROCEDURE)
{
    rotations.clear();
    locations.clear();

    for (unsigned i = 0; i < transformations.size(); ++i)
    {
        Matrix3T m = transformations[i].getTransformationMatrix().block(0, 0, 3, 3);
        Vector3T v = transformations[i].getTransformationMatrix().block(0, 3, 3, 1);

        rotations.push_back(m);
        locations.push_back(v);
    }
}


/** \tparam T scalar type
  *
  * Destructs the PivotCalibration object.
  */

template <class T>
PivotCalibration<T>::~PivotCalibration()
{
}


/** \tparam T scalar type
  *
  * Computes the sought translation vector.
  *
  * \throw ErrorObj If the number of rotations set by \c setRotations() diverges
  *                 from the number of locations set by \c setLocations(), an
  *                 error object is thrown and its error code is set to
  *                 \c UNEQUAL_CARDINALITY_OF_SETS.
  *
  * \see getTranslation()
  */

template <class T>
void PivotCalibration<T>::compute()
{
    // check for valid input data

    if (locations.size() != rotations.size())
    {
        ErrorObj error;
        error.setErrorMessage("The number of rotations is different to the number of locations.");
        error.setClassName("PivotCalibration");
        error.setFunctionName("compute()");
        error.setErrorCode(UNEQUAL_CARDINALITY_OF_SETS);
        throw error;
    }

    switch (algorithm)
    {
        case TWO_STEP_PROCEDURE:
            compute_two_step_procedure();
            break;

        case COMBINATORICAL_APPROACH:
            compute_combinatorical_approach();
            break;

        default:
            return;
    }
}


template <class T>
void PivotCalibration<T>::compute_two_step_procedure()
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

    // estimate sphere parameters

    FitSphere<T> fitSphere(locations);

    fitSphere.compute();

    Vector3T centerPoint = fitSphere.getCenterPoint().toArray();

    // compute the difference vector, transform it into the sensor system and average it

    translation = Vector3T(0, 0, 0);

    const int n = rotations.size();

    for (int i = 0; i < n; ++i)
    {
        Vector3T differenceVectorGlobal = centerPoint - locations[i];

        // transform the vector into the local sensor coordinate system

        Vector3T differenceVectorLocal = rotations[i].inverse() * differenceVectorGlobal;

        // average the found translation vector (part 1)

        translation += differenceVectorLocal;
    }

    // average the found translation vector (part 2)

    translation = translation / n;
}


template <class T>
void PivotCalibration<T>::compute_combinatorical_approach()
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

    // Build up A and b.

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

    // Solve the system of equations.

    translation = (A.transpose() * A).inverse() * (A.transpose() * b);
}


/** \tparam T scalar type
  *
  * \return returns the sought translation vector (in local/tool coordinates)
  *
  * \see compute()
  */

template <class T>
const typename PivotCalibration<T>::Vector3T & PivotCalibration<T>::getTranslation() const
{
    return translation;
}


/** \tparam T scalar type
  *
  * \param [in] algorithm
  *
  * Sets the algorithm used to compute the sought tool tip location.
  *
  * \see Algorithm
  */

template <class T>
void PivotCalibration<T>::setAlgorithm(Algorithm algorithm)
{
    switch (algorithm)
    {
        case TWO_STEP_PROCEDURE:
        case COMBINATORICAL_APPROACH:
            this->algorithm = algorithm;
            return;

        default:
            ErrorObj error;
            error.setClassName("PivotCalibration<T>");
            error.setFunctionName("setAlgorithm()");
            error.setErrorMessage("Unknown pivot calibration algorithm.");
            error.setErrorCode(UNKNOWN_ALGORITHM);
            throw error;
    }
}


/** \tparam T scalar type
  *
  * \param [in] locations locations of the sensor systems
  *
  * Sets the sampled locations of the sensor system. The translation vectors
  * can either be plain 3D or (normalized) 4D homogeneous coordinates.
  *
  * \throw ErrorObj If the dimension of the translation vectors diverges from
  *                 three or four, an error object is thrown and its error code
  *                 is set to \c WRONG_VECTOR_SIZE.
  *
  * \see setRotations() and setTransformations()
  */

template <class T>
void PivotCalibration<T>::setLocations(const std::vector<Coordinate<T> > & locations)
{
    const int n = locations.size();

    this->locations.clear();

    for (int i = 0; i < n; ++i)
    {
        if (!(locations[i].size() == 3 || locations[i].size() == 4))
        {
            ErrorObj error;
            error.setClassName("PivotCalibration<T>");
            error.setFunctionName("setLocations(...)");
            error.setErrorMessage("One or more translation vectors are of wrong size.");
            error.setErrorCode(WRONG_VECTOR_SIZE);
            throw error;
        }

        this->locations.push_back(locations[i].toArray().head(3));
    }
}


/** \tparam T scalar type
  *
  * \param [in] locations locations of the sensor systems
  *
  * Sets the sampled locations of the sensor system.
  *
  * \see setRotations() and setTransformations()
  */

template <class T>
void PivotCalibration<T>::setLocations(const std::vector<Vector3T> & locations)
{
    this->locations = locations;
}


/** \tparam T scalar type
  *
  * \param [in] locations locations of the sensor systems
  *
  * Sets the sampled locations of the sensor system. The translation vectors
  * are assumed to be normalized 4D homogeneous coordinates.
  *
  * \see setRotations() and setTransformations()
  */

template <class T>
void PivotCalibration<T>::setLocations(const std::vector<Vector4T, Eigen::aligned_allocator<Vector4T> > & locations)
{
    const int n = locations.size();

    for (int i = 0; i < n; ++i)
    {
        this->locations.push_back(locations[i].head(3));
    }
}


/** \tparam T scalar type
  *
  * \param [in] matrices rotation matrices of the sensor systems
  *
  * Sets the sampled rotation matrices of the sensor system.
  *
  * \see setLocations() and setTransformations()
  */

template <class T>
void PivotCalibration<T>::setRotations(const std::vector<Matrix3T> & matrices)
{
    rotations = matrices;
}


/** \tparam T scalar type
  *
  * \param [in] transformations rotation and location of the sensor systems
  *
  * Sets the sampled transformations of the sensor system. The rotation as well
  * as the location of the sensor system are stored in a matrix of the form
  * \f[
  * \begin{pmatrix}
  *     R & t \\
  *     0 & 1
  * \end{pmatrix}
  * \f]
  * where \f$ R \f$ denotes the rotation of the system and \f$ t \f$ its
  * location.
  *
  * \see setRotations() and setLocations()
  */

template <class T>
void PivotCalibration<T>::setTransformations(const std::vector<Transform3D<T>, Eigen::aligned_allocator<Transform3D<T> > > & transformations)
{
    const int n = transformations.size();

    rotations.clear();
    locations.clear();

    for (int i = 0; i < n; ++i)
    {
        rotations.push_back(transformations[i].getTransformationMatrix().block(0, 0, 3, 3));
        locations.push_back(transformations[i].getTransformationMatrix().block(0, 3, 3, 1));
    }
}


} // namespace TRTK


#endif // PIVOTCALIBRATION_HPP_3123933941
