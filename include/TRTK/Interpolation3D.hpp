/*
    Three-dimensional interpolation for irregularly-spaced data.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.2 (2013-08-20)
*/

/** \file Interpolation3D.hpp
  * \brief This file contains the \ref TRTK::Interpolation3D "Interpolation3D" class.
  */

#ifndef INTERPOLATION_3D_HPP_4326730820
#define INTERPOLATION_3D_HPP_4326730820


#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <flann/flann.hpp>

#include "Coordinate.hpp"
#include "Tools.hpp"

/*

All data items are stored in a kd-tree, internally. If an algorithm needs to
access them, the tree must be traversed. Since the tree behaves like normal
container, this can be easily done with iterators .

The algorithms are called via a function pointer which is set by the constructor
or a setter-method.

*/

namespace TRTK
{


/** \brief Three-dimensional interpolation for irregularly-spaced data.
  *
  * \tparam ValueType   Must be a floating point type.
  * \tparam PointType   Must provide \c operator[] for element access.
  * \tparam DataType    Type of the data associated with each point (e.g.
  *                     intensity, RGB-value, etc.). \p DataType must support
  *                     \p operator+= as well as the multiplication with a
  *                     scalar of type \p ValueType.
  *
  * This class provides a three-dimensional interpolation for irregularly-spaced
  * data. The data types can be arbitrary and must be specified as templates
  * parameters.
  *
  * The following \ref InterpolationMethod "interpolation methods" are available:
  *  - \ref INVERSE_DISTANCE_WEIGHTING "Inverse Distance Weighting":
  *    see \ref getDataInverseDistanceWeighting() for more details.
  *  - \ref INVERSE_DISTANCE_WEIGHTING_KNN "Inverse Distance Weighting KNN":
  *    see \ref getDataInverseDistanceWeightingKNN() for more details.
  *  - \ref NEAREST_NEIGHBOR "Nearest Neighbor":
  *    see \ref getDataNearestNeighbor() for more details.
  *
  * Here is an example of how to use the class:
  *
  * \code
  *
  * #include <iostream>
  * #include <TRTK/Interpolation3D.hpp>
  *
  * using namespace TRTK;
  *
  * int main()
  * {
  *     try
  *     {
  *         // Setup data.
  *
  *         typedef Interpolation3D<double, Coordinate<double>, double> Interpolation3D;
  *         typedef Interpolation3D::Point Point;
  *
  *         std::vector<Point> data_points;
  *         std::vector<Interpolation3D::data_type> data;
  *
  *         data_points.push_back(Point(1 ,1, 2));
  *         data.push_back(1);
  *
  *         data_points.push_back(Point(2, -1, 1));
  *         data.push_back(2);
  *
  *         data_points.push_back(Point(0, 0, 0));
  *         data.push_back(3);
  *
  *         data_points.push_back(Point(1, 0, -1));
  *         data.push_back(4);
  *
  *         data_points.push_back(Point(-1, 1, 3));
  *         data.push_back(5);
  *
  *         // Setup interpolation class.
  *
  *         Interpolation3D interpolation(data_points, data, 0);
  *
  *         interpolation.setInterpolationMethod(Interpolation3D::INVERSE_DISTANCE_WEIGHTING);
  *         interpolation.setNumberOfNearestNeighbors(10);
  *         interpolation.setRadius(10);
  *
  *         // Get interpolated values.
  *
  *         std::cout << interpolation(1.2, 1.0, 2.0) << std::endl;
  *     }
  *     catch (std::exception & error)
  *     {
  *         std::cout << error.what() << std::endl;
  *         return -1;
  *     }
  *
  *     return 0;
  * }
  *
  * \endcode
  *
  * Output:
  *
  * \code
  * 1.00032
  * \endcode
  *
  * To process higher-dimensional data, you might want to use the
  * \ref TRTK::Coordinate "Coordinate" class...:
  *
  * \code
  *
  * // Setup data.
  *
  * typedef Interpolation3D<double> Interpolation3D;
  * typedef Interpolation3D::Point Point;
  *
  * std::vector<Point> data_points;
  * std::vector<Interpolation3D::data_type> data;
  *
  * data_points.push_back(Point(0, 0, 0));
  * data.push_back(Point(100, 100, 0));
  *
  * data_points.push_back(Point(1 ,1, 1));
  * data.push_back(Point(255, 255, 255));
  *
  * data_points.push_back(Point(2, -1, 3));
  * data.push_back(Point(0, 0, 0));
  *
  * // Setup interpolation class.
  *
  * Interpolation3D interpolation(data_points, data, Point(0, 0, 0));
  *
  * // Get interpolated values.
  *
  * std::cout << interpolation(1.2, 0.1, 2) << std::endl;
  *
  * \endcode
  *
  * Output:
  *
  * \code
  * (173.451, 173.451, 165.952)
  * \endcode
  *
  * ... or a self-defined class as below:
  *
  * \code
  *
  * #include <iostream>
  *
  * #include <TRTK/Interpolation3D.hpp>
  *
  *
  * using namespace TRTK;
  *
  *
  * class RGB
  * {
  * public:
  *     RGB(unsigned short red = 0, unsigned short green = 0, unsigned short blue = 0)
  *     {
  *         data[0] = red;
  *         data[1] = green;
  *         data[2] = blue;
  *     }
  *
  *     RGB & operator+=(const RGB & other)
  *     {
  *         this->data[0] += other.data[0];
  *         this->data[1] += other.data[1];
  *         this->data[2] += other.data[2];
  *
  *         return *this;
  *     }
  *
  *     unsigned short operator[](size_t n) const
  *     {
  *         return data[n];
  *     };
  *
  * protected:
  *     unsigned short data[3];
  * };
  *
  *
  * RGB operator*(const RGB & rgb, float factor)
  * {
  *     return RGB(rgb[0] * factor, rgb[1] * factor, rgb[2] * factor);
  * }
  *
  *
  * int main()
  * {
  *     try
  *     {
  *         // Setup data.
  *
  *         typedef Interpolation3D<double, Coordinate<double>, RGB> Interpolation3D;
  *         typedef Interpolation3D::Point Point;
  *
  *         std::vector<Point> data_points;
  *         std::vector<Interpolation3D::data_type> data;
  *
  *         data_points.push_back(Point(0, 0, 0));
  *         data.push_back(RGB(100, 100, 0));
  *
  *         data_points.push_back(Point(1 ,1, 1));
  *         data.push_back(RGB(255, 255, 255));
  *
  *         data_points.push_back(Point(2, -1, 3));
  *         data.push_back(RGB(0, 0, 0));
  *
  *         // Setup interpolation class.
  *
  *         Interpolation3D interpolation(data_points, data);
  *
  *         interpolation.setInterpolationMethod(Interpolation3D::INVERSE_DISTANCE_WEIGHTING);
  *         interpolation.setRadius(10);
  *
  *         // Get interpolated values.
  *
  *         RGB value = interpolation(1.2, 0.1, 2);
  *
  *         std::cout << "("  << value[0]
  *                   << ", " << value[1]
  *                   << ", " << value[2]
  *                   << ")"  << std::endl;
  *     }
  *     catch (std::exception & error)
  *     {
  *         std::cout << error.what() << std::endl;
  *         return -1;
  *     }
  *
  *     return 0;
  * }
  *
  * \endcode
  *
  * Output:
  *
  * \code
  * (171, 171, 164)
  * \endcode
  *
  * \author Christoph Haenisch
  * \version 0.1.2
  * \date last changed on 2013-08-20
  */

template<class ValueType = double,
         class PointType = Coordinate<double>,
         class DataType  = Coordinate<double> >
class Interpolation3D
{
public:

    typedef ValueType value_type;
    typedef PointType point_type;
    typedef DataType  data_type;

    typedef PointType Point;            //!< Alias for point_type.

    enum InterpolationMethod
    {
        INVERSE_DISTANCE_WEIGHTING,     //!< Interpolation using Shepard's method. Only the k first nearest neighbors are taken into account. Searched is within a circle whose radius can be specified with \p setRadius.
        INVERSE_DISTANCE_WEIGHTING_KNN, //!< Interpolation using Shepard's method. Only the k first nearest neighbors are taken into account.
        NEAREST_NEIGHBOR                //!< Yields the datum of the nearest neighbor.
    };

    Interpolation3D(const DataType & default_value = DataType(),
                    InterpolationMethod interpolation_method = INVERSE_DISTANCE_WEIGHTING_KNN);

    Interpolation3D(const std::vector<Point> & data_points,
                    const std::vector<DataType> & data,
                    const DataType & default_value = DataType(),
                    InterpolationMethod interpolation_method = INVERSE_DISTANCE_WEIGHTING_KNN);

    ~Interpolation3D();

    DataType operator()(const Point & point) const;
    DataType operator()(ValueType x, ValueType y, ValueType z) const;

    DataType getData(const Point & point) const;
    DataType getData(ValueType x, ValueType y, ValueType z) const;

    void setData(const std::vector<Point> & data_points,
                 const std::vector<DataType> & data);

    void setExponent(ValueType value = 2.0);

    void setInterpolationMethod(InterpolationMethod interpolation_method);

    void setNumberOfNearestNeighbors(unsigned number = 10);

    void setRadius(ValueType radius);

protected:

    DataType (Interpolation3D::*interpolation_function)(ValueType, ValueType, ValueType) const;

    DataType getDataInverseDistanceWeighting(ValueType x, ValueType y, ValueType z) const;
    DataType getDataInverseDistanceWeightingKNN(ValueType x, ValueType y, ValueType z) const;
    DataType getDataNearestNeighbor(ValueType x, ValueType y, ValueType z) const;

    ValueType exponent;
    ValueType radius;

    unsigned max_number_of_nearest_neighbors;

    flann::Index<flann::L2<ValueType> > * tree;

    const DataType data_type_default_value;

    std::vector<DataType> data;

    flann::Matrix<ValueType> data_points;
    mutable flann::Matrix<ValueType> query_point;
    mutable flann::Matrix<ValueType> distances;
    mutable flann::Matrix<int> indices;
};


/** \brief Constructs an instance of Interpolation3D.
  *
  * \param [in] default_value               Default value for DataType (e.g. null vector).
  * \param [in] interpolation_method        Interpolation method to be used.
  */

template<class ValueType, class PointType, class DataType>
Interpolation3D<ValueType, PointType, DataType>::Interpolation3D(const DataType & default_value, InterpolationMethod interpolation_method) :
    exponent(2),
    radius(1),
    tree(NULL),
    data_type_default_value(default_value)
{
    query_point = flann::Matrix<ValueType>(new ValueType[3], 1, 3);

    setInterpolationMethod(interpolation_method);
    setNumberOfNearestNeighbors(10);
}


/** \brief Constructs an instance of Interpolation3D.
  *
  * \param [in] data_points                 Positions (in 3D) of the below data.
  * \param [in] data                        Data associated with the above positions (e.g. RGB-values).
  * \param [in] default_value               Default value for DataType (e.g. null vector).
  * \param [in] interpolation_method        Interpolation method to be used.
  */

template<class ValueType, class PointType, class DataType>
Interpolation3D<ValueType, PointType, DataType>::Interpolation3D(const std::vector<Point> & data_points,
                                                                 const std::vector<DataType> & data,
                                                                 const DataType & default_value,
                                                                 InterpolationMethod interpolation_method) :
    exponent(2),
    radius(1),
    tree(NULL),
    data_type_default_value(default_value)
{
    query_point = flann::Matrix<ValueType>(new ValueType[3], 1, 3);

    setData(data_points, data);
    setInterpolationMethod(interpolation_method);
    setNumberOfNearestNeighbors(10);
}


/** \brief Destructs the instance of Interpolation3D. */

template<class ValueType, class PointType, class DataType>
Interpolation3D<ValueType, PointType, DataType>::~Interpolation3D()
{
    if (tree != NULL) delete tree;

    data_points.free();
    query_point.free();
    distances.free();
    indices.free();
}


/** \brief Returns an interpolated value at the given position.
  *
  * \note Depending on the currently set interpolation method, an exception
  *       might be thrown.
  */

template<class ValueType, class PointType, class DataType>
inline DataType Interpolation3D<ValueType, PointType, DataType>::operator()(const Point & point) const
{
    return (this->*interpolation_function)(point.x(), point.y(), point.z());
}


/** \brief Returns an interpolated value at the given position.
  *
  * \note Depending on the currently set interpolation method, an exception
  *       might be thrown.
  */

template<class ValueType, class PointType, class DataType>
inline DataType Interpolation3D<ValueType, PointType, DataType>::operator()(ValueType x, ValueType y, ValueType z) const
{
    return (this->*interpolation_function)(x, y, z);
}


/** \brief Returns an interpolated value at the given position.
  *
  * \note Depending on the currently set interpolation method, an exception
  *       might be thrown.
  */

template<class ValueType, class PointType, class DataType>
inline DataType Interpolation3D<ValueType, PointType, DataType>::getData(const Point & point) const
{
    return (this->*interpolation_function)(point.x(), point.y(), point.z());
}


/** \brief Returns an interpolated value at the given position.
  *
  * \note Depending on the currently set interpolation method, an exception
  *       might be thrown.
  */

template<class ValueType, class PointType, class DataType>
inline DataType Interpolation3D<ValueType, PointType, DataType>::getData(ValueType x, ValueType y, ValueType z) const
{
    return (this->*interpolation_function)(x, y, z);
}


/** \brief Performs an Inverse Distance Weighting.
  *
  * \param [in] x   x position
  * \param [in] y   y position
  * \param [in] z   z position
  *
  * An interpolated value \f$ data' \f$ at postion \f$ (x, y, z) \f$ is computed
  * from its surrounding neighborhood \f$ \mathcal{N} \f$ by
  * \f[
  *     data' = \left( \sum_{i \in \mathcal{N}} {|d_i|}^{-p} \right)^{-1}
  *             \sum_{i \in \mathcal{N}} {|d_i|}^{-p} data_i
  * \f]
  * where \f$ d_i \f$ is the distance between the given point \f$ (x, y, z) \f$
  * and the location of the i-th data point. The Neighborhood is a circle whose
  * radius can be specified by \c setRadius(). At most \p max_number_of_nearest_neighbors
  * are used to compute the interpolant, which can be set by \p setNumberOfNearestNeighbors().
  * The exponent \f$ p \f$ can be set by \p setExponent().
  *
  * \exception std::range_error is thrown, if no data is given.
  * \exception std::runtime_error is thrown, if the search radius is too small and no nearest neighbor are found.
  *
  * \see InterpolationMethod
  */

template<class ValueType, class PointType, class DataType>
DataType Interpolation3D<ValueType, PointType, DataType>::getDataInverseDistanceWeighting(ValueType x, ValueType y, ValueType z) const
{
    if (this->tree->size() == 0)
    {
        throw std::range_error("Interpolation3D::getDataInverseDistanceWeighting(): Tree is empty.");
    }

    query_point[0][0] = x;
    query_point[0][1] = y;
    query_point[0][2] = z;

    // Find the first nearest neighbor.

    unsigned number_of_found_nearest_neighbors = tree->radiusSearch(query_point, indices, distances, radius, flann::SearchParams(128));

    if (number_of_found_nearest_neighbors == 0)
    {
        throw std::runtime_error("Interpolation3D::getDataInverseDistanceWeighting(): Radius too small. No nearest neighbor found.");
    }

    // If the found point is the query point itself, just return the associated data...

    if (Tools::isZero(distances[0][0]))
    {
        return data[indices[0][0]];
    }

    // ... otherwise return an interpolated datum from the first n nearest neighbors.

    unsigned number_of_neighbors = std::min(number_of_found_nearest_neighbors, max_number_of_nearest_neighbors);

    ValueType normalization_coefficient = 0;
    DataType data = data_type_default_value;

    for (unsigned i = 0; i < number_of_neighbors; ++i)
    {
        using std::pow;
        ValueType weight = pow(distances[0][i], -exponent);

        normalization_coefficient += weight;

        data += this->data[indices[0][i]] * weight;
    }

    return data * (1 / normalization_coefficient);
}


/** \brief Performs an Inverse Distance Weighting.
  *
  * \param [in] x   x position
  * \param [in] y   y position
  * \param [in] z   z position
  *
  * An interpolated value \f$ data' \f$ at postion \f$ (x, y, z) \f$ is computed
  * from its surrounding neighborhood \f$ \mathcal{N} \f$ by
  * \f[
  *     data' = \left( \sum_{i \in \mathcal{N}} {|d_i|}^{-p} \right)^{-1}
  *             \sum_{i \in \mathcal{N}} {|d_i|}^{-p} data_i
  * \f]
  * where \f$ d_i \f$ is the distance between the given point \f$ (x, y, z) \f$
  * and the location of the i-th data point. At most \p max_number_of_nearest_neighbors
  * are used to compute the interpolant, which can be set by \p setNumberOfNearestNeighbors().
  * The exponent \f$ p \f$ can be set by \p setExponent().
  *
  * \exception std::range_error is thrown, if no data is given.
  *
  * \see InterpolationMethod
  */

template<class ValueType, class PointType, class DataType>
DataType Interpolation3D<ValueType, PointType, DataType>::getDataInverseDistanceWeightingKNN(ValueType x, ValueType y, ValueType z) const
{
    if (this->tree->size() == 0)
    {
        throw std::range_error("Interpolation3D::getDataInverseDistanceWeightingKNN(): Tree is empty.");
    }

    query_point[0][0] = x;
    query_point[0][1] = y;
    query_point[0][2] = z;

    // Find the first nearest neighbor.

    tree->knnSearch(query_point, indices, distances, max_number_of_nearest_neighbors, flann::SearchParams(128));

    // If the found point is the query point itself, just return the associated data...

    if (Tools::isZero(distances[0][0]))
    {
        return data[indices[0][0]];
    }

    // ... otherwise return an interpolated datum from the first n nearest neighbors.

    const unsigned tree_size = tree->size();

    ValueType normalization_coefficient = 0;
    DataType data = data_type_default_value;

    for (unsigned i = 0; i < max_number_of_nearest_neighbors && i < tree_size; ++i)
    {
        using std::pow;
        ValueType weight = pow(distances[0][i], -exponent);

        normalization_coefficient += weight;

        data += this->data[indices[0][i]] * weight;
    }

    return data * (1 / normalization_coefficient);
}


/** \brief Performs a Nearest Neighbor Search.
  *
  * \param [in] x   x position
  * \param [in] y   y position
  * \param [in] z   z position
  *
  * \return The datum of the nearest neighbor is returned.
  *
  * \exception std::range_error is thrown, if no data is given.
  *
  * \see InterpolationMethod
  */

template<class ValueType, class PointType, class DataType>
DataType Interpolation3D<ValueType, PointType, DataType>::getDataNearestNeighbor(ValueType x, ValueType y, ValueType z) const
{
    if (this->tree->size() == 0)
    {
        throw std::range_error("Interpolation3D::getDataNearestNeighbor(): Tree is empty.");
    }

    query_point[0][0] = x;
    query_point[0][1] = y;
    query_point[0][2] = z;

    tree->knnSearch(query_point, indices, distances, 1, flann::SearchParams(128));

    return data[indices[0][0]];
}


/** \brief Sets the interpolation data.
  *
  * \param [in] data_points     Positions (in 3D) of the below data.
  * \param [in] data            Data associated with the above positions (e.g. RGB-values).
  *
  * The data may be irregularly spaced.
  *
  * \exception std::logic_error is thrown, if \p data_points and \p data do not have the same size.
  * \exception std::runtime_error is thrown, if the input data is emtpy.
  */

template<class ValueType, class PointType, class DataType>
void Interpolation3D<ValueType, PointType, DataType>::setData(const std::vector<Point> & data_points,
                                                              const std::vector<DataType> & data)
{
    if (data_points.size() != data.size())
    {
        throw std::logic_error("Interpolation3D::setData(): 'data_points' and 'data' must have the same size.");
    }

    if (data_points.size() == 0)
    {
        throw std::runtime_error("Interpolation3D::setData(): The input data may not be emtpy.");
    }

    // Copy the data points as well as the associated data.

    const unsigned number_points = data_points.size();

    this->data_points.free();
    this->data_points = flann::Matrix<ValueType>(new ValueType[3 * number_points], number_points, 3);;

    for (unsigned i = 0; i < number_points; ++i)
    {
        this->data_points[i][0] = data_points[i][0];
        this->data_points[i][1] = data_points[i][1];
        this->data_points[i][2] = data_points[i][2];
    }

    this->data = data;

    // Setup tree.

    if (tree != NULL) delete tree;

    tree = new flann::Index<flann::L2<ValueType> >(this->data_points, flann::KDTreeSingleIndexParams());
    tree->buildIndex();
}


/** \brief Sets the exponent for the inverse distance weighting.
  *
  * \param [in] value   Exponent.
  *
  * See \e Detailed \e Description for more details.
  */

template<class ValueType, class PointType, class DataType>
void Interpolation3D<ValueType, PointType, DataType>::setExponent(ValueType value)
{
    this->exponent = value;
}


/** \brief Sets the interpolation method.
  *
  * \param [in] interpolation_method    Interpolation method.
  *
  * \exception std::logic_error is thrown, if \p interpolation_method is unknown.
  *
  * \see InterpolationMethod
  */

template<class ValueType, class PointType, class DataType>
void Interpolation3D<ValueType, PointType, DataType>::setInterpolationMethod(InterpolationMethod interpolation_method)
{
    switch (interpolation_method)
    {
        case INVERSE_DISTANCE_WEIGHTING:
        {
            interpolation_function = &Interpolation3D::getDataInverseDistanceWeighting;
            break;
        }

        case INVERSE_DISTANCE_WEIGHTING_KNN:
        {
            interpolation_function = &Interpolation3D::getDataInverseDistanceWeightingKNN;
            break;
        }

        case NEAREST_NEIGHBOR:
        {
            interpolation_function = &Interpolation3D::getDataNearestNeighbor;
            break;
        }

        default:
        {
            throw std::logic_error("Interpolation3D::setInterpolationMethod(): Unknown interpolation method.");
        }
    }
}


/** \brief Sets the maximum number of nearest neighbors to take into account during computations.
  *
  * \param [in] number  Must be greater than zero.
  *
  * \exception std::logic_error is thrown, if \p number is equal to zero.
  *
  * \see getDataInverseDistanceWeighting(), getDataInverseDistanceWeightingKNN(),
  *      getDataNearestNeighbor()
  */

template<class ValueType, class PointType, class DataType>
void Interpolation3D<ValueType, PointType, DataType>::setNumberOfNearestNeighbors(unsigned number)
{
    if (number == 0)
    {
        throw std::logic_error("Interpolation3D::setNumberOfNearestNeighbors(): "
                               "Number of nearest neighbors must be greater than zero.");
    }

    this->max_number_of_nearest_neighbors = number;

    indices.free();
    distances.free();

    indices = flann::Matrix<int>(new int[number], 1, number);
    distances = flann::Matrix<ValueType>(new ValueType[number], 1, number);

}


/** \brief Sets the radius for the nearest neighbor search.
  *
  * \param [in] radius  Must be greater than zero.
  *
  * \exception std::logic_error is thrown, if \p radius is less than or equal to zero.
  *
  * \see getDataInverseDistanceWeighting(), getDataInverseDistanceWeightingKNN()
  */

template<class ValueType, class PointType, class DataType>
void Interpolation3D<ValueType, PointType, DataType>::setRadius(ValueType radius)
{
    if (radius <= 0)
    {
        throw std::logic_error("Interpolation3D::setRadius(): Radius must be greater than zero.");
    }

    this->radius = radius;
}


}


#endif // INTERPOLATION_3D_HPP_4326730820
