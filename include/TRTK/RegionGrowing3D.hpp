/*
    Segmentation of simply connected spaces in 3D data.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.4.0 (2012-07-05)
*/

/** \file RegionGrowing3D.hpp
  * \brief This file contains the \ref TRTK::RegionGrowing3D "RegionGrowing3D" class.
  */


#ifndef REGION_GROWING_3D_1234689743
#define REGION_GROWING_3D_1234689743

#include <cmath>
#include <cstddef>
#include <deque>
#include <list>
#include <queue>

#include "Coordinate.hpp"


namespace TRTK
{


/** \tparam BinaryDataType   Data type of the binary mask.
  * \tparam LabelType        Data type of the label mask (should be able to
  *                          hold the number of segmented regions).
  * \tparam BinaryFieldType  Field type of the binary data. This allows
  *                          overloading \c operator[]().
  *
  * \brief Segmentation of simply connected spaces in 3D data.
  *
  * Segmentation of simply connected spaces in 3D data. The segmentation is
  * done fully automatically, i.e. no seed points need to be provided (though,
  * they can be provided). The result of the region growing is a label mask and
  * a list of regions where, again, each region consists of a list of points
  * forming the respective region.
  *
  * Points are considered to be connected if they are lying in the same
  * neighborhood, where a neighborhood \f$ \cal{N}\f$ is defined as
  * \f[
  *     \cal{N} \left( (x_0, y_0, z_0) \right)
  *         := \left\{ (x, y, z) \; | \; (x - x_0)^2 + (y - y_0)^2 + (z - z_0)^2 \le c \right\}
  * \f]
  * The parameter \f$ c \f$ can be set by \ref setNeighborhoodSize(); its
  * default value is 2.
  *
  * Here is an example of how to use this class:
  *
  * \code
  *
  * #include <iostream>
  *
  * #include "TRTK/RegionGrowing3D.hpp"
  *
  *
  * using namespace std;
  * using namespace TRTK;
  *
  *
  * const unsigned WIDTH  = 7;
  * const unsigned HEIGHT = 5;
  * const unsigned DEPTH  = 2;
  *
  * bool data[WIDTH * HEIGHT * DEPTH] = {0, 0, 0, 0, 0, 0, 1,
  *                                      0, 1, 1, 0, 0, 0, 0,
  *                                      0, 1, 1, 0, 0, 0, 0,
  *                                      0, 0, 0, 0, 0, 1, 1,
  *                                      0, 0, 0, 0, 0, 1, 1,
  *
  *                                      0, 0, 0, 0, 0, 0, 0,
  *                                      0, 1, 1, 0, 0, 0, 0,
  *                                      0, 0, 1, 0, 0, 0, 0,
  *                                      0, 0, 0, 0, 0, 1, 1,
  *                                      0, 0, 0, 0, 0, 1, 1};
  *
  * int main()
  * {
  *     // Print the data.
  *
  *     cout << "Data" << endl << endl;
  *
  *     for (unsigned d = 0; d < DEPTH; ++d)
  *     {
  *         for (unsigned m = 0; m < HEIGHT; ++m)
  *         {
  *             for (unsigned n = 0; n < WIDTH; ++n)
  *             {
  *                 cout << data[d * WIDTH * HEIGHT + m * WIDTH + n] << "  ";
  *             }
  *             cout << endl;
  *         }
  *         cout << endl << endl;
  *     }
  *
  *     // Perform the region growing.
  *
  *     RegionGrowing3D<bool> regionGrowing3D(data, WIDTH, HEIGHT, DEPTH);
  *     regionGrowing3D.setNeighborhoodSize(2);
  *     regionGrowing3D.compute();
  *
  *     // Print the label mask.
  *
  *     cout << endl << "Label mask" << endl << endl;
  *
  *     const RegionGrowing3D<bool>::label_type * label_mask = regionGrowing3D.getLabelMask();
  *
  *     for (unsigned d = 0; d < DEPTH; ++d)
  *     {
  *         for (unsigned m = 0; m < HEIGHT; ++m)
  *         {
  *             for (unsigned n = 0; n < WIDTH; ++n)
  *             {
  *                 cout << label_mask[d * WIDTH * HEIGHT + m * WIDTH + n] << "  ";
  *             }
  *             cout << endl;
  *         }
  *         cout << endl << endl;
  *     }
  *
  *     // Print the regions.
  *
  *     cout << endl << "Regions" << endl << endl;
  *
  *     for (unsigned label = 0; label < regionGrowing3D.getRegions().size(); ++label)
  *     {
  *         for (unsigned i = 0; i < regionGrowing3D.getRegions()[label].size(); ++i)
  *         {
  *             cout << regionGrowing3D.getRegions()[label][i] << "  ";
  *         }
  *         cout << endl;
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
  *
  * Data
  *
  * 0  0  0  0  0  0  1
  * 0  1  1  0  0  0  0
  * 0  1  1  0  0  0  0
  * 0  0  0  0  0  1  1
  * 0  0  0  0  0  1  1
  *
  *
  * 0  0  0  0  0  0  0
  * 0  1  1  0  0  0  0
  * 0  0  1  0  0  0  0
  * 0  0  0  0  0  1  1
  * 0  0  0  0  0  1  1
  *
  *
  *
  * Label mask
  *
  * 0  0  0  0  0  0  1
  * 0  2  2  0  0  0  0
  * 0  2  2  0  0  0  0
  * 0  0  0  0  0  3  3
  * 0  0  0  0  0  3  3
  *
  *
  * 0  0  0  0  0  0  0
  * 0  2  2  0  0  0  0
  * 0  0  2  0  0  0  0
  * 0  0  0  0  0  3  3
  * 0  0  0  0  0  3  3
  *
  *
  *
  * Regions
  *
  * (6, 0, 0)
  * (1, 1, 0)  (1, 1, 1)  (1, 2, 0)  (2, 1, 0)  (2, 1, 1)  (2, 2, 0)  (2, 2, 1)
  * (5, 3, 0)  (5, 3, 1)  (5, 4, 0)  (5, 4, 1)  (6, 3, 0)  (6, 3, 1)  (6, 4, 0)  (6, 4, 1)
  *
  * \endcode
  *
  * \see RegionGrowing2D
  *
  * \author Christoph Haenisch
  * \version 0.4.0
  * \date last changed on 2012-07-05
  */

template <class BinaryDataType, class LabelType = unsigned short, class BinaryFieldType = const BinaryDataType *>
class RegionGrowing3D
{
public:
    typedef BinaryDataType binary_value_type;
    typedef LabelType label_type;

    typedef Coordinate<unsigned> Point;
    typedef std::deque<Point> Region;
    typedef std::deque<Region> Regions;

    RegionGrowing3D();
    RegionGrowing3D(BinaryFieldType data, const unsigned width, const unsigned height, const unsigned depth);
    virtual ~RegionGrowing3D();

    void setData(BinaryFieldType data, const unsigned width, const unsigned height, const unsigned depth);
    void setNeighborhoodSize(const unsigned size);

    template <int StrideX, int StrideY, int StrideZ> void compute();
    void compute();
    void compute(const Point & seed_point);
    void compute(const std::deque<Point> & seed_points);

    const Regions & getRegions() const;
    const LabelType * getLabelMask() const;

private:

    unsigned getIndexFromPoint(const Point & point) const;
    std::list<Point> getNeighbors(const Point & point, const unsigned size = 2) const;
    Point getPointFromIndex(const unsigned index) const;
    void initLabelMask();

    unsigned height;
    unsigned width;
    unsigned depth;
    unsigned neighborhood_size;

    bool data_available;
    BinaryFieldType data;
    LabelType * label_mask;
    Regions regions;
};


/** Constructs an instance of RegionGrowing3D. */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::RegionGrowing3D() :
    height(0),
    width(0),
    depth(0),
    neighborhood_size(2),
    data_available(false),
    label_mask(NULL)
{
}


/** \param [in] data    3D data
  * \param [in] width   number of columns of \p data
  * \param [in] height  number of rows of \p data
  * \param [in] depth   number of planes of \p data
  *
  * Constructs an instance of RegionGrowing3D.
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::RegionGrowing3D(BinaryFieldType data, const unsigned width, const unsigned height, const unsigned depth) :
    height(height),
    width(width),
    depth(depth),
    neighborhood_size(2),
    data_available(true),
    data(data)
{
    label_mask = new LabelType[width * height * depth];
    initLabelMask();
}


/** Destructs the instance of RegionGrowing3D. */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::~RegionGrowing3D()
{
    if (label_mask) delete[] label_mask;
}


/** \brief Performs the region growing.
  *
  * All potential regions within the data set are extracted. The region growing
  * depends on the size of the neighborhood. The result is a label mask and a
  * list of regions which again are lists consisting of points of the respective
  * regions.
  *
  * \see setNeighborhoodSize(), getLabelMask(), getRegions()
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
void RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::compute()
{
    // Note: We use Point(x, y, z) and index = (z * width * height + y * width + x)
    //       interchangeably. Conversions are done with getPointFromIndex() and
    //       getIndexFromPoint().

    if (!data_available) return;

    regions.clear();
    initLabelMask();

    LabelType unique_label = 1;

    // For each point in the binary image, do...

    for (unsigned index = 0; index < width * height * depth; ++index)
    {
        // Skip the point if it is not set, i.e., if its value is zero.

        if (!data[index])
        {
            continue;
        }
        else
        {
            // If the point has a label it was already assigned to a region and
            // thus skip it. Otherwise, this point is part of a new region. Hence,
            // get a new unique region label and add the current point as well as
            // its neighbors to this new region. To be able to process all points
            // and to add new points at the same time, a queue is used (FIFO).

            if (label_mask[index])
            {
                continue;
            }

            LabelType label = unique_label++;
            regions.push_back(Region());

            std::queue<Point> queue;
            queue.push(getPointFromIndex(index));

            while (!queue.empty())
            {
                Point point = queue.front();
                queue.pop();

                // Assign the label of the current region to the current point.

                label_mask[getIndexFromPoint(point)] = label;

                // Save the point in a global list which only contains points of a certain region.

                regions[label - 1].push_back(point);

                // Now, add those points of the neighborhood to the queue which are
                // set within the binary mask but still don't carry any label. Then
                // set the label of these points to the current region. As a result,
                // these points won't be selected a second time.

                std::list<Point> neighbors = getNeighbors(point, neighborhood_size);

                for(std::list<Point>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
                {
                    const unsigned index = getIndexFromPoint(*neighbor);
                    if (data[index] && !label_mask[index])
                    {
                        label_mask[index] = label;
                        queue.push(*neighbor);
                    }
                }
            }
        }
    }
}


/** \brief Performs the region growing.
  *
  * \tparam StrideX          Step size in x while traversing the data.
  * \tparam StrideY          Step size in y while traversing the data.
  * \tparam StrideZ          Step size in z while traversing the data.
  *
  * All potential regions within the data set are extracted. The region growing
  * depends on the size of the neighborhood. The result is a label mask and a
  * list of regions which again are lists consisting of points of the respective
  * regions.
  *
  * The template parameters \p StrideX, \p StrideY and \p StrideZ specify the
  * step size with which the binary mask is traversed while searching for
  * potential seed points. This may lead to a significant speed-up. However,
  * structures may be missed if their dimensions are smaller than these sizes.
  *
  * \see setNeighborhoodSize(), getLabelMask(), getRegions()
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
template <int StrideX, int StrideY, int StrideZ>
void RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::compute()
{
    // Note: We use Point(x, y, z) and index = (z * width * height + y * width + x)
    //       interchangeably. Conversions are done with getPointFromIndex() and
    //       getIndexFromPoint().

    if (!data_available) return;

    regions.clear();
    initLabelMask();

    LabelType unique_label = 1;

    // For each point in the binary image, do...

    for (unsigned z = 0; z < depth; z += StrideZ)
    {
        for (unsigned y = 0; y < height; y += StrideY)
        {
            for (unsigned x = 0; x < width; x += StrideX)
            {
                unsigned index = z * width * height + y * width + x;

                // Skip the point if it is not set, i.e., if its value is zero.

                if (!data[index])
                {
                    continue;
                }
                else
                {
                    // If the point has a label it was already assigned to a region and
                    // thus skip it. Otherwise, this point is part of a new region. Hence,
                    // get a new unique region label and add the current point as well as
                    // its neighbors to this new region. To be able to process all points
                    // and to add new points at the same time, a queue is used (FIFO).

                    if (label_mask[index])
                    {
                        continue;
                    }

                    LabelType label = unique_label++;
                    regions.push_back(Region());

                    std::queue<Point> queue;
                    queue.push(getPointFromIndex(index));

                    while (!queue.empty())
                    {
                        Point point = queue.front();
                        queue.pop();

                        // Assign the label of the current region to the current point.

                        label_mask[getIndexFromPoint(point)] = label;

                        // Save the point in a global list which only contains points of a certain region.

                        regions[label - 1].push_back(point);

                        // Now, add those points of the neighborhood to the queue which are
                        // set within the binary mask but still don't carry any label. Then
                        // set the label of these points to the current region. As a result,
                        // these points won't be selected a second time.

                        std::list<Point> neighbors = getNeighbors(point, neighborhood_size);

                        for(std::list<Point>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
                        {
                            const unsigned index = getIndexFromPoint(*neighbor);
                            if (data[index] && !label_mask[index])
                            {
                                label_mask[index] = label;
                                queue.push(*neighbor);
                            }
                        }
                    }
                }
            }
        }
    }
}


/** \brief Performs the region growing.
  *
  * \param [in] seed_point     Starting point for region growing.
  *
  * Only a single region is extracted. The region growing depends on the size
  * of the neighborhood. The result is a label mask and a list of regions which
  * again are lists consisting of points of the respective regions.
  *
  * \see setNeighborhoodSize(), getLabelMask(), getRegions()
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
void RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::compute(const Point & seed_point)
{
    std::deque<Point> seed_points;
    seed_points.push_back(seed_point);
    compute(seed_points);
}


/** \brief Performs the region growing.
  *
  * \param [in] seed_points     List of seed points.
  *
  * A seed point is a starting point from which a region is grown up. Only
  * those regions are extracted from which one or more points are in the list
  * of seed points. The region growing depends on the size of the neighborhood.
  * The result is a label mask and a list of regions which again are lists
  * consisting of points of the respective regions.
  *
  * \see setNeighborhoodSize(), getLabelMask(), getRegions()
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
void RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::compute(const std::deque<Point> & seed_points)
{
    // Note: We use Point(x, y, z) and index = (z * width * height + y * width + x)
    //       interchangeably. Conversions are done with getPointFromIndex() and
    //       getIndexFromPoint().

    if (!data_available) return;

    regions.clear();
    initLabelMask();

    LabelType unique_label = 1;

    // For each point in the binary image, do...

    for (unsigned i = 0; i < seed_points.size(); ++i)
    {
        unsigned index = getIndexFromPoint(seed_points[i]);

        // Skip the point if it is not set, i.e., if its value is zero.

        if (!data[index])
        {
            continue;
        }
        else
        {
            // If the point has a label it was already assigned to a region and
            // thus skip it. Otherwise, this point is part of a new region. Hence,
            // get a new unique region label and add the current point as well as
            // its neighbors to this new region. To be able to process all points
            // and to add new points at the same time, a queue is used (FIFO).

            if (label_mask[index])
            {
                continue;
            }

            LabelType label = unique_label++;
            regions.push_back(Region());

            std::queue<Point> queue;
            queue.push(getPointFromIndex(index));


            while (!queue.empty())
            {
                Point point = queue.front();
                queue.pop();

                // Assign the label of the current region to the current point.

                label_mask[getIndexFromPoint(point)] = label;

                // Save the point in a global list which only contains points of a certain region.

                regions[label - 1].push_back(point);

                // Now, add those points of the neighborhood to the queue which are
                // set within the binary mask but still don't carry any label. Then
                // set the label of these points to the current region. As a result,
                // these points won't be selected a second time.

                std::list<Point> neighbors = getNeighbors(point, neighborhood_size);

                for(std::list<Point>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
                {
                    const unsigned index = getIndexFromPoint(*neighbor);
                    if (data[index] && !label_mask[index])
                    {
                        label_mask[index] = label;
                        queue.push(*neighbor);
                    }
                }
            }
        }
    }
}


/** \param [in] point   A Cartesian coordinate (x, y, z).
  *
  * \brief Helper function.
  *
  * This function transforms \p point into an index such that
  * \c data[index] is equal to \c data(x, y, z).
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
inline unsigned RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::getIndexFromPoint(const Point & point) const
{
    return point.z() * width * height + point.y() * width + point.x();
}


/** A label mask is returned, whose entries denote, to which region a datum in
  * \c data belongs to.
  *
  * \return Pointer to the label mask or NULL if no region growing has been
  *         performed, yet.
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
const LabelType * RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::getLabelMask() const
{
    return label_mask;
}


/** \param [in] point   A point, whose neighborhood is to be determined.
  * \param [in] size    Size of the neighborhood.
  *
  * \brief Helper function.
  *
  * \returns A list with neighboring points is returned. Only points lying in
  *          data are returned.
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
std::list<typename RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::Point> RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::getNeighbors(const Point & point, const unsigned size) const
{
    // typical values for size are 0, 1, 2, 4, 8

    // Return a list with all adjacent neighbors. Also check,
    // whether a neighbor is still within the data.

    std::list<Point> neighbors;

    const int delta = size == 2 ? 1 : int(std::ceil(std::sqrt(float(size))));

    for (int delta_x = -delta; delta_x <= delta; ++delta_x)
    {
        for (int delta_y = -delta; delta_y <= delta; ++delta_y)
        {
            for (int delta_z = -delta; delta_z <= delta; ++delta_z)
            {
                if (unsigned(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) <= size)
                {
                    int x = point.x() + delta_x;
                    int y = point.y() + delta_y;
                    int z = point.z() + delta_z;

                    if (0 <= x && x < int(width) && 0 <= y && y < int(height) && 0 <= z && z < int(depth))
                    {
                        neighbors.push_back(Point(x, y, z));
                    }
                }
            }
        }
    }

    return neighbors;
}


/** \param [in] index A value between 0 and
  *             <tt>data.width * data.height * data.depth - 1<\tt>.
  *
  * \brief Helper function.
  *
  * \return A point (x, y, z) is returned, such that \c data[index] is equal to
  *         \c data(x, y, z).
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
inline typename RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::Point RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::getPointFromIndex(const unsigned index) const
{
    const unsigned remainder_xy = index % (width * height);
    const Point::value_type x = remainder_xy % width;
    const Point::value_type y = remainder_xy / width;
    const Point::value_type z = index / (width * height);

    return Point(x, y, z);
}


/** \return Returns a list with regions, where again each region consists of a
  *         list of points lying in this particular region.
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
inline const typename RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::Regions & RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::getRegions() const
{
    return regions;
}


/** \brief Helper function.
  *
  * Intitializes the label mask with zero entries.
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
void RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::initLabelMask()
{
    for (unsigned i = 0; i < width * height * depth; ++i)
    {
        label_mask[i] = LabelType(0);
    }
}


/** \param [in] data    3D data
  * \param [in] width   number of columns of \p data
  * \param [in] height  number of rows of \p data
  * \param [in] depth   number of planes of \p data
  *
  * Sets the binary image as well as its width, height and depth.
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
void RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::setData(BinaryFieldType data, const unsigned width, const unsigned height, const unsigned depth)
{
    this->data = data;
    this->data_available = true;

    this->width = width;
    this->height = height;
    this->depth = depth;

    if (label_mask) delete[] label_mask;

    label_mask = new LabelType[width * height * depth];
    initLabelMask();
}


/** \param [in] size    Size of the neighborhood.
  *
  * Sets the size \f$ c \f$ of the neighborhood \f$ \cal{N}\f$ of a point
  * \f$ (x_0, y_0, z_0) \f$, where the neighborhood is defined as
  * \f[
  *     \cal{N} \left( (x_0, y_0, z_0) \right)
  *         := \left\{ (x, y, z) \; | \; (x - x_0)^2 + (y - y_0)^2 + (z - z_0)^2 \le c \right\}
  * \f]
  */

template <class BinaryDataType, class LabelType, class BinaryFieldType>
void RegionGrowing3D<BinaryDataType, LabelType, BinaryFieldType>::setNeighborhoodSize(const unsigned size)
{
    neighborhood_size = size;
}


} // namespace TRTK

#endif // REGION_GROWING_3D_1234689743
