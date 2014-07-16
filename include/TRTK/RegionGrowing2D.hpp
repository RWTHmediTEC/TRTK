/*
    Segmentation of simply connected spaces in 2D data.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.3.0 (2012-07-05)
*/

/** \file RegionGrowing2D.hpp
  * \brief This file contains the \ref TRTK::RegionGrowing2D "RegionGrowing2D" class.
  */


#ifndef REGION_GROWING_2D_1376134334
#define REGION_GROWING_2D_1376134334

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
  *
  * \brief Segmentation of simply connected spaces in 2D data.
  *
  * Segmentation of simply connected spaces in 2D data. The segmentation is
  * done fully automatically, i.e. no seed points need to be provided (though,
  * they can be provided). The result of the region growing is a label mask and
  * a list of regions where, again, each region consists of a list of points
  * forming the respective region.
  *
  * Points are considered to be connected if they are lying in the same
  * neighborhood, where a neighborhood \f$ \cal{N}\f$ is defined as
  * \f[
  *     \cal{N} \left( (x_0, y_0) \right)
  *         := \left\{ (x, y) \; | \; (x - x_0)^2 + (y - y_0)^2 \le c \right\}
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
  * #include "TRTK/RegionGrowing2D.hpp"
  *
  *
  * using namespace std;
  * using namespace TRTK;
  *
  *
  * const unsigned WIDTH = 11;
  * const unsigned HEIGHT = 10;
  *
  * bool data[WIDTH * HEIGHT] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
  *                              0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
  *                              0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
  *                              0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
  *                              0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
  *                              0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0,
  *                              0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0,
  *                              0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0,
  *                              0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
  *                              0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0};
  *
  * int main()
  * {
  *     // Print the data.
  *
  *     cout << "Data" << endl << endl;
  *
  *     for (unsigned m = 0; m < HEIGHT; ++m)
  *     {
  *         for (unsigned n = 0; n < WIDTH; ++n)
  *         {
  *             cout << data[m * WIDTH + n] << "  ";
  *         }
  *         cout << endl;
  *     }
  *
  *     // Perform the region growing.
  *
  *     RegionGrowing2D<bool> regionGrowing2D(data, WIDTH, HEIGHT);
  *     regionGrowing2D.setNeighborhoodSize(2);
  *     regionGrowing2D.compute();
  *
  *     // Print the label mask.
  *
  *     cout << endl << "Label mask" << endl << endl;
  *
  *     const RegionGrowing2D<bool>::label_type * label_mask = regionGrowing2D.getLabelMask();
  *
  *     for (unsigned m = 0; m < HEIGHT; ++m)
  *     {
  *         for (unsigned n = 0; n < WIDTH; ++n)
  *         {
  *             cout << label_mask[m * WIDTH + n] << "  ";
  *         }
  *         cout << std::endl;
  *     }
  *
  *     // Print the regions.
  *
  *     cout << endl << "Regions" << endl << endl;
  *
  *     for (unsigned label = 0; label < regionGrowing2D.getRegions().size(); ++label)
  *     {
  *         for (unsigned i = 0; i < regionGrowing2D.getRegions()[label].size(); ++i)
  *         {
  *             cout << regionGrowing2D.getRegions()[label][i] << "  ";
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
  * 0  0  0  0  0  0  0  0  1  1  0
  * 0  0  0  0  0  0  0  0  1  1  0
  * 0  1  1  1  0  0  0  0  0  0  0
  * 0  1  1  1  0  0  0  0  0  0  0
  * 0  0  1  1  0  0  0  0  0  0  0
  * 0  0  1  0  0  0  1  1  1  0  0
  * 0  0  0  0  0  0  1  1  1  0  0
  * 0  0  0  0  0  0  1  1  1  0  0
  * 0  0  0  0  0  0  0  0  1  0  0
  * 0  0  1  1  0  0  0  0  0  0  0
  *
  * Label mask
  *
  * 0  0  0  0  0  0  0  0  1  1  0
  * 0  0  0  0  0  0  0  0  1  1  0
  * 0  2  2  2  0  0  0  0  0  0  0
  * 0  2  2  2  0  0  0  0  0  0  0
  * 0  0  2  2  0  0  0  0  0  0  0
  * 0  0  2  0  0  0  3  3  3  0  0
  * 0  0  0  0  0  0  3  3  3  0  0
  * 0  0  0  0  0  0  3  3  3  0  0
  * 0  0  0  0  0  0  0  0  3  0  0
  * 0  0  4  4  0  0  0  0  0  0  0
  *
  * Regions
  *
  * (8, 0)  (8, 1)  (9, 0)  (9, 1)
  * (1, 2)  (1, 3)  (2, 2)  (2, 3)  (2, 4)  (3, 2)  (3, 3)  (3, 4)  (2, 5)
  * (6, 5)  (6, 6)  (7, 5)  (7, 6)  (6, 7)  (7, 7)  (8, 5)  (8, 6)  (8, 7)  (8, 8)
  * (2, 9)  (3, 9)
  *
  * \endcode
  *
  * \see RegionGrowing3D
  *
  * \author Christoph Haenisch
  * \version 0.3.0
  * \date last changed on 2012-07-05
  */

template <class BinaryDataType, class LabelType = unsigned short>
class RegionGrowing2D
{
public:
    typedef BinaryDataType binary_value_type;
    typedef LabelType label_type;

    typedef Coordinate<unsigned> Point;
    typedef std::deque<Point> Region;
    typedef std::deque<Region> Regions;

    RegionGrowing2D();
    RegionGrowing2D(const BinaryDataType * data, const unsigned width, const unsigned height);
    virtual ~RegionGrowing2D();

    void setData(const BinaryDataType * data, const unsigned width, const unsigned height);
    void setNeighborhoodSize(const unsigned size);

    void compute();
    template <int StrideX, int StrideY> void compute();
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
    unsigned neighborhood_size;
    const BinaryDataType * data;
    LabelType * label_mask;
    Regions regions;
};


/** Constructs an instance of RegionGrowing2D. */

template <class BinaryDataType, class LabelType>
RegionGrowing2D<BinaryDataType, LabelType>::RegionGrowing2D() :
    height(0),
    width(0),
    neighborhood_size(2),
    data(NULL),
    label_mask(NULL)
{
}


/** \param [in] data    2D data
  * \param [in] width   number of columns of \p data
  * \param [in] height  number of rows of \p data
  *
  * Constructs an instance of RegionGrowing2D.
  */

template <class BinaryDataType, class LabelType>
RegionGrowing2D<BinaryDataType, LabelType>::RegionGrowing2D(const BinaryDataType * data, const unsigned width, const unsigned height) :
    height(height),
    width(width),
    neighborhood_size(2),
    data(data)
{
    label_mask = new LabelType[width * height];
    initLabelMask();
}


/** Destructs the instance of RegionGrowing2D. */

template <class BinaryDataType, class LabelType>
RegionGrowing2D<BinaryDataType, LabelType>::~RegionGrowing2D()
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

template <class BinaryDataType, class LabelType>
void RegionGrowing2D<BinaryDataType, LabelType>::compute()
{
    // Note: We use Point(x, y) and index = (y * width + x) interchangeably.
    //       Conversions are done with getPointFromIndex() and getIndexFromPoint().

    if (data == NULL) return;

    regions.clear();
    initLabelMask();

    LabelType unique_label = 1;

    // For each point in the binary image, do...

    for (unsigned index = 0; index < width * height; ++index)
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
  *
  * All potential regions within the data set are extracted. The region growing
  * depends on the size of the neighborhood. The result is a label mask and a
  * list of regions which again are lists consisting of points of the respective
  * regions.
  *
  * The template parameters \p StrideX and \p StrideY specify the step size
  * with which the binary mask is traversed while searching for potential seed
  * points. This may lead to a significant speed-up. However, structures may be
  * missed if their dimensions are smaller than these sizes.
  *
  * \see setNeighborhoodSize(), getLabelMask(), getRegions()
  */

template <class BinaryDataType, class LabelType>
template <int StrideX, int StrideY>
void RegionGrowing2D<BinaryDataType, LabelType>::compute()
{
    // Note: We use Point(x, y) and index = (y * width + x) interchangeably.
    //       Conversions are done with getPointFromIndex() and getIndexFromPoint().

    if (data == NULL) return;

    regions.clear();
    initLabelMask();

    LabelType unique_label = 1;

    // For each point in the binary image, do...

    for (unsigned y = 0; y < height; y += StrideY)
    {
        for (unsigned x = 0; x < width; x += StrideX)
        {
            unsigned index = y * width + x;

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

template <class BinaryDataType, class LabelType>
void RegionGrowing2D<BinaryDataType, LabelType>::compute(const Point & seed_point)
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

template <class BinaryDataType, class LabelType>
void RegionGrowing2D<BinaryDataType, LabelType>::compute(const std::deque<Point> & seed_points)
{
    // Note: We use Point(x, y) and index = (y * width + x) interchangeably.
    //       Conversions are done with getPointFromIndex() and getIndexFromPoint().

    if (data == NULL) return;

    regions.clear();
    initLabelMask();

    LabelType unique_label = 1;

    // For each seed point do...

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


/** \param [in] point   A Cartesian coordinate (x, y).
  *
  * \brief Helper function.
  *
  * This function transforms \p point into an index such that
  * \c data[index] is equal to \c data(x, y).
  */

template <class BinaryDataType, class LabelType>
inline unsigned RegionGrowing2D<BinaryDataType, LabelType>::getIndexFromPoint(const Point & point) const
{
    return point.y() * width + point.x();
}


/** \return A label mask is returned, whose entries denote, to which region
  * a datum in \c data belongs to.
  */

template <class BinaryDataType, class LabelType>
const LabelType * RegionGrowing2D<BinaryDataType, LabelType>::getLabelMask() const
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

template <class BinaryDataType, class LabelType>
std::list<typename RegionGrowing2D<BinaryDataType, LabelType>::Point> RegionGrowing2D<BinaryDataType, LabelType>::getNeighbors(const Point & point, const unsigned size) const
{
    // Typical values for size are 0, 1, 2, 4, 8.

    // Return a list with all adjacent neighbors. Also check,
    // whether a neighbor is still within the data.

    std::list<Point> neighbors;

	const int delta = size == 2 ? 1 : int(std::ceil(std::sqrt(float(size))));

    for (int delta_x = -delta; delta_x <= delta; ++delta_x)
    {
        for (int delta_y = -delta; delta_y <= delta; ++delta_y)
        {
            if (unsigned(delta_x * delta_x + delta_y * delta_y) <= size)
            {
                int x = point.x() + delta_x;
                int y = point.y() + delta_y;

                if (0 <= x && x < int(width) && 0 <= y && y < int(height))
                {
                    neighbors.push_back(Point(x, y));
                }
            }
        }
    }

    return neighbors;
}


/** \param [in] index A value between 0 and <tt>data.width * data.height - 1<\tt>.
  *
  * \brief Helper function.
  *
  * \return A point (x, y) is returned, such that \c data[index] is equal to
  *         \c data(x, y).
  */

template <class BinaryDataType, class LabelType>
inline typename RegionGrowing2D<BinaryDataType, LabelType>::Point RegionGrowing2D<BinaryDataType, LabelType>::getPointFromIndex(const unsigned index) const
{
    const Point::value_type x = index % width;
    const Point::value_type y = index / width;

    return Point(x, y);
}


/** \return Returns a list with regions, where again each region consists of a
  *         list of points lying in this particular region.
  */

template <class BinaryDataType, class LabelType>
inline const typename RegionGrowing2D<BinaryDataType, LabelType>::Regions & RegionGrowing2D<BinaryDataType, LabelType>::getRegions() const
{
    return regions;
}


/** \brief Helper function.
  *
  * Intitializes the label mask with zero entries.
  */

template <class BinaryDataType, class LabelType>
void RegionGrowing2D<BinaryDataType, LabelType>::initLabelMask()
{
    for (unsigned i = 0; i < width * height; ++i)
    {
        label_mask[i] = LabelType(0);
    }
}


/** \param [in] data    2D data
  * \param [in] width   number of columns of \p data
  * \param [in] height  number of rows of \p data
  *
  * Sets the binary image as well as its width and height.
  */

template <class BinaryDataType, class LabelType>
void RegionGrowing2D<BinaryDataType, LabelType>::setData(const BinaryDataType * data, const unsigned width, const unsigned height)
{
    this->data = data;
    this->width = width;
    this->height = height;

    if (label_mask) delete[] label_mask;

    label_mask = new LabelType[width * height];
    initLabelMask();
}


/** \param [in] size    Size of the neighborhood.
  *
  * Sets the size \f$ c \f$ of the neighborhood \f$ \cal{N}\f$ of a point
  * \f$ (x_0, y_0) \f$, where the neighborhood is defined as
  * \f[
  *     \cal{N} \left( (x_0, y_0) \right)
  *         := \left\{ (x, y) \; | \; (x - x_0)^2 + (y - y_0)^2 \le c \right\}
  * \f]
  */

template <class BinaryDataType, class LabelType>
void RegionGrowing2D<BinaryDataType, LabelType>::setNeighborhoodSize(const unsigned size)
{
    neighborhood_size = size;
}


} // namespace TRTK

#endif // REGION_GROWING_2D_1376134334
