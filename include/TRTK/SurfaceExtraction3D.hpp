/*
    Segmentation of simply connected spaces in 3D data.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    Version 0.1.0 (2011-10-14)
*/

/** \file SurfaceExtraction3D.hpp
  * \brief This file contains the \ref TRTK::SurfaceExtraction3D "SurfaceExtraction3D" class.
  */


#ifndef SURFACE_EXTRACTION_3D_7431089643
#define SURFACE_EXTRACTION_3D_7431089643

#include <cstddef>

#include "Coordinate.hpp"


namespace TRTK
{


/** \tparam DataType        Data type of the input and output mask.
  *
  * \brief Surface extraction of simply connected spaces in 3D data.
  *
  * This class performs a simple surface extraction of voxel based volume data.
  * The output data is voxel based and has the same type.
  *
  * A point is considered to be a surface point, if there is a point in the
  * neighborhood which is not set (i.e., which is equal to zero) or carries
  * a different label. The neighborhood is a 6-cell octahedral neighborhood.
  *
  * Here is an example of how to use this class:
  *
  * \code
  * #include <iostream>
  *
  * #include "TRTK/SurfaceExtraction3D.hpp"
  *
  *
  * using namespace std;
  * using namespace TRTK;
  *
  *
  * const unsigned WIDTH  = 7;
  * const unsigned HEIGHT = 5;
  * const unsigned DEPTH  = 3;
  *
  * unsigned short data[WIDTH * HEIGHT * DEPTH] = {0, 0, 0, 0, 0, 0, 0,
  *                                                0, 1, 1, 1, 1, 2, 2,
  *                                                0, 1, 1, 1, 1, 2, 2,
  *                                                0, 1, 1, 1, 0, 2, 2,
  *                                                0, 0, 0, 0, 0, 2, 2,
  *
  *                                                0, 0, 0, 0, 0, 0, 0,
  *                                                0, 1, 1, 1, 1, 2, 2,
  *                                                0, 1, 1, 1, 1, 2, 2,
  *                                                0, 1, 1, 1, 0, 2, 2,
  *                                                0, 0, 0, 0, 0, 2, 2,
  *
  *                                                0, 0, 0, 0, 0, 0, 0,
  *                                                0, 1, 1, 1, 1, 2, 2,
  *                                                0, 1, 1, 1, 1, 2, 2,
  *                                                0, 1, 1, 1, 0, 2, 2,
  *                                                0, 0, 0, 0, 0, 2, 2};
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
  *     // Perform the surface extraction.
  *
  *     typedef SurfaceExtraction3D<unsigned short> SurfaceExtraction3D;
  *
  *     SurfaceExtraction3D surfaceExtraction3D(data, WIDTH, HEIGHT, DEPTH);
  *     surfaceExtraction3D.compute();
  *
  *     // Print the label mask.
  *
  *     cout << endl << "Surface voxels" << endl << endl;
  *
  *     const SurfaceExtraction3D::data_type * surfaceData = surfaceExtraction3D.getSurface();
  *
  *     for (unsigned d = 0; d < DEPTH; ++d)
  *     {
  *         for (unsigned m = 0; m < HEIGHT; ++m)
  *         {
  *             for (unsigned n = 0; n < WIDTH; ++n)
  *             {
  *                 cout << surfaceData[d * WIDTH * HEIGHT + m * WIDTH + n] << "  ";
  *             }
  *             cout << endl;
  *         }
  *         cout << endl << endl;
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
  * 0  0  0  0  0  0  0
  * 0  1  1  1  1  2  2
  * 0  1  1  1  1  2  2
  * 0  1  1  1  0  2  2
  * 0  0  0  0  0  2  2
  *
  *
  * 0  0  0  0  0  0  0
  * 0  1  1  1  1  2  2
  * 0  1  1  1  1  2  2
  * 0  1  1  1  0  2  2
  * 0  0  0  0  0  2  2
  *
  *
  * 0  0  0  0  0  0  0
  * 0  1  1  1  1  2  2
  * 0  1  1  1  1  2  2
  * 0  1  1  1  0  2  2
  * 0  0  0  0  0  2  2
  *
  *
  *
  * Surface voxels
  *
  * 0  0  0  0  0  0  0
  * 0  1  1  1  1  2  2
  * 0  1  1  1  1  2  2
  * 0  1  1  1  0  2  2
  * 0  0  0  0  0  2  2
  *
  *
  * 0  0  0  0  0  0  0
  * 0  1  1  1  1  2  2
  * 0  1  0  0  1  2  2
  * 0  1  1  1  0  2  2
  * 0  0  0  0  0  2  2
  *
  *
  * 0  0  0  0  0  0  0
  * 0  1  1  1  1  2  2
  * 0  1  1  1  1  2  2
  * 0  1  1  1  0  2  2
  * 0  0  0  0  0  2  2
  *
  * \endcode
  *
  * \see SurfaceExtraction2D
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2011-10-14
  */

template <class DataType>
class SurfaceExtraction3D
{
public:
    typedef DataType data_type;

    SurfaceExtraction3D();
    SurfaceExtraction3D(const DataType * data, const unsigned width, const unsigned height, const unsigned depth);
    virtual ~SurfaceExtraction3D();

    void compute();

    void setData(const DataType * data, const unsigned width, const unsigned height, const unsigned depth);
    const DataType * getSurface() const;

private:
    void initSurfaceData();
    unsigned int toIndex(const DataType x, const DataType y, const DataType z);

    unsigned height;
    unsigned width;
    unsigned depth;

    const DataType * data;
    DataType * surfaceData;
};


/** Constructs an instance of SurfaceExtraction3D. */

template <class DataType>
SurfaceExtraction3D<DataType>::SurfaceExtraction3D() :
    height(0),
    width(0),
    depth(0),
    data(NULL),
    surfaceData(NULL)
{
}


/** \param [in] data    3D data
  * \param [in] width   number of columns of \p data
  * \param [in] height  number of rows of \p data
  * \param [in] depth   number of planes of \p data
  *
  * Constructs an instance of SurfaceExtraction3D.
  */

template <class DataType>
SurfaceExtraction3D<DataType>::SurfaceExtraction3D(const DataType * data, const unsigned width, const unsigned height, const unsigned depth) :
    height(height),
    width(width),
    depth(depth),
    data(data)
{
    surfaceData = new DataType[width * height * depth];
    initSurfaceData();
}


/** Destructs the instance of SurfaceExtraction3D. */

template <class DataType>
SurfaceExtraction3D<DataType>::~SurfaceExtraction3D()
{
    if (surfaceData) delete[] surfaceData;
}


/** \brief Performs the surface extraction.
  *
  * All surfaces within the data set are extracted. Points of the same volume
  * resp. region must have the same value. The result is a volume mask containing
  * the surface points (having the same labels as their original regions).
  *
  * \see getSurface()
  */

template <class DataType>
void SurfaceExtraction3D<DataType>::compute()
{
    assert(width  > 0);
    assert(height > 0);
    assert(depth  > 0);

    // Do nothing in the case of a 0-dimensional data set.

    if (width * height * depth == 0) return;

    // Copy the boundary elements (sides of the data volume) since these are
    // surface elements by definition.

    // Note: toIndex(x, y, z) = z * width * height + y * width + x

    unsigned index = 0;

    // front and back

    for (unsigned x = 0; x < width; ++x)
    {
        for (unsigned y = 0; y < height; ++y)
        {
            index = toIndex(x, y, 0);
            surfaceData[index] = data[index];

            index = toIndex(x, y, depth - 1);
            surfaceData[index] = data[index];
        }
    }

    // top and bottom

    for (unsigned x = 0; x < width; ++x)
    {
        for (unsigned z = 0; z < depth; ++z)
        {
            index = toIndex(x, 0, z);
            surfaceData[index] = data[index];

            index = toIndex(x, height - 1, z);
            surfaceData[index] = data[index];
        }
    }

    // left and right side

    for (unsigned y = 0; y < height; ++y)
    {
        for (unsigned z = 0; z < depth; ++z)
        {
            index = toIndex(0, y, z);
            surfaceData[index] = data[index];

            index = toIndex(width - 1, y, z);
            surfaceData[index] = data[index];
        }
    }

    // Compute the surface elements.

    for (unsigned z = 1; z < depth - 1; ++z)
    {
        for (unsigned y = 1; y < height - 1; ++y)
        {
            for (unsigned x = 1; x < width - 1; ++x)
            {
                // If there is a voxel in the neighborhood which does not own
                // the same label (might be zero or another region label), then
                // the current voxel must be a surface voxel.

                const data_type label = data[toIndex(x, y, z)];

                if (!(data[toIndex(x - 1, y, z)] == label &&
                      data[toIndex(x + 1, y, z)] == label &&
                      data[toIndex(x, y - 1, z)] == label &&
                      data[toIndex(x, y + 1, z)] == label &&
                      data[toIndex(x, y, z - 1)] == label &&
                      data[toIndex(x, y, z + 1)] == label ))
                {
                    // surface voxel

                    surfaceData[toIndex(x, y, z)] = data[toIndex(x, y, z)];
                }
                else
                {
                    // voxel completely surrounded by other (identically labeled) voxels
                    // --> must be inner volume voxel

                    surfaceData[toIndex(x, y, z)] = 0;
                }
            }
        }
    }
}

/** \return Volume data where a voxel denotes a surface point. */

template <class DataType>
const DataType * SurfaceExtraction3D<DataType>::getSurface() const
{
    return surfaceData;
}


/** \brief Helper function.
  *
  * Intitializes the surface data with zero entries.
  */

template <class DataType>
void SurfaceExtraction3D<DataType>::initSurfaceData()
{
    for (unsigned i = 0; i < width * height * depth; ++i)
    {
        surfaceData[i] = DataType(0);
    }
}


/** \param [in] data    3D data
  * \param [in] width   number of columns of \p data
  * \param [in] height  number of rows of \p data
  * \param [in] depth   number of planes of \p data
  *
  * Sets the binary image as well as its width, height and depth.
  */

template <class DataType>
void SurfaceExtraction3D<DataType>::setData(const DataType * data, const unsigned width, const unsigned height, const unsigned depth)
{
    this->data = data;
    this->width = width;
    this->height = height;
    this->depth = depth;

    if (surfaceData) delete[] surfaceData;

    surfaceData = new DataType[width * height * depth];
    initSurfaceData();
}


/** \brief Helper function
  *
  * Converts the voxel coordinate (x, y, z) to an index showing to the same
  * position within the input/ouput data.
  */

template <class DataType>
inline unsigned int SurfaceExtraction3D<DataType>::toIndex(const DataType x, const DataType y, const DataType z)
{
    return z * width * height + y * width + x;
}


} // namespace TRTK

#endif // SURFACE_EXTRACTION_3D_7431089643
