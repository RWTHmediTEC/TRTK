// Last changed on 2011-05-27.


#include <TRTK/RegionGrowing3D.hpp>

#include "unit_test.hpp"


bool is3DPointInRegion(const TRTK::RegionGrowing3D<bool>::Region & region, const TRTK::RegionGrowing3D<bool>::Point & point)
{
    for (unsigned i = 0; i < region.size(); ++i)
    {
        if (region[i] == point) return true;
    }

    return false;
}


void unit_test_RegionGrowing3D()
{
    using namespace TRTK;

    const unsigned WIDTH  = 7;
    const unsigned HEIGHT = 5;
    const unsigned DEPTH  = 2;

    bool data[WIDTH * HEIGHT * DEPTH] = {
        0, 0, 0, 0, 0, 0, 1,
        0, 1, 1, 0, 0, 0, 0,
        0, 1, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 1, 1,

        0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 1, 1};

    RegionGrowing3D<bool>::label_type label_mask[WIDTH * HEIGHT * DEPTH] = {
        0, 0, 0, 0, 0, 0, 1,
        0, 2, 2, 0, 0, 0, 0,
        0, 2, 2, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 3, 3,
        0, 0, 0, 0, 0, 3, 3,

        0, 0, 0, 0, 0, 0, 0,
        0, 2, 2, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 3, 3,
        0, 0, 0, 0, 0, 3, 3};

    RegionGrowing3D<bool>::label_type label_mask2[WIDTH * HEIGHT * DEPTH] = {
        0, 0, 0, 0, 0, 0, 1,
        0, 2, 2, 0, 0, 0, 0,
        0, 2, 2, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 1, 1,

        0, 0, 0, 0, 0, 0, 0,
        0, 2, 2, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 1,
        0, 0, 0, 0, 0, 1, 1};


    HEADING(RegionGrowing3D<BinaryDataType>)


    SUBHEADING(Constructors)


        // regionGrowing3Da

        START_TEST
            RegionGrowing3D<bool> regionGrowing3Da;
            assert(regionGrowing3Da.getLabelMask() == NULL);
            assert(regionGrowing3Da.getRegions().size() == 0);
        STOP_TEST


        // regionGrowing3Db

        START_TEST
            RegionGrowing3D<bool> regionGrowing3Db(data, WIDTH, HEIGHT, DEPTH);
            assert(regionGrowing3Db.getLabelMask() != NULL);
            assert(regionGrowing3Db.getRegions().size() == 0);
        STOP_TEST


    SUBHEADING(setData())


        START_TEST
            regionGrowing3Da.setData(data, WIDTH, HEIGHT, DEPTH);
            assert(regionGrowing3Da.getLabelMask() != NULL);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            regionGrowing3Da.compute();
        STOP_TEST


        START_TEST
            regionGrowing3Db.compute();
        STOP_TEST


    SUBHEADING(getLabelMask())


        bool label_mask_is_correct = true;


        // regionGrowing3Da

        START_TEST
            const RegionGrowing3D<bool>::label_type * label_mask_a = regionGrowing3Da.getLabelMask();

            for (unsigned d = 0; d < DEPTH; ++d)
            {
                for (unsigned m = 0; m < HEIGHT; ++m)
                {
                    for (unsigned n = 0; n < WIDTH; ++n)
                    {
                        label_mask_is_correct &= (label_mask[d * WIDTH * HEIGHT + m * WIDTH + n] == label_mask_a[d * WIDTH * HEIGHT + m * WIDTH + n]);
                    }
                }
            }

            assert(label_mask_is_correct);
        STOP_TEST


        // regionGrowing3Db

        START_TEST
            const RegionGrowing3D<bool>::label_type * label_mask_b = regionGrowing3Db.getLabelMask();

            for (unsigned d = 0; d < DEPTH; ++d)
            {
                for (unsigned m = 0; m < HEIGHT; ++m)
                {
                    for (unsigned n = 0; n < WIDTH; ++n)
                    {
                        label_mask_is_correct &= (label_mask[d * WIDTH * HEIGHT + m * WIDTH + n] == label_mask_b[d * WIDTH * HEIGHT + m * WIDTH + n]);
                    }
                }
            }

            assert(label_mask_is_correct);
        STOP_TEST


        SUBHEADING(getRegions())


        // regionGrowing3Da

        START_TEST
            assert(regionGrowing3Da.getRegions().size() == 3);

            assert(regionGrowing3Da.getRegions()[0].size() == 1);
            assert(regionGrowing3Da.getRegions()[1].size() == 7);
            assert(regionGrowing3Da.getRegions()[2].size() == 8);
        STOP_TEST


        START_TEST
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[0], RegionGrowing3D<bool>::Point(6, 0, 0)));

            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[1], RegionGrowing3D<bool>::Point(1, 1, 0)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[1], RegionGrowing3D<bool>::Point(1, 1, 1)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[1], RegionGrowing3D<bool>::Point(1, 2, 0)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[1], RegionGrowing3D<bool>::Point(2, 1, 0)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[1], RegionGrowing3D<bool>::Point(2, 1, 1)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[1], RegionGrowing3D<bool>::Point(2, 2, 0)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[1], RegionGrowing3D<bool>::Point(2, 2, 1)));

            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[2], RegionGrowing3D<bool>::Point(5, 3, 0)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[2], RegionGrowing3D<bool>::Point(5, 3, 1)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[2], RegionGrowing3D<bool>::Point(5, 4, 0)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[2], RegionGrowing3D<bool>::Point(5, 4, 1)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[2], RegionGrowing3D<bool>::Point(6, 3, 0)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[2], RegionGrowing3D<bool>::Point(6, 3, 1)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[2], RegionGrowing3D<bool>::Point(6, 4, 0)));
            assert(is3DPointInRegion(regionGrowing3Da.getRegions()[2], RegionGrowing3D<bool>::Point(6, 4, 1)));
        STOP_TEST


        // regionGrowing3Db

        START_TEST
            assert(regionGrowing3Db.getRegions().size() == 3);

            assert(regionGrowing3Db.getRegions()[0].size() == 1);
            assert(regionGrowing3Db.getRegions()[1].size() == 7);
            assert(regionGrowing3Db.getRegions()[2].size() == 8);
        STOP_TEST


        START_TEST
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[0], RegionGrowing3D<bool>::Point(6, 0, 0)));

            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[1], RegionGrowing3D<bool>::Point(1, 1, 0)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[1], RegionGrowing3D<bool>::Point(1, 1, 1)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[1], RegionGrowing3D<bool>::Point(1, 2, 0)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[1], RegionGrowing3D<bool>::Point(2, 1, 0)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[1], RegionGrowing3D<bool>::Point(2, 1, 1)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[1], RegionGrowing3D<bool>::Point(2, 2, 0)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[1], RegionGrowing3D<bool>::Point(2, 2, 1)));

            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[2], RegionGrowing3D<bool>::Point(5, 3, 0)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[2], RegionGrowing3D<bool>::Point(5, 3, 1)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[2], RegionGrowing3D<bool>::Point(5, 4, 0)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[2], RegionGrowing3D<bool>::Point(5, 4, 1)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[2], RegionGrowing3D<bool>::Point(6, 3, 0)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[2], RegionGrowing3D<bool>::Point(6, 3, 1)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[2], RegionGrowing3D<bool>::Point(6, 4, 0)));
            assert(is3DPointInRegion(regionGrowing3Db.getRegions()[2], RegionGrowing3D<bool>::Point(6, 4, 1)));
        STOP_TEST


    SUBHEADING(setNeighborhoodSize())


        START_TEST
            regionGrowing3Da.setNeighborhoodSize(9);
            regionGrowing3Da.compute();

            const RegionGrowing3D<bool>::label_type * label_mask_c = regionGrowing3Da.getLabelMask();

            label_mask_is_correct = true;
            for (unsigned d = 0; d < DEPTH; ++d)
            {
                for (unsigned m = 0; m < HEIGHT; ++m)
                {
                    for (unsigned n = 0; n < WIDTH; ++n)
                    {
                        label_mask_is_correct &= (label_mask2[d * WIDTH * HEIGHT + m * WIDTH + n] == label_mask_c[d * WIDTH * HEIGHT + m * WIDTH + n]);
                    }
                }
            }
            assert(label_mask_is_correct);
            regionGrowing3Da.setNeighborhoodSize(2);
        STOP_TEST


    SUBHEADING(compute(seed_point))


        START_TEST
            TRTK::RegionGrowing3D<bool, unsigned int>::Point point(0, 0, 0);
            regionGrowing3Da.compute(point);
            assert(regionGrowing3Da.getRegions().size() == 0);
        STOP_TEST


        START_TEST
            point.x() = 6;
            point.y() = 0;
            point.z() = 0;
            regionGrowing3Da.compute(point);
            assert(regionGrowing3Da.getRegions().size() == 1);
        STOP_TEST


    SUBHEADING(compute(std::deque seed_points))


        START_TEST
            std::deque<TRTK::RegionGrowing3D<bool, unsigned int>::Point> points;
            point.x() = 0;
            point.y() = 0;
            point.z() = 0;
            points.push_back(point);
            point.x() = 1;
            point.y() = 1;
            point.z() = 0;
            points.push_back(point);
            point.x() = 1;
            point.y() = 1;
            point.z() = 1;
            points.push_back(point);
            regionGrowing3Da.compute(points);
            assert(regionGrowing3Da.getRegions().size() == 1);
        STOP_TEST


    SUBHEADING(compute(StrideX StrideY StrideZ))


        START_TEST
            regionGrowing3Da.compute<3, 3, 1>();
            assert(regionGrowing3Da.getRegions().size() == 2);
        STOP_TEST
}
