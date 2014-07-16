// Last changed on 2011-05-27.


#include <TRTK/RegionGrowing2D.hpp>
#include <TRTK/Coordinate.hpp>

#include "unit_test.hpp"



bool is2DPointInRegion(const TRTK::RegionGrowing2D<bool>::Region & region, const TRTK::RegionGrowing2D<bool>::Point & point)
{
    for (unsigned i = 0; i < region.size(); ++i)
    {
        if (region[i] == point) return true;
    }

    return false;
}


void unit_test_RegionGrowing2D()
{
    using namespace TRTK;

    const unsigned WIDTH = 11;
    const unsigned HEIGHT = 10;

    bool data[WIDTH * HEIGHT] = {
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
        0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0};

    RegionGrowing2D<bool>::label_type label_mask[WIDTH * HEIGHT] = {
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
        0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0, 3, 3, 3, 0, 0,
        0, 0, 0, 0, 0, 0, 3, 3, 3, 0, 0,
        0, 0, 0, 0, 0, 0, 3, 3, 3, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0,
        0, 0, 4, 4, 0, 0, 0, 0, 0, 0, 0};

    RegionGrowing2D<bool>::label_type label_mask2[WIDTH * HEIGHT] = {
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0,
        0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 2, 0, 0, 0, 2, 2, 2, 0, 0,
        0, 0, 0, 0, 0, 0, 2, 2, 2, 0, 0,
        0, 0, 0, 0, 0, 0, 2, 2, 2, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0,
        0, 0, 3, 3, 0, 0, 0, 0, 0, 0, 0};


    HEADING(RegionGrowing2D<BinaryDataType>)


    SUBHEADING(Constructors)


        // regionGrowing2Da

        START_TEST
            RegionGrowing2D<bool> regionGrowing2Da;
            assert(regionGrowing2Da.getLabelMask() == NULL);
            assert(regionGrowing2Da.getRegions().size() == 0);
        STOP_TEST


        // regionGrowing2Db

        START_TEST
            RegionGrowing2D<bool> regionGrowing2Db(data, WIDTH, HEIGHT);
            assert(regionGrowing2Db.getLabelMask() != NULL);
            assert(regionGrowing2Db.getRegions().size() == 0);
        STOP_TEST


    SUBHEADING(setData())


        START_TEST
            regionGrowing2Da.setData(data, WIDTH, HEIGHT);
            assert(regionGrowing2Da.getLabelMask() != NULL);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            regionGrowing2Da.compute();
        STOP_TEST


        START_TEST
            regionGrowing2Db.compute();
        STOP_TEST


    SUBHEADING(getLabelMask())


        bool label_mask_is_correct = true;


        // regionGrowing2Da

        START_TEST
            const RegionGrowing2D<bool>::label_type * label_mask_a = regionGrowing2Da.getLabelMask();

            for (unsigned m = 0; m < HEIGHT; ++m)
            {
                for (unsigned n = 0; n < WIDTH; ++n)
                {
                    label_mask_is_correct &= (label_mask[m * WIDTH + n] == label_mask_a[m * WIDTH + n]);
                }
            }

            assert(label_mask_is_correct);
        STOP_TEST


        // regionGrowing2Db

        START_TEST
            const RegionGrowing2D<bool>::label_type * label_mask_b = regionGrowing2Db.getLabelMask();

            for (unsigned m = 0; m < HEIGHT; ++m)
            {
                for (unsigned n = 0; n < WIDTH; ++n)
                {
                    label_mask_is_correct &= (label_mask[m * WIDTH + n] == label_mask_b[m * WIDTH + n]);
                }
            }

            assert(label_mask_is_correct);
        STOP_TEST


    SUBHEADING(getRegions())


        // regionGrowing2Da

        START_TEST
            assert(regionGrowing2Da.getRegions().size() == 4);

            assert(regionGrowing2Da.getRegions()[0].size() == 4);
            assert(regionGrowing2Da.getRegions()[1].size() == 9);
            assert(regionGrowing2Da.getRegions()[2].size() == 10);
            assert(regionGrowing2Da.getRegions()[3].size() == 2);
        STOP_TEST


        START_TEST
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[0], RegionGrowing2D<bool>::Point(8, 0)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[0], RegionGrowing2D<bool>::Point(8, 1)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[0], RegionGrowing2D<bool>::Point(9, 0)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[0], RegionGrowing2D<bool>::Point(9, 1)));

            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(1, 2)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(1, 3)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(2, 2)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(2, 3)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(2, 4)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(3, 2)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(3, 3)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(3, 4)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(2, 5)));

            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(6, 5)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(6, 6)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(7, 5)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(7, 6)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(6, 7)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(7, 7)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(8, 5)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(8, 6)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(8, 7)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[2], RegionGrowing2D<bool>::Point(8, 8)));

            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[3], RegionGrowing2D<bool>::Point(2, 9)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[3], RegionGrowing2D<bool>::Point(3, 9)));
        STOP_TEST


        // regionGrowing2Db

        START_TEST
            assert(regionGrowing2Db.getRegions().size() == 4);

            assert(regionGrowing2Db.getRegions()[0].size() == 4);
            assert(regionGrowing2Db.getRegions()[1].size() == 9);
            assert(regionGrowing2Db.getRegions()[2].size() == 10);
            assert(regionGrowing2Db.getRegions()[3].size() == 2);
        STOP_TEST


        START_TEST
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[0], RegionGrowing2D<bool>::Point(8, 0)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[0], RegionGrowing2D<bool>::Point(8, 1)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[0], RegionGrowing2D<bool>::Point(9, 0)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[0], RegionGrowing2D<bool>::Point(9, 1)));

            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[1], RegionGrowing2D<bool>::Point(1, 2)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[1], RegionGrowing2D<bool>::Point(1, 3)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[1], RegionGrowing2D<bool>::Point(2, 2)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[1], RegionGrowing2D<bool>::Point(2, 3)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[1], RegionGrowing2D<bool>::Point(2, 4)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[1], RegionGrowing2D<bool>::Point(3, 2)));
            assert(is2DPointInRegion(regionGrowing2Da.getRegions()[1], RegionGrowing2D<bool>::Point(3, 3)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[1], RegionGrowing2D<bool>::Point(3, 4)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[1], RegionGrowing2D<bool>::Point(2, 5)));

            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(6, 5)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(6, 6)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(7, 5)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(7, 6)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(6, 7)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(7, 7)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(8, 5)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(8, 6)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(8, 7)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[2], RegionGrowing2D<bool>::Point(8, 8)));

            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[3], RegionGrowing2D<bool>::Point(2, 9)));
            assert(is2DPointInRegion(regionGrowing2Db.getRegions()[3], RegionGrowing2D<bool>::Point(3, 9)));
        STOP_TEST


    SUBHEADING(setNeighborhoodSize())


        START_TEST
            regionGrowing2Da.setNeighborhoodSize(10);
            regionGrowing2Da.compute();

            const RegionGrowing2D<bool>::label_type * label_mask_c = regionGrowing2Da.getLabelMask();

            label_mask_is_correct = true;
            for (unsigned m = 0; m < HEIGHT; ++m)
            {
                for (unsigned n = 0; n < WIDTH; ++n)
                {
                    label_mask_is_correct &= (label_mask2[m * WIDTH + n] == label_mask_c[m * WIDTH + n]);
                }
            }
            assert(label_mask_is_correct);
            regionGrowing2Da.setNeighborhoodSize(2);
        STOP_TEST


    SUBHEADING(compute(seed_point))


        START_TEST
            TRTK::RegionGrowing2D<bool, unsigned int>::Point point(0,0);
            regionGrowing2Da.compute(point);
            assert(regionGrowing2Da.getRegions().size() == 0);
        STOP_TEST


        START_TEST
            point.x() = 8;
            point.y() = 0;
            regionGrowing2Da.compute(point);
            assert(regionGrowing2Da.getRegions().size() == 1);
        STOP_TEST


    SUBHEADING(compute(std::deque seed_points))


        START_TEST
            std::deque<TRTK::RegionGrowing2D<bool, unsigned int>::Point> points;
            point.x() = 0;
            point.y() = 0;
            points.push_back(point);
            point.x() = 8;
            point.y() = 0;
            points.push_back(point);
            point.x() = 8;
            point.y() = 1;
            points.push_back(point);
            regionGrowing2Da.compute(points);
            assert(regionGrowing2Da.getRegions().size() == 1);
        STOP_TEST


    SUBHEADING(compute<StrideX StrideY>())


        START_TEST
            regionGrowing2Da.compute<2, 2>();
            assert(regionGrowing2Da.getRegions().size() == 3);
        STOP_TEST
}
