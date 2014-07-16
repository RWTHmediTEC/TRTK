// Last changed on 2012-08-31.


#include <TRTK/SurfaceExtraction3D.hpp>

#include "unit_test.hpp"


void unit_test_SurfaceExtraction3D()
{
    using namespace std;
    using namespace TRTK;

    const unsigned WIDTH  = 7;
    const unsigned HEIGHT = 5;
    const unsigned DEPTH  = 3;

    unsigned short data[WIDTH * HEIGHT * DEPTH] = {
        0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 1, 1, 2, 2,
        0, 1, 1, 1, 1, 2, 2,
        0, 1, 1, 1, 0, 2, 2,
        0, 0, 0, 0, 0, 2, 2,

        0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 1, 1, 2, 2,
        0, 1, 1, 1, 1, 2, 2,
        0, 1, 1, 1, 0, 2, 2,
        0, 0, 0, 0, 0, 2, 2,

        0, 0, 0, 0, 0, 0, 0,
        0, 1, 1, 1, 1, 2, 2,
        0, 1, 1, 1, 1, 2, 2,
        0, 1, 1, 1, 0, 2, 2,
        0, 0, 0, 0, 0, 2, 2};


    unsigned short surface_mask[WIDTH * HEIGHT * DEPTH] = {
         0, 0, 0, 0, 0, 0, 0,
         0, 1, 1, 1, 1, 2, 2,
         0, 1, 1, 1, 1, 2, 2,
         0, 1, 1, 1, 0, 2, 2,
         0, 0, 0, 0, 0, 2, 2,

         0, 0, 0, 0, 0, 0, 0,
         0, 1, 1, 1, 1, 2, 2,
         0, 1, 0, 0, 1, 2, 2,
         0, 1, 1, 1, 0, 2, 2,
         0, 0, 0, 0, 0, 2, 2,


         0, 0, 0, 0, 0, 0, 0,
         0, 1, 1, 1, 1, 2, 2,
         0, 1, 1, 1, 1, 2, 2,
         0, 1, 1, 1, 0, 2, 2,
         0, 0, 0, 0, 0, 2, 2};


    HEADING(SurfaceExtraction3D<DataType>)


    SUBHEADING(Constructors)


        // SurfaceExtraction3Da

        START_TEST
            SurfaceExtraction3D<unsigned short> SurfaceExtraction3Da;
            assert(SurfaceExtraction3Da.getSurface() == NULL);
        STOP_TEST


        // SurfaceExtraction3Db

        START_TEST
            SurfaceExtraction3D<unsigned short> SurfaceExtraction3Db(data, WIDTH, HEIGHT, DEPTH);
            assert(SurfaceExtraction3Db.getSurface() != NULL);
        STOP_TEST


    SUBHEADING(setData())


        START_TEST
            SurfaceExtraction3Da.setData(data, WIDTH, HEIGHT, DEPTH);
            assert(SurfaceExtraction3Da.getSurface() != NULL);
        STOP_TEST


    SUBHEADING(compute())


        START_TEST
            SurfaceExtraction3Da.compute();
        STOP_TEST


        START_TEST
            SurfaceExtraction3Db.compute();
        STOP_TEST


    SUBHEADING(getSurface())


        bool surface_is_correct = true;


        // SurfaceExtraction3Da

        START_TEST
            const SurfaceExtraction3D<unsigned short>::data_type * surfaceData_a = SurfaceExtraction3Da.getSurface();

            for (unsigned d = 0; d < DEPTH; ++d)
            {
                for (unsigned m = 0; m < HEIGHT; ++m)
                {
                    for (unsigned n = 0; n < WIDTH; ++n)
                    {
                        surface_is_correct &= (surface_mask[d * WIDTH * HEIGHT + m * WIDTH + n] == surfaceData_a[d * WIDTH * HEIGHT + m * WIDTH + n]);
                    }
                }
            }

            assert(surface_is_correct);
        STOP_TEST


        // SurfaceExtraction3Db

        START_TEST
           const SurfaceExtraction3D<unsigned short>::data_type * surfaceData_b = SurfaceExtraction3Db.getSurface();

            for (unsigned d = 0; d < DEPTH; ++d)
            {
                for (unsigned m = 0; m < HEIGHT; ++m)
                {
                    for (unsigned n = 0; n < WIDTH; ++n)
                    {
                        surface_is_correct &= (surface_mask[d * WIDTH * HEIGHT + m * WIDTH + n] == surfaceData_b[d * WIDTH * HEIGHT + m * WIDTH + n]);
                    }
                }
            }

            assert(surface_is_correct);
        STOP_TEST
}