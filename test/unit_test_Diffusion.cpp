// Last changed on 2011-11-21


#include <cmath>
#include <vector>
#include <sstream>

#include <TRTK/Diffusion.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


double diffusivity(double)
{
    return 1.0;
}


void unit_test_Diffusion()
{
    using namespace TRTK;
    using namespace TRTK::Tools;
    using namespace TRTK::Diffusion;

    using std::vector;


    HEADING(Diffusion)


    SUBHEADING(firstDerivative())

        // Initialize Signal.

        vector<double> signal;

        for (unsigned i = 0; i < 10; ++i)
        {
            signal.push_back(0.5 * i);
        }

        // Compute Derivatives.

        vector<double> derivative1 = firstDerivative(signal, CONTINUED);
        vector<double> derivative2 = firstDerivative(signal, CIRCULAR);
        vector<double> derivative3 = firstDerivative(signal, INTERPOLATED);
        vector<double> derivative4 = firstDerivative(signal, REFLECTED);
        vector<double> derivative5 = firstDerivative(signal, ZERO);


        START_TEST
            assert(abs(derivative1[0] - 0.5) < 0.5);
            assert(abs(derivative1[derivative1.size() - 1] - 0.5) < 0.5);

            for (unsigned i = 1; i < derivative1.size() - 1; ++i)
            {
                assert(isEqual(derivative1[i], 0.5));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative2[0] - 0.5) < 3);
            assert(abs(derivative2[derivative2.size() - 1] - 0.5) < 3);

            for (unsigned i = 1; i < derivative2.size() - 1; ++i)
            {
                assert(isEqual(derivative2[i], 0.5));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative3[0] - 0.5) < 0.5);
            assert(abs(derivative3[derivative3.size() - 1] - 0.5) < 0.5);

            for (unsigned i = 1; i < derivative3.size() - 1; ++i)
            {
                assert(isEqual(derivative3[i], 0.5));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative4[0] - 0.5) < 0.5);
            assert(abs(derivative4[derivative4.size() - 1] - 0.5) < 0.5);

            for (unsigned i = 1; i < derivative4.size() - 1; ++i)
            {
                assert(isEqual(derivative4[i], 0.5));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative5[0] - 0.5) < 0.5);
            assert(abs(derivative5[derivative5.size() - 1] - 0.5) < 3);

            for (unsigned i = 1; i < derivative5.size() - 1; ++i)
            {
                assert(isEqual(derivative5[i], 0.5));
            }
        STOP_TEST

        // Clear used vectors

        signal.clear();
        derivative1.clear();
        derivative2.clear();
        derivative3.clear();
        derivative4.clear();
        derivative5.clear();

        // Initialize Signal.

        for (unsigned i = 0; i < 10; ++i)
        {
            signal.push_back(i * i + 3);
        }

         // Compute Derivatives.

        derivative1 = firstDerivative(signal, CONTINUED);
        derivative2 = firstDerivative(signal, CIRCULAR);
        derivative3 = firstDerivative(signal, INTERPOLATED);
        derivative4 = firstDerivative(signal, REFLECTED);
        derivative5 = firstDerivative(signal, ZERO);


        START_TEST
            assert(abs(derivative1[0] - 0.5) < 0.5);
            assert(abs(derivative1[derivative1.size() - 1] - 0.5) < 8.5);

            for (unsigned i = 1; i < derivative1.size() - 1; ++i)
            {
                assert(isEqual(derivative1[i], double(i * 2)));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative2[0] - 0.5) < 41);
            assert(abs(derivative2[derivative2.size() - 1] - 0.5) < 33);

            for (unsigned i = 1; i < derivative2.size() - 1; ++i)
            {
                assert(isEqual(derivative2[i], double(i * 2)));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative3[0] - 0.5) < 1);
            assert(abs(derivative3[derivative3.size() - 1] - 0.5) < 17);

            for (unsigned i = 1; i < derivative3.size() - 1; ++i)
            {
                assert(isEqual(derivative3[i], double(i * 2)));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative4[0] - 0.5) < 0.5);
            assert(abs(derivative4[derivative4.size() - 1] - 0.5) < 8.5);

            for (unsigned i = 1; i < derivative4.size() - 1; ++i)
            {
                assert(isEqual(derivative4[i], double(i * 2)));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative5[0] - 0.5) < 2);
            assert(abs(derivative5[derivative5.size() - 1] - 0.5) < 34.5);

            for (unsigned i = 1; i < derivative5.size() - 1; ++i)
            {
                assert(isEqual(derivative5[i], double(i * 2)));
            }
        STOP_TEST

        // Clear used vectors

        signal.clear();
        derivative1.clear();
        derivative2.clear();
        derivative3.clear();
        derivative4.clear();
        derivative5.clear();


    SUBHEADING(secondDerivative())

        // Initialize Signal.

        for (unsigned i = 0; i < 10; ++i)
        {
            signal.push_back(i * i + 3);
        }

         // Compute Derivatives.

        derivative1 = secondDerivative(signal, CONTINUED);
        derivative2 = secondDerivative(signal, CIRCULAR);
        derivative3 = secondDerivative(signal, INTERPOLATED);
        derivative4 = secondDerivative(signal, REFLECTED);
        derivative5 = secondDerivative(signal, ZERO);


        START_TEST
            assert(abs(derivative1[0] - 0.5) < 1);
            assert(abs(derivative1[derivative1.size() - 1] - 0.5) < 18);

            for (unsigned i = 1; i < derivative1.size() - 1; ++i)
            {
                assert(isEqual(derivative1[i], 2.0));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative2[0] - 0.5) < 82);
            assert(abs(derivative2[derivative2.size() - 1] - 0.5) < 99);

            for (unsigned i = 1; i < derivative2.size() - 1; ++i)
            {
                assert(isEqual(derivative2[i], 2.0));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative3[0] - 0.5) < 1);
            assert(abs(derivative3[derivative3.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative3.size() - 1; ++i)
            {
                assert(isEqual(derivative3[i], 2.0));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative4[0] - 0.5) < 1);
            assert(abs(derivative4[derivative4.size() - 1] - 0.5) < 18);

             for (unsigned i = 1; i < derivative4.size() - 1; ++i)
            {
                assert(isEqual(derivative4[i], 2.0));
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative5[0] - 0.5) < 3);
            assert(abs(derivative5[derivative5.size() - 1] - 0.5) < 102);

            for (unsigned i = 1; i < derivative5.size() - 1; ++i)
            {
                assert(isEqual(derivative5[i], 2.0));
            }
        STOP_TEST

        // Clear used vectors

        signal.clear();
        derivative1.clear();
        derivative2.clear();
        derivative3.clear();
        derivative4.clear();
        derivative5.clear();


    SUBHEADING(linearDiffusion())

        // Initialize Signal.

        for (unsigned i = 0; i < 12; ++i)
        {
            if(i<6)
            {
                signal.push_back(0.0);
            }
            else
            {
                signal.push_back(1.0);
            }
        }

        // Compute Derivatives.

        derivative1 = linearDiffusion(signal, 2.0, 0.25, CONTINUED);
        derivative2 = linearDiffusion(signal, 2.0, 0.25, CIRCULAR);
        derivative3 = linearDiffusion(signal, 2.0, 0.25, INTERPOLATED);
        derivative4 = linearDiffusion(signal, 2.0, 0.25, REFLECTED);
        derivative5 = linearDiffusion(signal, 2.0, 0.25, ZERO);


        START_TEST
            assert(abs(derivative1[0] - 0.5) < 1);
            assert(abs(derivative1[derivative1.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative1.size() - 1; ++i)
            {
                assert(abs(derivative1[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative2[0] - 0.5) < 1);
            assert(abs(derivative2[derivative2.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative2.size() - 1; ++i)
            {
                assert(abs(derivative2[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative3[0] - 0.5) < 1);
            assert(abs(derivative3[derivative3.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative3.size() - 1; ++i)
            {
                assert(abs(derivative3[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative4[0] - 0.5) < 1);
            assert(abs(derivative4[derivative4.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative4.size() - 2; ++i)
            {
                assert(abs(derivative4[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative5[0] - 0.5) < 1);
            assert(abs(derivative5[derivative5.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative5.size() - 2; ++i)
            {
                assert(abs(derivative5[0] - 0.5) < 1);
            }
        STOP_TEST

        // Clear used vectors

        signal.clear();
        derivative1.clear();
        derivative2.clear();
        derivative3.clear();
        derivative4.clear();
        derivative5.clear();

        // Initialize Signal.

        for (unsigned i = 0; i < 12; ++i)
        {
            if(i<6)
            {
                signal.push_back(0.0);
            }
            else
            {
                signal.push_back(1.0);
            }
        }

        // Compute Derivatives.

        derivative1 = linearDiffusion(signal, 1000.0, 0.25, CONTINUED);
        derivative5 = linearDiffusion(signal, 1000.0, 0.25, ZERO);


        START_TEST
            for (unsigned i = 0; i < derivative1.size() - 1; ++i)
            {
                 assert(isEqual(derivative1[i], 0.5));
            }
        STOP_TEST


        START_TEST
            for (unsigned i = 0; i < derivative5.size() - 1; ++i)
            {
                 assert(isEqual(floor(derivative5[i] * 10000) / 10000, 0.0));
            }
        STOP_TEST


        // Clear used vectors

        signal.clear();
        derivative1.clear();
        derivative5.clear();


    SUBHEADING(nonlinearIsotropicDiffusion())

        // Initialize Signal.

        for (unsigned i = 0; i < 12; ++i)
        {
            if(i<6)
            {
                signal.push_back(0.0);
            }
            else
            {
                signal.push_back(1.0);
            }
        }

        // Compute Derivatives.

        derivative1 = nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, CONTINUED);
        derivative2 = nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, CIRCULAR);
        derivative3 = nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, INTERPOLATED);
        derivative4 = nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, REFLECTED);
        derivative5 = nonlinearIsotropicDiffusion(signal, 2.0, 0.25, 0.0, ZERO);


        START_TEST
            assert(abs(derivative1[0] - 0.5) < 1);
            assert(abs(derivative1[derivative1.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative1.size() - 1; ++i)
            {
                assert(abs(derivative1[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative2[0] - 0.5) < 1);
            assert(abs(derivative2[derivative2.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative2.size() - 1; ++i)
            {
                assert(abs(derivative2[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative3[0] - 0.5) < 1);
            assert(abs(derivative3[derivative3.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative3.size() - 1; ++i)
            {
                assert(abs(derivative3[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative4[0] - 0.5) < 1);
            assert(abs(derivative4[derivative4.size() - 1] - 0.5) < 1);

             for (unsigned i = 1; i < derivative4.size() - 1; ++i)
            {
                assert(abs(derivative4[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            assert(abs(derivative5[0] - 0.5) < 1);
            assert(abs(derivative5[derivative5.size() - 1] - 0.5) < 1);

            for (unsigned i = 1; i < derivative5.size() - 1; ++i)
            {
                assert(abs(derivative5[0] - 0.5) < 1);
            }
        STOP_TEST


        // Clear used vectors

        signal.clear();
        derivative1.clear();
        derivative2.clear();
        derivative3.clear();
        derivative4.clear();
        derivative5.clear();


        // Initialize Signal.

        for (unsigned i = 0; i < 12; ++i)
        {
            if(i<6)
            {
                signal.push_back(0.0);
            }
            else
            {
                signal.push_back(1.0);
            }
        }

        // Compute Derivatives.

        derivative1 = nonlinearIsotropicDiffusion(signal, 1000.0, 0.25, 0.0, CONTINUED);
        derivative5 = nonlinearIsotropicDiffusion(signal, 1000.0, 0.25, 0.0, ZERO);


        START_TEST
            for (unsigned i = 0; i < derivative1.size() - 1; ++i)
            {
                 assert(isEqual(derivative1[i], 0.5));
            }
        STOP_TEST


        START_TEST
            for (unsigned i = 0; i < derivative5.size() - 1; ++i)
            {
                 assert(isEqual(floor(derivative5[i] * 10000) / 10000, 0.0));
            }
        STOP_TEST


        // Clear used vectors

        signal.clear();
        derivative1.clear();
        derivative5.clear();


    SUBHEADING(nonlinearIsotropicDiffusion())

        // Initialize Signal.

        for (unsigned i = 0; i < 12; ++i)
        {
            if(i<6)
            {
                signal.push_back(0.0);
            }
            else
            {
                signal.push_back(1.0);
            }
        }

        // Compute Derivatives.

        derivative1 = nonlinearIsotropicDiffusion(signal, &diffusivity, 2.0, 0.25, CONTINUED);
        derivative2 = nonlinearIsotropicDiffusion(signal, &diffusivity, 2.0, 0.25, CIRCULAR);
        derivative3 = nonlinearIsotropicDiffusion(signal, &diffusivity, 2.0, 0.25, INTERPOLATED);
        derivative4 = nonlinearIsotropicDiffusion(signal, &diffusivity, 2.0, 0.25, REFLECTED);
        derivative5 = nonlinearIsotropicDiffusion(signal, &diffusivity, 2.0, 0.25, ZERO);


        START_TEST
            for (unsigned i = 0; i < derivative1.size() - 1; ++i)
            {
                assert(abs(derivative1[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            for (unsigned i = 0; i < derivative2.size() - 1; ++i)
            {
                assert(abs(derivative2[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            for (unsigned i = 0; i < derivative3.size() - 1; ++i)
            {
                assert(abs(derivative3[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            for (unsigned i = 0; i < derivative4.size() - 1; ++i)
            {
                assert(abs(derivative4[0] - 0.5) < 1);
            }
        STOP_TEST


        START_TEST
            for (unsigned i = 0; i < derivative5.size() - 1; ++i)
            {
                assert(abs(derivative5[0] - 0.5) < 1);
            }
        STOP_TEST
}