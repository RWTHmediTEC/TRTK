// Last changed on 2011-06-06.

#include <TRTK/Tools.hpp> // check for missing headers 

#include <fstream>
#include <list>
#include <sstream>
#include <vector>

#include <TRTK/Coordinate.hpp>
#include <TRTK/Transform3D.hpp>

#include "unit_test.hpp"


/*

The following functions will (intentionally) not be test:

- getCurrentDate()
- getCurrentTime()

*/

  
// #pragma GCC diagnostic push
// #pragma GCC diagnostic warning "-Wall"

class A
{
public:
    virtual ~A() {};
};


class B : public A
{
public:
    virtual ~B() {};
};


class C
{
public:
    virtual ~C() {};
};


void unit_test_Tools()
{
    HEADING(Tools)


    cout << endl << "Note: The following functions are not tested:"
         << endl << "      - Eigen3x3_to_QTransfom()"
         << endl << endl;


    using namespace TRTK;
    using namespace TRTK::Tools;


    SUBHEADING(fileExists())


        START_TEST
            assert(fileExists("./res/existing_file") == true);
        STOP_TEST


        START_TEST
            assert(fileExists("./res/nonexisting_file") == false);
        STOP_TEST


    SUBHEADING(fileLength())


        START_TEST
            assert(fileLength("./res/existing_file") == 68);
        STOP_TEST


        START_TEST
            assert(fileLength("./res/nonexisting_file") == 0);
        STOP_TEST


    SUBHEADING(fileLength())


        START_TEST
            std::ifstream file("./res/existing_file");

            if (file)
            {
                assert(fileLength(file) == 68);
            }
            else
            {
                bool FILE_NOT_FOUND = false;
                assert(FILE_NOT_FOUND);
            }
        STOP_TEST


    SUBHEADING(isClass())


        A a1;
        B b1;
        C c1;

        A * a2 = new A;
        A * a3 = new B;
        B * b2 = new B;
        C * c2 = new C;

        const C & c3 = c1;
        const C * c4 = c2;


        START_TEST
            assert(isClass<B>(a1) == false);
        STOP_TEST


        START_TEST
            assert(isClass<B>(a2) == false);
        STOP_TEST


        START_TEST
            assert(isClass<B>(a3) == true);
        STOP_TEST


        START_TEST
            assert(isClass<B>(b1) == true);
        STOP_TEST


        START_TEST
            assert(isClass<B>(b2) == true);
        STOP_TEST


        START_TEST
            assert(isClass<B>(c1) == false);
        STOP_TEST


        START_TEST
            assert(isClass<B>(c2) == false);
        STOP_TEST


        START_TEST
            assert(isClass<B>(c3) == false);
        STOP_TEST


        START_TEST
            assert(isClass<B>(c4) == false);
        STOP_TEST


    SUBHEADING(isDerivedFrom())


        A * a_ = new A;
        B * b_ = new B;
        C * c_ = new C;

        const B * b2_ = b_;
        const C * c2_ = c_;


        START_TEST
            assert(isDerivedFrom<A>(a_) == true);
        STOP_TEST


        START_TEST
            assert(isDerivedFrom<A>(b_) == true);
        STOP_TEST


        START_TEST
            assert(isDerivedFrom<A>(b2_) == true);
        STOP_TEST


        START_TEST
            assert(isDerivedFrom<A>(c_) == false);
        STOP_TEST


        START_TEST
            assert(isDerivedFrom<A>(c2_) == false);
        STOP_TEST


    SUBHEADING(isEqual())


        double a = 1.0;
        double b = 2.0;
        int c = 2;


        START_TEST
            // template argument is automatically deduced
            assert(isEqual(a, a) == true);
        STOP_TEST


        START_TEST
            // template argument is automatically deduced
            // --> here we use the specialized template for ints
            assert(isEqual(c, c) == true);
        STOP_TEST


        START_TEST
            // template argument is automatically deduced
            assert(isEqual(a, b) == false);
        STOP_TEST


        START_TEST
            // template argument is automatically deduced
            assert(isEqual(2 * a, b) == true);
        STOP_TEST


        START_TEST
            assert(isEqual<double>(2 * a, c) == true);
        STOP_TEST


    SUBHEADING(isZero())


        START_TEST
            // template argument (int) is automatically deduced
            assert(isZero(0) == true);
        STOP_TEST


        START_TEST
            // template argument (double) is automatically deduced
            assert(isZero(0.0) == true);
        STOP_TEST


        START_TEST
            // template argument (int) is automatically deduced
            assert(isZero(1) == false);
        STOP_TEST


        START_TEST
            // template argument is automatically deduced
            a = 1.0 / 3;
            assert(isZero(a - 1 / 3.0) == true);
        STOP_TEST


        START_TEST
            a = 0.42; // a is a double value --> might give a compiler warning
            assert(isZero<int>(a) == true);
        STOP_TEST

        
    SUBHEADING(listToVector())


        START_TEST
            std::list<int> lst;
            std::vector<int> vec;

            lst.push_back(1);
            lst.push_back(2);
            lst.push_back(3);

            vec = listToVector(lst);

            assert(vec.size() == 3);
            assert(vec[0] == 1);
            assert(vec[1] == 2);
            assert(vec[2] == 3);
        STOP_TEST


    SUBHEADING(mean())


        START_TEST
            std::list<Coordinate<double> > list2;
            list2.push_back(Coordinate<double>(1, 4));
            list2.push_back(Coordinate<double>(-1, 2));
            list2.push_back(Coordinate<double>(0, 3));
            Coordinate<double> result = mean(list2, Coordinate<double>(0, 0));
            assert(result == Coordinate<double>(0, 3)); 
        STOP_TEST


    SUBHEADING(orthogonalMatrixToQuaternion())
        

        START_TEST
            // Test for id = orthogonalMatrixToQuaternion o quaternionToOrthogonalMatrix.

            double error = 0;

            for (unsigned i = 0; i < 100; ++i)
            {
                Coordinate<double> quaternion(rand(-1.0, 1.0), rand(-1.0, 1.0), rand(-1.0, 1.0), rand(-1.0, 1.0));
                quaternion *= sign(quaternion.x()); // since q = -q
                Coordinate<double> quaternion2 = orthogonalMatrixToQuaternion(quaternionToOrthogonalMatrix(quaternion.normalize()));
                error += (quaternion - quaternion2).norm() / 100;
            }

            assert(error < 1e-10);
        STOP_TEST


    SUBHEADING(quaternionToOrthogonalMatrix())


        START_TEST
            Eigen::Matrix3d orthogonal_matrix = quaternionToOrthogonalMatrix(Coordinate<double>(2, -3, 5, 3).normalize());
            assert(abs(orthogonal_matrix.col(0).transpose() * orthogonal_matrix.col(0) - 1) < 10e-10);
            assert(abs(orthogonal_matrix.col(0).transpose() * orthogonal_matrix.col(1) - 0) < 10e-10);
            assert(abs(orthogonal_matrix.col(0).transpose() * orthogonal_matrix.col(2) - 0) < 10e-10);
            assert(abs(orthogonal_matrix.col(1).transpose() * orthogonal_matrix.col(1) - 1) < 10e-10);
            assert(abs(orthogonal_matrix.col(1).transpose() * orthogonal_matrix.col(2) - 0) < 10e-10);
            assert(abs(orthogonal_matrix.col(2).transpose() * orthogonal_matrix.col(2) - 1) < 10e-10);
            assert(orthogonal_matrix.determinant() > 0);
        STOP_TEST
    

    SUBHEADING(rand())


        START_TEST
            std::vector<double> vect;
            for (unsigned i = 0; i < 5000; ++i)
            {
                double d = rand<double>();
                vect.push_back(d);
            }

            double mean = 0;
            for (unsigned i = 0; i < vect.size(); ++i)
            {
                mean += vect[i];
            }
            mean /= vect.size();
            assert(abs(mean - 0.5) < 0.01);

            double variance = 0;
            for (unsigned i = 0; i < vect.size(); ++i)
            {
                variance += (vect[i] - mean) * (vect[i] - mean);
            }
            variance /= vect.size();
            assert(abs(variance - 0.0833) < 0.01);
        STOP_TEST


    SUBHEADING(rand(a, b))


        START_TEST
            a = 3.0;
            b = 20.0;
            vect.clear();
            for (unsigned i = 0; i < 5000; ++i)
            {
                double d = rand<double>(a, b);
                vect.push_back(d);
            }

            mean = 0;
            for (unsigned i = 0; i < vect.size(); ++i)
            {
                mean += vect[i];
            }
            mean /= vect.size();
            assert(abs(mean - 0.5*(a+b)) < 0.1);

            variance = 0;
            for (unsigned i = 0; i < vect.size(); ++i)
            {
                variance += (vect[i] - mean) * (vect[i] - mean);
            }
            variance /= vect.size();
            assert(abs(variance - 0.0833*(b-a)*(b-a)) < 0.3);
        STOP_TEST


    SUBHEADING(randn())

        START_TEST
            double mu = 0;
            double sigma = 1;
            vect.clear();
            for (unsigned i = 0; i < 5000; ++i)
            {
                double d = randn<double>();
                vect.push_back(d);
            }

            mean = 0;
            for (unsigned i = 0; i < vect.size(); ++i)
            {
                mean += vect[i];
            }
            mean /= vect.size();
            assert(abs(mean - mu) < 0.01);

            variance = 0;
            for (unsigned i = 0; i < vect.size(); ++i)
            {
                variance += (vect[i] - mean) * (vect[i] - mean);
            }
            variance /= vect.size();
            assert(abs(variance - sigma * sigma) < 0.05);
        STOP_TEST


    SUBHEADING(randn(mu, sigma))


        START_TEST
            mu = 13.0;
            sigma = 3.0;
            vect.clear();
            for (unsigned i = 0; i < 5000; ++i)
            {
                double d = randn<double>(mu, sigma);
                vect.push_back(d);
            }

            mean = 0;
            for (unsigned i = 0; i < vect.size(); ++i)
            {
                mean += vect[i];
            }
            mean /= vect.size();
            assert(abs(mean - mu) < 0.1);

            variance = 0;
            for (unsigned i = 0; i < vect.size(); ++i)
            {
                variance += (vect[i] - mean) * (vect[i] - mean);
            }
            variance /= vect.size();
            assert(abs(variance - sigma * sigma) < 0.3);
        STOP_TEST


    SUBHEADING(sign())


        START_TEST
            assert(sign(-134.2) == -1);
            assert(sign(0.0) == 0);
            assert(sign(134.2) == 1);
        STOP_TEST


    SUBHEADING(round())


        // Test doubles

        START_TEST
            a = -1.7;
            assert(isEqual(round(double(-1.7)), -2.0));
        STOP_TEST


        START_TEST
            a = -1.3;
            assert(isEqual(round(a), -1.0));
        STOP_TEST


        START_TEST
            a = 0.0;
            assert(isEqual(round(a), 0.0));
        STOP_TEST


        START_TEST
            a = 1.3;
            assert(isEqual(round(a), 1.0));
        STOP_TEST


        START_TEST
            a = 1.7;
            assert(isEqual(round(a), 2.0));
        STOP_TEST


        // Test for integer values

        START_TEST
            c = 1;
            assert(isEqual(round(c), 1));
        STOP_TEST


    SUBHEADING(standardDeviation())


        START_TEST
            std::list<Coordinate<double> > list3;
            list3.push_back(Coordinate<double>(2, 8));
            list3.push_back(Coordinate<double>(-2, 2));
            list3.push_back(Coordinate<double>(0, 5));
            result = standardDeviation(list3, Coordinate<double>(0, 0));
            assert((result - Coordinate<double>(2.255, 3.383)).norm() < 1e-2); 
        STOP_TEST


    SUBHEADING(toString())


        std::stringstream ss;


        START_TEST
            ss.str("");
            ss << toString(1);
            assert(ss.str() == "1");
        STOP_TEST


        START_TEST
            ss.str("");
            ss << toString(1.3);
            assert(ss.str() == "1.3");
        STOP_TEST


    SUBHEADING(variance())


        START_TEST
            std::list<Coordinate<double> > list4;
            list4.push_back(Coordinate<double>(1, 8));
            list4.push_back(Coordinate<double>(-1, 2));
            list4.push_back(Coordinate<double>(0, 5));
            result = Tools::variance(list4, Coordinate<double>(0, 0));
            cout << endl << result << endl;
            assert((result - Coordinate<double>(1, 9)).norm() < 1e-2); 
        STOP_TEST


    SUBHEADING(vectorToList())


        START_TEST
            lst.clear();
            vec.clear();

            vec.push_back(4);
            vec.push_back(5);
            vec.push_back(6);

            lst = vectorToList(vec);

            assert(lst.size() == 3);
            assert(lst.back() == 6); lst.pop_back();
            assert(lst.back() == 5); lst.pop_back();
            assert(lst.back() == 4);
        STOP_TEST
}

// #pragma GCC diagnostic pop
