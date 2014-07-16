// Last changed on 2014-07-05.


#include <TRTK/Coordinate.hpp>
#include <TRTK/Tools.hpp>

#include "unit_test.hpp"


void unit_test_Coordinate()
{
    using TRTK::Coordinate;
    using TRTK::Tools::isEqual;
    using TRTK::Tools::isZero;


    HEADING(Coordinate<T>)

    // We assume the constructors, the assignment operator as well as
    // operator==() to work properly.


    SUBHEADING(Constructors)

        // Check if there are any compiler errors...


        START_TEST
            Coordinate<double> a(1);
        STOP_TEST


        START_TEST
            Coordinate<double> b(1, 2);
        STOP_TEST


        START_TEST
            Coordinate<double> c(1, 2, 3);
        STOP_TEST


        START_TEST
            Coordinate<double> d(1, 2, 3, 4);
        STOP_TEST


        START_TEST
            Coordinate<int> i(a);
        STOP_TEST


        START_TEST
            a = Coordinate<double>(Eigen::Array3d(1, 2, 3));
            assert(a.toArray().isApprox(Eigen::Array3d(1, 2, 3)));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(Eigen::Vector3d(1, 2, 3));
            assert(a.toArray().matrix().isApprox(Eigen::Vector3d(1, 2, 3)));
        STOP_TEST


        START_TEST
            // conversion from row vector to column vector
            Eigen::RowVectorXd rowVector = Eigen::RowVector3d(1, 5, 3);
            a = Coordinate<double>(rowVector);
            assert(a.toArray().matrix().isApprox(Eigen::Vector3d(1, 5, 3)));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(Eigen::Vector3d());
        STOP_TEST


    SUBHEADING(Element Access)


        START_TEST
            i = Coordinate<int>(1, 2, 3, 4);
            assert(i.x() == 1);
            assert(i.y() == 2);
            assert(i.z() == 3);
            assert(i.w() == 4);
        STOP_TEST


        START_TEST
            i = Coordinate<int>(1, 2, 3, 4);
            assert(i[0] == 1);
            assert(i[1] == 2);
        STOP_TEST


    SUBHEADING(Operators)

        // operator+=(const T)
        // operator-=(const T)
        // operator*=(const T)
        // operator/=(const T)

        START_TEST
            a = Coordinate<double>(1, 2, 3, 4);
            a += 1;
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 3.0) &&
                   isEqual(a[2], 4.0) &&
                   isEqual(a[3], 5.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4);
            a -= 1;
            assert(isEqual(a[0], 0.0) &&
                   isEqual(a[1], 1.0) &&
                   isEqual(a[2], 2.0) &&
                   isEqual(a[3], 3.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4);
            a *= 2;
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 4.0) &&
                   isEqual(a[2], 6.0) &&
                   isEqual(a[3], 8.0));
        STOP_TEST


        START_TEST
            i = Coordinate<int>(1, 2, 3, 4);
            i /= 2;
            assert(isEqual(i[0], 0) &&
                   isEqual(i[1], 1) &&
                   isEqual(i[2], 1) &&
                   isEqual(i[3], 2));
        STOP_TEST


        cout << endl;


        // operator+=(const Coordinate &)
        // operator-=(const Coordinate &)
        // operator*=(const Coordinate &)
        // operator/=(const Coordinate &)

        START_TEST
            a = Coordinate<double>(1, 2, 3, 4);
            a += Coordinate<double>(1, 1, 1, 1);
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 3.0) &&
                   isEqual(a[2], 4.0) &&
                   isEqual(a[3], 5.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4);
            a -= Coordinate<double>(1, 1, 1, 1);
            assert(isEqual(a[0], 0.0) &&
                   isEqual(a[1], 1.0) &&
                   isEqual(a[2], 2.0) &&
                   isEqual(a[3], 3.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4);
            a *= Coordinate<double>(2, 2, 2, 2);
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 4.0) &&
                   isEqual(a[2], 6.0) &&
                   isEqual(a[3], 8.0));
        STOP_TEST


        START_TEST
            i = Coordinate<int> (1, 2, 3, 4);
            i /= Coordinate<int>(2, 2, 2, 2);
            assert(isEqual(i[0], 0) &&
                   isEqual(i[1], 1) &&
                   isEqual(i[2], 1) &&
                   isEqual(i[3], 2));
        STOP_TEST


        cout << endl;


        // operator+(const T)
        // operator-(const T)
        // operator*(const T)
        // operator/(const T)

        START_TEST
            a = Coordinate<double>(1, 2, 3, 4) + 1;
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 3.0) &&
                   isEqual(a[2], 4.0) &&
                   isEqual(a[3], 5.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4) - 1;
            assert(isEqual(a[0], 0.0) &&
                   isEqual(a[1], 1.0) &&
                   isEqual(a[2], 2.0) &&
                   isEqual(a[3], 3.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4) * 2;
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 4.0) &&
                   isEqual(a[2], 6.0) &&
                   isEqual(a[3], 8.0));
        STOP_TEST


        START_TEST
            i = Coordinate<int>(1, 2, 3, 4) / 2;
            assert(isEqual(i[0], 0) &&
                   isEqual(i[1], 1) &&
                   isEqual(i[2], 1) &&
                   isEqual(i[3], 2));
        STOP_TEST


        cout << endl;


        // operator+(const Coordinate &)
        // operator-(const Coordinate &)
        // operator*(const Coordinate &)
        // operator/(const Coordinate &)

        START_TEST
            a = Coordinate<double>(1, 2, 3, 4) + Coordinate<double>(1, 1, 1, 1);
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 3.0) &&
                   isEqual(a[2], 4.0) &&
                   isEqual(a[3], 5.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4) - Coordinate<double>(1, 1, 1, 1);
            assert(isEqual(a[0], 0.0) &&
                   isEqual(a[1], 1.0) &&
                   isEqual(a[2], 2.0) &&
                   isEqual(a[3], 3.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4) * Coordinate<double>(2, 2, 2, 2);
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 4.0) &&
                   isEqual(a[2], 6.0) &&
                   isEqual(a[3], 8.0));
        STOP_TEST


        START_TEST
            i = Coordinate<int>(1, 2, 3, 4) / Coordinate<int>(2, 2, 2, 2);
            assert(isEqual(i[0], 0) &&
                   isEqual(i[1], 1) &&
                   isEqual(i[2], 1) &&
                   isEqual(i[3], 2));
        STOP_TEST


        // operator data_type()

        START_TEST
            Eigen::Array<double, Eigen::Dynamic, 1> array1 = Coordinate<double>(1, 2, 3);
        STOP_TEST


        // operator const_data_type() const

        START_TEST
            const Coordinate<double> const_coordinate(1, 2, 3);
            const Eigen::Array<double, Eigen::Dynamic, 1> & array2 = const_coordinate;
        STOP_TEST


    SUBHEADING(Automatic Type Conversion)


        const Coordinate<double> c1(2.1, 3.2);
        const Coordinate<double> c2(4, 5);
        const Coordinate<int> c3(4, 5);

        Coordinate<double> result;


        // Coordinate<double> operator+(int, Coordinate<double> &)
        // Coordinate<double> operator-(int, Coordinate<double> &)
        // Coordinate<double> operator*(int, Coordinate<double> &)
        // Coordinate<double> operator/(int, Coordinate<double> &)

        START_TEST
            result = 2 + c1;
            assert(isEqual(result[0], 4.1) &&
                   isEqual(result[1], 5.2));
        STOP_TEST


        START_TEST
            result = 2 - c1;
            assert(isEqual(result[0], -0.1) &&
                   isEqual(result[1], -1.2));
        STOP_TEST


        START_TEST
            result = 2 * c1;
            assert(isEqual(result[0], 4.2) &&
                   isEqual(result[1], 6.4));
        STOP_TEST


        START_TEST
            result = 2 / c2;
            assert(isEqual(result[0], 0.5) &&
                   isEqual(result[1], 0.4));
        STOP_TEST


        cout << endl;


        // Coordinate<int> operator+(double, Coordinate<int> &)
        // Coordinate<int> operator-(double, Coordinate<int> &)
        // Coordinate<int> operator*(double, Coordinate<int> &)
        // Coordinate<int> operator/(double, Coordinate<int> &)

        START_TEST
            result = 2.1 + c3;
            assert(isEqual<double>(result[0], 6) &&
                   isEqual<double>(result[1], 7));
        STOP_TEST


        START_TEST
            result = 2.1 - c3;
            assert(isEqual<double>(result[0], -2) &&
                   isEqual<double>(result[1], -3));
        STOP_TEST


        START_TEST
            result = 2.1 * c3;
            assert(isEqual<double>(result[0], 8) &&
                   isEqual<double>(result[1], 10));
        STOP_TEST


        START_TEST
            result = 1.5 * Coordinate<int>(1000);
            assert(isEqual<double>(result[0], 1500));
        STOP_TEST


        START_TEST
            result = 2.0 / c3;
            assert(isEqual<double>(result[0], 0) &&
                   isEqual<double>(result[1], 0));
        STOP_TEST


        cout << endl;


        // Coordinate<T> operator+(T, const Coordinate<T> &)
        // Coordinate<T> operator+(T, const Coordinate<T> &)
        // Coordinate<T> operator+(T, const Coordinate<T> &)
        // Coordinate<T> operator+(T, const Coordinate<T> &)


        START_TEST
            result = 2.1 + c1;
            assert(isEqual(result[0], 4.2) &&
                   isEqual(result[1], 5.3));
        STOP_TEST



        START_TEST
            result = 2.1 - c1;
            assert(isEqual(result[0],  0.0) &&
                   isEqual(result[1], -1.1));
        STOP_TEST



        START_TEST
            result = 2.1 * c1;
            assert(isEqual(result[0], 4.41) &&
                   isEqual(result[1], 6.72));
        STOP_TEST



        START_TEST
            result = 2.5 / Coordinate<double>(2, 4);
            assert(isEqual(result[0], 1.250) &&
                   isEqual(result[1], 0.625));
        STOP_TEST


        cout << endl;


    SUBHEADING(dot())


        double residual = 0.0;


        START_TEST

            a = Coordinate<double>(1, 2, 3);
            b = Coordinate<double>(4, 5, 6);

            residual = a.dot(b) - 32.0;
            assert(isZero(residual));

            residual = a.dot(a) - a.norm() * a.norm();
            assert(isZero(residual));

            residual = b.dot(b) - b.norm() * b.norm();
            assert(isZero(residual));

        STOP_TEST


    SUBHEADING(cross())


        START_TEST

            a = Coordinate<double>(1, 0, 0);
            b = Coordinate<double>(0, 1, 0);
            c = Coordinate<double>(0, 0, 1);

            residual = (a.cross(b) - c).norm();  // A x B = C
            assert(isZero(residual));

            residual = (b.cross(c) - a).norm();  // B x C = A
            assert(isZero(residual));

            residual = (c.cross(a) - b).norm();  // C x A = B
            assert(isZero(residual));

            residual = (b.cross(a) + c).norm();  // B x A = -C
            assert(isZero(residual));

            residual = (c.cross(b) + a).norm();  // C x B = -A
            assert(isZero(residual));

            residual = (a.cross(c) + b).norm();  // A x C = -B
            assert(isZero(residual));

        STOP_TEST


        START_TEST

            a = Coordinate<double>(1, 0, 0);
            b = Coordinate<double>(0, 1, 0);
            c = Coordinate<double>(0, 0, 1);

            residual = a.cross(b).norm() - 1.0;
            assert(isZero(residual));

            residual = b.cross(c).norm() - 1.0;
            assert(isZero(residual));

            residual = c.cross(b).norm() - 1.0;
            assert(isZero(residual));

        STOP_TEST


        // |A x B|^2 = |A|^2 * |B|^2 - |A*B|^2

        START_TEST

            a = Coordinate<double>(1, 3, 1);
            b = Coordinate<double>(2, 1, 3);
            c = a.cross(b);

            double a_norm2 = a.norm() * a.norm();
            double b_norm2 = b.norm() * b.norm();
            double c_norm2 = c.norm() * c.norm();

            residual = c_norm2 - a_norm2 * b_norm2 + a.dot(b) * a.dot(b);
            assert(isZero(residual));

        STOP_TEST


        // C = A x B must be perpendicular to A and B

        START_TEST

            a = Coordinate<double>(1, 3, 1);
            b = Coordinate<double>(2, 1, 3);
            c = a.cross(b);

            residual = a.dot(c);
            assert(isZero(residual));

            residual = b.dot(c);
            assert(isZero(residual));

        STOP_TEST


    SUBHEADING(orthogonal())


        a = Coordinate<double>(1, 0);
        b = Coordinate<double>(0, 1);
        c = Coordinate<double>(2, -5, 3);


        START_TEST

            residual = (a.orthogonal() - b).norm();
            assert(isZero(residual));

        STOP_TEST


        START_TEST

            residual = (b.orthogonal() - a).norm();
            assert(isZero(residual));

        STOP_TEST


        START_TEST

            residual = abs(c.dot(c.orthogonal()));
            assert(isZero(residual));

        STOP_TEST


    SUBHEADING(orthonormal())


        a = Coordinate<double>(2, -5, 3);

        START_TEST

            residual = abs(a.dot(a.orthonormal()));
            assert(isZero(residual));

        STOP_TEST


        START_TEST

            residual = abs(a.orthonormal().norm() - 1);
            assert(isZero(residual));

        STOP_TEST


    SUBHEADING(normalize())


        START_TEST

            c = Coordinate<double>(-4, 7, 0);

            c.normalize();

            residual = c.norm() - 1;
            assert(isZero(residual));

        STOP_TEST


    SUBHEADING(normalized())


        START_TEST

            using std::abs;

            c = Coordinate<double>(-4, 7, 0);
            d = c.normalized();

            residual = d.norm() - 1;
            assert(isZero(residual));

            residual = abs(d.dot(c)) - c.norm();
            assert(isZero(residual));

        STOP_TEST


    SUBHEADING(size())


        START_TEST
            a = Coordinate<double>(1, 2, 3);
            assert(a.size() == 3);
        STOP_TEST


    SUBHEADING(resize())


        START_TEST
            a = Coordinate<double>(1);
            a.resize(3);
            assert(isEqual(a[0], 1.0) &&
                   isEqual(a[1], 0.0) &&
                   isEqual(a[2], 0.0));
        STOP_TEST


        START_TEST
            a = Coordinate<double>(1, 0, 0);
            a.resize(2);
            assert(isEqual(a[0], 1.0) &&
                   isEqual(a[1], 0.0));
        STOP_TEST


    SUBHEADING(reserve())


        START_TEST
            a = Coordinate<double>();
            a.reserve(100);
            assert(a.size() == 100);
        STOP_TEST


    SUBHEADING(fill())


        START_TEST
            a = Coordinate<double>(1, 0, 0);
            a.fill(2);
            assert(isEqual(a[0], 2.0) &&
                   isEqual(a[1], 2.0) &&
                   isEqual(a[2], 2.0));
        STOP_TEST


    SUBHEADING(norm())


        START_TEST
            a = Coordinate<double>(3, 4);
            assert(isEqual<double>(a.norm(), 5));
        STOP_TEST


    SUBHEADING(squaredNorm())


        START_TEST
            a = Coordinate<double>(3, 4);
            assert(isEqual<double>(a.squaredNorm(), 25));
        STOP_TEST


    SUBHEADING(toArray())


        START_TEST
            assert(Coordinate<double>(1, 2).toArray().rows() == 2);
        STOP_TEST


        START_TEST
            assert(static_cast<const Coordinate<double> >(Coordinate<double>(1, 2)).toArray().rows() == 2);
        STOP_TEST


    SUBHEADING(toMatrix())


        START_TEST
            assert(Coordinate<double>(1, 2).toMatrix().rows() == 2);
        STOP_TEST


    SUBHEADING(rand())


        START_TEST
            a = Coordinate<double>::rand(10000);
            unsigned N = a.size();

            double mean = 0;
            for (unsigned i = 0; i < N; ++i)
            {
                mean += a[i];
            }
            mean /= N;
            assert(abs(mean - 0.5) < 0.05);

            double variance = 0;
            for (unsigned i = 0; i < N; ++i)
            {
                variance += (a[i] - mean) * (a[i] - mean);
            }
            variance /= (N - 1);
            assert(abs(variance - 1.0/12) < 0.05);
        STOP_TEST


        START_TEST
            a = Coordinate<double>::rand(10000, -2, 5);
            N = a.size();

            mean = 0;
            for (unsigned i = 0; i < N; ++i)
            {
                mean += a[i];
            }
            mean /= N;
            assert(abs(mean - 1.5) / 1.5 < 0.05);

            variance = 0;
            for (unsigned i = 0; i < N; ++i)
            {
                variance += (a[i] - mean) * (a[i] - mean);
            }
            variance /= (N - 1);
            assert(abs(variance - 4.0833) / 4.0833 < 0.05);
        STOP_TEST


    SUBHEADING(randn())


        START_TEST
            a = Coordinate<double>::randn(10000);
            N = a.size();

            mean = 0;
            for (unsigned i = 0; i < N; ++i)
            {
                mean += a[i];
            }
            mean /= N;
            assert(abs(mean) < 0.05);

            variance = 0;
            for (unsigned i = 0; i < N; ++i)
            {
                variance += (a[i] - mean) * (a[i] - mean);
            }
            variance /= (N - 1);
            assert(abs(variance - 1) < 0.05);
        STOP_TEST


        START_TEST
            a = Coordinate<double>::randn(10000, -2, 5);
            N = a.size();

            mean = 0;
            for (unsigned i = 0; i < N; ++i)
            {
                mean += a[i];
            }
            mean /= N;
            assert(abs(mean - -2) / 2 < 0.05);

            variance = 0;
            for (unsigned i = 0; i < N; ++i)
            {
                variance += (a[i] - mean) * (a[i] - mean);
            }
            variance /= (N - 1);
            assert(abs(variance - 25) / 25 < 0.05);
        STOP_TEST


    SUBHEADING(operator<<() and operator>>())


        START_TEST
            a = Coordinate<double>(1, 2, 3, 4);
            std::stringstream ss;
            ss << a;
            assert(ss.str() == "(1, 2, 3, 4)");
        STOP_TEST


        START_TEST
            a.resize(4);
            ss.str("");
            ss << "(1, 2, 3, 4)";
            ss >> a;
            assert(a == Coordinate<double>(1, 2, 3, 4));
        STOP_TEST
}
