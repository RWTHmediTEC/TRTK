// Last changed on 2012-03-20.


#include <TRTK/Tools.hpp>
#include <TRTK/Transform2D.hpp>

#include "unit_test.hpp"


void unit_test_Transform2D()
{
    using namespace TRTK;
    using namespace TRTK::Tools;


    HEADING(Transform2D<T>)


    // We assume 'Transform2D(const Matrix3T &)' and
    // 'getTransformationMatrix()' to work properly.

    typedef Transform2D<double> transform2d_type;
    typedef Transform2D<float> transform2d_type2; // must be different from transform2d_type

    transform2d_type::Vector2T vec1;
    transform2d_type::Vector2T vec2;
    transform2d_type::Vector3T vec3;
    transform2d_type::Vector3T vec4;

    transform2d_type::Matrix3T A, B;

    A <<  1,  2,  3,
          4,  5,  6,
          7,  8,  9;


    SUBHEADING(Constructors)


        START_TEST
            transform2d_type transform1;
            assert(transform1.getTransformationMatrix() == transform2d_type::Matrix3T::Identity());
            assert(transform1.is_affine() == true);
        STOP_TEST


        START_TEST
            transform2d_type transform2(A);
            assert(transform2.getTransformationMatrix() == A);
            assert(transform2.is_affine() == false);
        STOP_TEST


        START_TEST
            transform2d_type transform3( 1,  2,  3,
                                         4,  5,  6,
                                         7,  8,  9);
            assert(transform3.getTransformationMatrix() == A);
            assert(transform3.is_affine() == false);
        STOP_TEST


        // Copy constructor with different scalar types

        START_TEST
            transform2d_type2 transform_of_different_type = transform3;
        STOP_TEST


    SUBHEADING(Reset)


        START_TEST
            A = transform2d_type::Matrix3T::Constant(42);
            B = transform2d_type::Matrix3T::Identity();
            transform1 = transform2d_type(A);
            transform1.reset();
            assert(transform1.getTransformationMatrix() == B);
            assert(transform1.is_affine() == true);
        STOP_TEST


    SUBHEADING(Element Access)


        // read-only access

        START_TEST
            A <<  1,  2,  3,
                  4,  5,  6,
                  7,  8,  9;

            const transform2d_type transform4 = transform2d_type(A);

            assert(isEqual<double>(transform4.a11(), 1));
            assert(isEqual<double>(transform4.a12(), 2));
            assert(isEqual<double>(transform4.a13(), 3));
            assert(isEqual<double>(transform4.a21(), 4));
            assert(isEqual<double>(transform4.a22(), 5));
            assert(isEqual<double>(transform4.a23(), 6));
            assert(isEqual<double>(transform4.a31(), 7));
            assert(isEqual<double>(transform4.a32(), 8));
            assert(isEqual<double>(transform4.a33(), 9));
        STOP_TEST


        // read-write access

        START_TEST
            A <<  1,  2,  3,
                  4,  5,  6,
                  7,  8,  9;

            transform1 = transform2d_type(transform2d_type::Matrix3T::Zero());

            transform1.a11() = 1;
            transform1.a12() = 2;
            transform1.a13() = 3;
            transform1.a21() = 4;
            transform1.a22() = 5;
            transform1.a23() = 6;
            transform1.a31() = 7;
            transform1.a32() = 8;
            transform1.a33() = 9;

            assert(transform1.getTransformationMatrix() == A);
            assert(transform1.is_affine() == false);
        STOP_TEST


    SUBHEADING(Operators)


        // Affine transformation (input argument is a Coordinate)

        START_TEST
            transform1.reset().translate(1, 1);
            Coordinate<double> a(1, 2);
            a = transform1 * a;
            assert(isEqual(a.x(), 2.0) &&
                   isEqual(a.y(), 3.0));
        STOP_TEST


        // Projective transformation (input argument is a Coordinate)

        START_TEST
            transform1.reset().translate(1, 1);
            transform1.a33() = 2;
            a = Coordinate<double>(1, 2);
            a = transform1 * a;
            assert(isEqual(a.x(), 1.0) &&
                   isEqual(a.y(), 1.5));
        STOP_TEST


        // Affine transformation (input argument is an Eigen 2D Vector)

        START_TEST
            vec1 << 1, 2;
            transform1.reset().translate(1, 1);
            vec2 = transform1 * vec1;
            assert(isEqual(vec2(0), 2.0) &&
                   isEqual(vec2(1), 3.0));
        STOP_TEST


        // Affine transformation (input argument is an Eigen 3D Vector)

        START_TEST
            vec3 << 1, 2, 1;
            transform1.reset().translate(1, 1);
            vec4 = transform1 * vec3;
            assert(isEqual(vec4(0), 2.0) &&
                   isEqual(vec4(1), 3.0) &&
                   isEqual(vec4(2), 1.0));
        STOP_TEST


        // Composition [operator*(const Matrix3T &) const]

        START_TEST
            A << 1,  0,  3,
                 0, -1,  0,
                 0,  0,  1;

            B << 1, -1,  3,
                 0, -2,  2,
                 0,  0,  1;

            transform1 = transform2d_type(1,  1,  0,
                                          0,  2,  2,
                                          0,  0,  1);
            transform2 = transform1 * A;
            assert(transform2.getTransformationMatrix().isApprox(B));
        STOP_TEST


        // Composition [operator*(const Transform2D &) const]

        START_TEST
            transform1 = transform2d_type(1,  1,  0,
                                          0,  2,  2,
                                          0,  0,  1);

            transform2 = transform2d_type(1,  0,  3,
                                          0, -1,  0,
                                          0,  0,  1);

            A << 1, -1,  3,
                 0, -2,  2,
                 0,  0,  1;

            transform3 = transform1 * transform2;
            assert(transform3.getTransformationMatrix().isApprox(A));
        STOP_TEST


        // Composition [operator>>(const Transform2D &) const]

        START_TEST
            transform1 = transform2d_type(1,  1,  0,
                                          0,  2,  2,
                                          0,  0,  1);

            transform2 = transform2d_type(1,  0,  3,
                                          0, -1,  0,
                                          0,  0,  1);

            A << 1,  1,  3,
                 0, -2, -2,
                 0,  0,  1;

            transform3 = transform1 >> transform2;
            assert(transform3.getTransformationMatrix().isApprox(A));
        STOP_TEST


    SUBHEADING(is_affine())


        START_TEST
            A << 1, 0, 2,
                 0, 1, 3,
                 0, 0, 1;
            transform1 = transform2d_type(A);
            assert(transform1.is_affine());
        STOP_TEST


        START_TEST
            A << 1, 0, 2,
                 0, 1, 3,
                 0, 1, 1;
            transform1 = transform2d_type(A);
            assert(!transform1.is_affine());
        STOP_TEST


    SUBHEADING(Translation)


        START_TEST
            A << 1, 0, 2,
                 0, 1, 3,
                 0, 0, 1;
            transform1.reset().translate(2, 3);
            assert(transform1.getTransformationMatrix().isApprox(A));
        STOP_TEST


        START_TEST
            A << 1, 0, 2,
                 0, 1, 3,
                 0, 0, 1;
            transform1.reset().translate(Coordinate<double>(2, 3));
            assert(transform1.getTransformationMatrix().isApprox(A));
        STOP_TEST


    SUBHEADING(Scaling)


        START_TEST
            A << 2, 0, 0,
                 0, 3, 0,
                 0, 0, 1;
            transform1.reset().scale(2, 3);
            assert(transform1.getTransformationMatrix().isApprox(A));
        STOP_TEST


    SUBHEADING(Shearing)


        START_TEST
            transform1.reset().shear(2, 0);
            a = Coordinate<double>(3, 4);
            a = transform1 * a;
            assert(isEqual(a.x(), 11.0) &&
                   isEqual(a.y(), 4.0));
        STOP_TEST


        START_TEST
            transform1.reset().shear(0, 3);
            a = Coordinate<double>(2, 5);
            a = transform1 * a;
            assert(isEqual(a.x(), 2.0) &&
                   isEqual(a.y(), 11.0));
        STOP_TEST


    SUBHEADING(Rotation)


        const double pi = transform2d_type::pi;

        START_TEST
            A << 1,  1,  4,
                 0,  2,  5,
                 0,  0,  1;

            B << 0, -2, -5,
                 1,  1,  4,
                 0,  0,  1;

            transform1.reset().rotate(pi/2);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        START_TEST
            transform1.reset().rotate(90, transform2d_type::DEGREES);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


    SUBHEADING(Inverse)


        START_TEST
            transform1.reset().rotate(pi/2).translate(1, 2);
            assert((transform1 * transform1.inverse()).getTransformationMatrix().isApprox(transform2d_type::Matrix3T::Identity()));
        STOP_TEST
}
