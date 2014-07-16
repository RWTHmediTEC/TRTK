// Last changed on 2012-03-20.


#include <TRTK/Tools.hpp>
#include <TRTK/Transform3D.hpp>

#include "unit_test.hpp"


void unit_test_Transform3D()
{
    using namespace TRTK;
    using namespace TRTK::Tools;


    HEADING(Transform3D<T>)


    cout << endl << "Note: The following functions are not (yet) tested:"
         << endl << "      - orthographicProjection()"
         << endl << "      - perspectiveProjection()"
         << endl << endl;


    // We assume 'Transform3D(const Matrix4T &)' and
    // 'getTransformationMatrix()' to work properly.

    typedef Transform3D<double> transform3d_type;
    typedef Transform3D<float> transform3d_type2; // must be different from transform3d_type

    transform3d_type::Vector3T vec1;
    transform3d_type::Vector3T vec2;
    transform3d_type::Vector4T vec3;
    transform3d_type::Vector4T vec4;

    transform3d_type::Matrix4T A, B;

    A <<  1,  2,  3,  4,
          5,  6,  7,  8,
          9, 10, 11, 12,
         13, 14, 15, 16;


    SUBHEADING(Constructors)


        START_TEST
            transform3d_type transform1;
            assert(transform1.getTransformationMatrix() == transform3d_type::Matrix4T::Identity());
            assert(transform1.is_affine() == true);
        STOP_TEST


        START_TEST
            transform3d_type transform2(A);
            assert(transform2.getTransformationMatrix() == A);
            assert(transform2.is_affine() == false);
        STOP_TEST


        START_TEST
            transform3d_type transform3( 1,  2,  3,  4,
                                         5,  6,  7,  8,
                                         9, 10, 11, 12,
                                        13, 14, 15, 16);
            assert(transform3.getTransformationMatrix() == A);
            assert(transform3.is_affine() == false);
        STOP_TEST


        // Copy constructor with different scalar types

        START_TEST
            transform3d_type2 transform_of_different_type = transform3;
        STOP_TEST


    SUBHEADING(Reset)


        START_TEST
            A = transform3d_type::Matrix4T::Constant(42);
            B = transform3d_type::Matrix4T::Identity();
            transform1 = transform3d_type(A);
            transform1.reset();
            assert(transform1.getTransformationMatrix() == B);
            assert(transform1.is_affine() == true);
        STOP_TEST


    SUBHEADING(Element Access)


        // read-only access

        START_TEST
            A <<  1,  2,  3,  4,
                  5,  6,  7,  8,
                  9, 10, 11, 12,
                 13, 14, 15, 16;

            const transform3d_type transform4 = transform3d_type(A);

            assert(isEqual<double>(transform4.a11(),  1));
            assert(isEqual<double>(transform4.a12(),  2));
            assert(isEqual<double>(transform4.a13(),  3));
            assert(isEqual<double>(transform4.a14(),  4));
            assert(isEqual<double>(transform4.a21(),  5));
            assert(isEqual<double>(transform4.a22(),  6));
            assert(isEqual<double>(transform4.a23(),  7));
            assert(isEqual<double>(transform4.a24(),  8));
            assert(isEqual<double>(transform4.a31(),  9));
            assert(isEqual<double>(transform4.a32(), 10));
            assert(isEqual<double>(transform4.a33(), 11));
            assert(isEqual<double>(transform4.a34(), 12));
            assert(isEqual<double>(transform4.a41(), 13));
            assert(isEqual<double>(transform4.a42(), 14));
            assert(isEqual<double>(transform4.a43(), 15));
            assert(isEqual<double>(transform4.a44(), 16));
        STOP_TEST


        // read-write access

        START_TEST
            A <<  1,  2,  3,  4,
                  5,  6,  7,  8,
                  9, 10, 11, 12,
                 13, 14, 15, 16;

            transform1 = transform3d_type(transform3d_type::Matrix4T::Zero());

            transform1.a11() = 1;
            transform1.a12() = 2;
            transform1.a13() = 3;
            transform1.a14() = 4;
            transform1.a21() = 5;
            transform1.a22() = 6;
            transform1.a23() = 7;
            transform1.a24() = 8;
            transform1.a31() = 9;
            transform1.a32() = 10;
            transform1.a33() = 11;
            transform1.a34() = 12;
            transform1.a41() = 13;
            transform1.a42() = 14;
            transform1.a43() = 15;
            transform1.a44() = 16;

            assert(transform1.getTransformationMatrix() == A);
            assert(transform1.is_affine() == false);
        STOP_TEST


    SUBHEADING(Operators)


        // Affine transformation (input argument is a Coordinate)

        START_TEST
            transform1.reset().translate(1, 1, 1);
            Coordinate<double> a(1, 2, 3);
            a = transform1 * a;
            assert(isEqual(a.x(), 2.0) &&
                   isEqual(a.y(), 3.0) &&
                   isEqual(a.z(), 4.0));
        STOP_TEST


        // Projective transformation (input argument is a Coordinate)

        START_TEST
            transform1.reset().translate(1, 1, 1);
            transform1.a44() = 2;
            a = Coordinate<double>(1, 2, 3);
            a = transform1 * a;
            assert(isEqual(a.x(), 1.0) &&
                   isEqual(a.y(), 1.5) &&
                   isEqual(a.z(), 2.0));
        STOP_TEST


        // Affine transformation (input argument is an Eigen 3D Vector)

        START_TEST
            vec1 << 1, 2, 3;
            transform1.reset().translate(1, 1, 1);
            vec2 = transform1 * vec1;
            assert(isEqual(vec2(0), 2.0) &&
                   isEqual(vec2(1), 3.0) &&
                   isEqual(vec2(2), 4.0));
        STOP_TEST


        // Affine transformation (input argument is an Eigen 4D Vector)

        START_TEST
            vec3 << 1, 2, 3, 1;
            transform1.reset().translate(1, 1, 1);
            vec4 = transform1 * vec3;
            assert(isEqual(vec4(0), 2.0) &&
                   isEqual(vec4(1), 3.0) &&
                   isEqual(vec4(2), 4.0) &&
                   isEqual(vec4(3), 1.0));
        STOP_TEST


        // Composition [operator*(const Matrix4T &) const]

        START_TEST
            A << 1, 0, 0, 0,
                 0, 1, 0, 1,
                 0, 0, 2, 2,
                 0, 0, 0, 1;

            B << 1, 0, 0, 0,
                 0, 2, 0, 3,
                 0, 0, 2, 2,
                 0, 0, 0, 1;

            transform1 = transform3d_type(1, 0, 0, 0,
                                          0, 2, 0, 1,
                                          0, 0, 1, 0,
                                          0, 0, 0, 1);
            transform2 = transform1 * A;
            assert(transform2.getTransformationMatrix().isApprox(B));
        STOP_TEST


        // Composition [operator*(const Transform3D &) const]

        START_TEST
            transform1 = transform3d_type(1, 0, 0, 0,
                                          0, 2, 0, 1,
                                          0, 0, 1, 0,
                                          0, 0, 0, 1);

            transform2 = transform3d_type(1, 0, 0, 0,
                                          0, 1, 0, 1,
                                          0, 0, 2, 2,
                                          0, 0, 0, 1);

            A << 1, 0, 0, 0,
                 0, 2, 0, 3,
                 0, 0, 2, 2,
                 0, 0, 0, 1;

            transform3 = transform1 * transform2;
            assert(transform3.getTransformationMatrix().isApprox(A));
        STOP_TEST


        // Composition [operator>>(const Transform3D &) const]

        START_TEST
            transform1 = transform3d_type(1, 0, 0, 0,
                                          0, 2, 0, 1,
                                          0, 0, 1, 0,
                                          0, 0, 0, 1);

            transform2 = transform3d_type(1, 0, 0, 0,
                                          0, 1, 0, 1,
                                          0, 0, 2, 2,
                                          0, 0, 0, 1);

            A << 1, 0, 0, 0,
                 0, 2, 0, 2,
                 0, 0, 2, 2,
                 0, 0, 0, 1;

            transform3 = transform1 >> transform2;
            assert(transform3.getTransformationMatrix().isApprox(A));
        STOP_TEST


    SUBHEADING(is_affine())


        START_TEST
            A << 1, 0, 0, 2,
                 0, 1, 0, 3,
                 0, 0, 1, 4,
                 0, 0, 0, 1;
            transform1 = transform3d_type(A);
            assert(transform1.is_affine());
        STOP_TEST


        START_TEST
            A << 1, 0, 0, 2,
                 0, 1, 0, 3,
                 0, 0, 1, 4,
                 0, 1, 0, 1;
            transform1 = transform3d_type(A);
            assert(!transform1.is_affine());
        STOP_TEST


    SUBHEADING(Translation)


        START_TEST
            A << 1, 0, 0, 2,
                 0, 1, 0, 3,
                 0, 0, 1, 4,
                 0, 0, 0, 1;
            transform1.reset().translate(2, 3, 4);
            assert(transform1.getTransformationMatrix().isApprox(A));
        STOP_TEST


        START_TEST
            A << 1, 0, 0, 2,
                 0, 1, 0, 3,
                 0, 0, 1, 4,
                 0, 0, 0, 1;
            transform1.reset().translate(Coordinate<double>(2, 3, 4));
            assert(transform1.getTransformationMatrix().isApprox(A));
        STOP_TEST


    SUBHEADING(Scaling)


        START_TEST
            A << 2, 0, 0, 0,
                 0, 3, 0, 0,
                 0, 0, 4, 0,
                 0, 0, 0, 1;
            transform1.reset().scale(2, 3, 4);
            assert(transform1.getTransformationMatrix().isApprox(A));
        STOP_TEST


    SUBHEADING(Rotation)


        const double pi = transform3d_type::pi;

        // rotate(), x-axis

        START_TEST
            A << 1, 0, 0, 4,
                 0, 2, 0, 5,
                 0, 0, 3, 6,
                 0, 0, 0, 1;

            B << 1, 0,  0,  4,
                 0, 0, -3, -6,
                 0, 2,  0,  5,
                 0, 0,  0,  1;

            transform1.reset().rotate(pi/2, transform3d_type::X_AXIS);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotate(), x-axis

        START_TEST
            transform1.reset().rotate(90, transform3d_type::X_AXIS, transform3d_type::DEGREES);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotateX()

        START_TEST
            transform1.reset().rotateX(pi/2);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotate(), y-axis

        START_TEST
            A << 1, 0, 0, 4,
                 0, 2, 0, 5,
                 0, 0, 3, 6,
                 0, 0, 0, 1;

            B <<  0, 0, 3,  6,
                  0, 2, 0,  5,
                 -1, 0, 0, -4,
                  0, 0, 0,  1;

            transform1.reset().rotate(pi/2, transform3d_type::Y_AXIS);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotate(), y-axis

        START_TEST
            transform1.reset().rotate(90, transform3d_type::Y_AXIS, transform3d_type::DEGREES);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotateY()

        START_TEST
            transform1.reset().rotateY(pi/2);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotate(), z-axis

        START_TEST
            A << 1, 0, 0, 4,
                 0, 2, 0, 5,
                 0, 0, 3, 6,
                 0, 0, 0, 1;

            B << 0, -2, 0, -5,
                 1,  0, 0,  4,
                 0,  0, 3,  6,
                 0,  0, 0,  1;

            transform1.reset().rotate(pi/2, transform3d_type::Z_AXIS);
            assert((transform1 * A - B).norm() < 10e-14);
        STOP_TEST


        // rotate(), z-axis

        START_TEST
            transform1.reset().rotate(90, transform3d_type::Z_AXIS, transform3d_type::DEGREES);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotateZ()

        START_TEST
            transform1.reset().rotateZ(pi/2);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotate(), arbitrary axis

        START_TEST

            A << 1, 0, 0, 4,
                 0, 2, 0, 5,
                 0, 0, 3, 6,
                 0, 0, 0, 1;

            B <<  0.36,  1.60, -1.44,  2.56,
                 -0.80,  0.00, -1.80, -6.80,
                 -0.48,  1.20,  1.92,  4.92,
                  0.00,  0.00,  0.00,  1.00;

            Coordinate<double> axis(3, 0, -4);

            transform1.reset().rotateAxis(pi/2, axis);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


        // rotate(), arbitrary axis

        START_TEST
            transform1.reset().rotateAxis(90, axis, transform3d_type::DEGREES);
            assert((transform1 * A).isApprox(B));
        STOP_TEST


    SUBHEADING(Inverse)


        START_TEST
            transform1.reset().rotateZ(pi/2).translate(1, 2, 3);
            assert((transform1 * transform1.inverse()).getTransformationMatrix().isApprox(transform3d_type::Matrix4T::Identity()));
        STOP_TEST
}
