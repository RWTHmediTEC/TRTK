/*
    This class implements a simple camera model as often used in computer vision.

    Copyright (C) 2010 - 2019 Christoph Hänisch

    SurgiTAIX AG
    Kaiserstr. 100, TPH 1, 2. Et.
    52134 Herzogenrath
    Germany
*/

/**
 * \file PinholeCameraModel.hpp
 * \brief This file contains the \ref TRTK::PinholeCameraModel "PinholeCameraModel" class.
 */


#ifndef PINHOLE_CAMERA_MODEL_HPP_4231789408
#define PINHOLE_CAMERA_MODEL_HPP_4231789408


#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#ifdef CPPOPTLIB_FOUND
    #include <cppoptlib/meta.h>
    #include <cppoptlib/problem.h>
    #include <cppoptlib/solver/bfgssolver.h>
#endif // CPPOPTLIB_FOUND

#include "ErrorObj.hpp"
#include "Coordinate.hpp"
#include "Range.hpp"


namespace TRTK
{

    /**************************************************************
     *                      Helper function                       *
     **************************************************************/

    /**
     * \tparam T scalar (floating point) type
     *
     * \brief  RQ decomposition of a square matrix.
     *
     * \param [in] A    square matrix
     *
     * This decomposition is unique if A is of full rank (i.e., if A is
     * invertable). Then, R is computed in such a way that all diagonal
     * elements are positive.
     *
     * Note, this function computes the RQ decomposition of a square matrix
     * and not the QR decomposition. The order of the orthogonal matrix and
     * the upper triangular matrix is reversed.
     *
     * \returns This function returns the tuple (R, Q).
     *
     * \references https://math.stackexchange.com/questions/1640695/rq-decomposition/1640762
     *
     * \author Christoph Hänisch
     * \version 1.0.0
     * \date last changed on 2019-05-22
     */

    template <class T>
    std::tuple<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>>
        computeRQDecomposition(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> & A)
    {
        /*
        This RQ decomposition scheme is based on the math.stackexchange article
        https://math.stackexchange.com/questions/1640695/rq-decomposition/1640762.

        Algorithm
        =========

        Given the square matrix A.

        1. Define P as the exchange matrix, i.e., an anti-diagonal matrix with ones along the counter-diagonal
        2. Compute A_tilda = P * A
        3. Compute the decomposition of A_tilda^T = Q_tilda * R_tilda
        4. Set Q := P * Q_tilda^T
        5. Set R := P * R_tilda^T * P

        Then, it holds that

            R * Q = (P * R_tilda^T * P)(P * Q_tilda^T) = P * R_tilda^T * Q_tilda^T
                  = P * (Q_tilda * R_tilda)^T = P * (A_tilda^T)^T
                  = P * A_tilda = P * P * A = A

        since P = P^T and P * P = I.

        Uniqueness
        ==========

        Eigen's Housholder QR decomposition is not unique. This can be easily rectified
        by changing QR to QDDR=(QD)(DR) where D = diag(sign(r11), ..., sign(rnn)). That
        is, a diagonal matrix D is formed whose entries are the signs of the entries of
        the main diagonal of R. Then the rows of DR are altered in such a way that the
        new diagonal entries are all positive. Since DD^T = DD = I it holds that
        (QD)(DR) = QR. Also, D is orthogonal and so is QD. Thus, the result is still a
        valid QR decomposition.

        This circumstance is used below when computing the QR decomposition of A_tilda^T.
        */

        using namespace std;
        using namespace Eigen;

        using Matrix = Eigen::Matrix<T, Dynamic, Dynamic>;

        const bool INPUT_MATRIX_SQUARE = A.cols() == A.rows();
        assert(INPUT_MATRIX_SQUARE);

        const int n = (int) A.cols();

        Matrix P = Matrix::Zero(n, n);
        for (int i=0; i < n; ++i) P(n-1-i, i) = 1;

        Matrix A_tilda = P * A;  // reverse rows of A
        HouseholderQR<Matrix> qr(A_tilda.transpose());
        Matrix Q_tilda = qr.householderQ();
        Matrix R_tilda = qr.matrixQR().triangularView<Upper>();
        DiagonalMatrix<T, Dynamic, Dynamic> D(n);
        for (int i = 0; i < n; ++i) D.diagonal()(i) = R_tilda(i, i) >= 0 ? 1 : -1;
        Q_tilda = Q_tilda * D;
        R_tilda = D * R_tilda; // make R unique by making the diagonal entries positive
        Matrix Q = P * Q_tilda.transpose();
        Matrix R = P * R_tilda.transpose() * P;  // reverse rows and columns of R_tilda

        const bool Q_IS_ORTHOGONAL = (Q * Q.transpose() - Matrix::Identity(n, n)).norm() < 1e-7;
        assert(Q_IS_ORTHOGONAL);

        return make_pair(R, Q);
    }


    /**************************************************************
     *                     PinholeCameraModel                     *
     **************************************************************/

    /**
     * \tparam T scalar (floating point) type
     *
     * \brief  A simple pinhole camera model.
     *
     * This class implements a simple pinhole camera model. The model parameters
     * can be set / modified by the user or estimated given a set of corresponding
     * point pairs.
     *
     * \image html "Pinhole camera model.png" "Pinhole camera model."
     *
     * The camera is modeled by a projection matrix \f$ T_{proj} \f$ which maps
     * a three-dimensional point \f$ p \f$ to the two-dimensional point
     * \f$ p' = T_{proj} * p \f$ that lies in the image plane of the camera. The
     * optical properties of the camera are defined by the focal lengths \f$ f_x \f$
     * and \f$ f_y \f$, the image center \f$ (c_x, c_y) \f$ where the optical axis
     * intersects the image plane, and a parameter \f$ \tau \f$ which models the
     * skew of the image plane axes. The camera's pose (i.e., its location and
     * orientation) in world coordinates is described by the mapping \f$ T_{pose} \f$
     * which maps from the world coordinate system to the camera coordinate system.
     * Thus, the overall system is described with the following equation:
     *
     * \f[
     *     p' \sim T_{proj} T_{pose} p
     * \f]
     *
     * Here, the matrix entries are described in more detail:
     *
     * \f[
     *     \begin{pmatrix}
     *         x'  \\  y'  \\  1
     *     \end{pmatrix}
     *     \sim
     *     \begin{pmatrix}
     *         f_x  &  \tau  &  c_x  &  0  \\
     *         0    &  f_y   &  c_y  &  0  \\
     *         0    &  0     &  1    &  0
     *     \end{pmatrix}
     *     \begin{pmatrix}
     *         r_{11}  &  r_{12}  &  r_{13}  &  t_1  \\
     *         r_{21}  &  r_{22}  &  r_{23}  &  t_2  \\
     *         r_{31}  &  r_{32}  &  r_{33}  &  t_3  \\
     *         0       &  0       &  0       &  1
     *     \end{pmatrix}
     *     \begin{pmatrix}
     *         x  \\  y \\  z  \\  1
     *     \end{pmatrix}
     * \f]
     *
     * For more details, please be referred to [1], [2], [3], and [4].
     *
     * \par Example:
     *
     * \code
     *
     * #include <iostream>
     * #include <iomanip>
     * #include <vector>
     *
     * #include<Eigen/Core>
     * #include<Eigen/StdVector>
     *
     * #include <TRTK/Coordinate.hpp>
     * #include <TRTK/PinholeCameraModel.hpp>
     * #include <TRTK/Tools.hpp>
     * #include <TRTK/Transform3D.hpp>
     *
     *
     * using namespace Eigen;
     * using namespace std;
     * using namespace TRTK;
     * using namespace TRTK::Tools;
     *
     * using Point3D = PinholeCameraModel<double>::Vector3T;
     * using Point2D = PinholeCameraModel<double>::Vector2T;
     *
     *
     * struct GenerateTestData
     * {
     *     GenerateTestData()
     *     {
     *         // Instantiate a pinhole camera model.
     *
     *         PinholeCameraModel<double> camera_model;
     *
     *         // Set the intrinsic camera parameters.
     *
     *         camera_model.setFocalLengths(80, 79);
     *         camera_model.setImageCenter(250, 251);
     *         camera_model.setSkew(0.1);
     *
     *         // Set the extrinsic camera parameters.
     *
     *         Transform3D<double> T;
     *         T.rotateAxis(40, Point3D(1, 2, 3), Transform3D<double>::DEGREES);
     *         T.translate(130, 135, 90);
     *         camera_model.setExtrinsicParameters(T.getTransformationMatrix());
     *
     *         // Generate some test data.
     *
     *         const int number_of_points = 6;
     *
     *         for (int i = 0; i < number_of_points; ++i) // generate points in the world COS
     *         {
     *             auto x = randn(0.0, 30.0);    // in the camera COS
     *             auto y = randn(0.0, 30.0);    // in the camera COS
     *             auto z = randn(600.0, 10.0);  // in the camera COS
     *             Point3D p = T.inverse() * Point3D(x, y, z);  // in the world COS
     *             points_world.emplace_back(p);
     *         }
     *
     *         for (auto const & p_W : points_world) // generate points in the camera COS
     *         {
     *             Point2D p_D = camera_model.transform(p_W);
     *             points_display.push_back(p_D);
     *         }
     *     }
     *
     *     vector<Point3D> points_world;
     *     vector<Point2D> points_display; // image plane
     * };
     *
     *
     * int main()
     * {
     *
     *     srand(0);
     *     GenerateTestData test_data;
     *
     *     PinholeCameraModel<double> model;
     *     auto rmse = model.estimate(make_range(test_data.points_world), make_range(test_data.points_display));
     *
     *     cout << setprecision(4);
     *     cout << fixed;
     *     cout << "RMSE: " << rmse << endl << endl;
     *     cout << "Extrinsic camera parameters: " << endl << model.getExtrinsicParameters() << endl << endl;
     *     cout << "Intrinsic camera parameters: " << endl << model.getIntrinsicParameters() << endl;
     *
     *     return 0;
     * }
     *
     * \endcode
     *
     * \par Output:
     *
     * \verbatim

       RMSE: 0.0000

       Extrinsic camera parameters:
         0.7828  -0.4820   0.3937 130.0000
         0.5488   0.8329  -0.0715 135.0000
        -0.2935   0.2721   0.9164  90.0000
         0.0000   0.0000   0.0000   1.0000

       Intrinsic camera parameters:
        80.0000   0.1000 250.0000   0.0000
         0.0000  79.0000 251.0000   0.0000
         0.0000   0.0000   1.0000   0.0000

       \endverbatim
     *
     * \note
     *
     * The \c estimateWithConstraints() function is only available if the \macro{CPPOPTLIB_FOUND}
     * macro is defined. Then, the include directory of the CppOptimizationLibrary must be set
     * appropriately. In CMake you might want to insert the following lines in your
     * \c CMakeLists.txt file.
     *
     * \verbatim
       find_path(CPPOPTLIB NAMES cppoptlib cppoptlib-1.0.0)
       if(CPPOPTLIB)
           set(CPPOPTLIB_FOUND 1)
           include_directories(${CPPOPTLIB})
           add_definitions(-DCPPOPTLIB_FOUND)
       endif()
       \endverbatim
     *
     * The library can be found at https://github.com/PatWie/CppNumericalSolvers .
     *
     * \references
     *
     * [1] Hartley and Zisserman, "Multiple view geometry in computer vision", 2003
     *
     * [2] Szeliski, "Computer Vision - Algorithms and Applications", 2011, Springer
     *
     * [4] Sutherland, "Three-Dimensional Data Input by Tablet", 1974, IEEE
     *
     * [3] Tuceryan et al., "Single point active alignment method (SPAAM) for optical
     *     see-through HMD calibration for AR", 2000, IEEE
     *
     * \see TRTK::Transform3D
     *
     * \author Christoph Hänisch
     * \version 1.1.0
     * \date last changed on 2019-07-02
     */

    template <class T>
    class PinholeCameraModel
    {
    public:
        enum Error {
            NOT_ENOUGH_INPUT_DATA,
            UNEQUAL_CARDINALITY_OF_INPUT_SETS,
            UNKNOWN_ESTIMATION_METHOD,
            UNKNOWN_ERROR
        };

        enum Constraints {                          ///< Constraints w.r.t. the projection matrix.
            NO_SKEW,                                ///< The parameter tau is assumed to be zero (no skewed image plane).
            SAME_FOCAL_LENGTHS,                     ///< The focal lengths are assumed to be equal to each other.
            SAME_FOCAL_LENGTHS_AND_NO_SKEW,         ///< The focal lengths are assumed to be equal to each other and there is no skewed image plane.
        };

        using value_type = T;
        using Vector2T =  Eigen::Matrix<T, 2, 1>;
        using Vector3T =  Eigen::Matrix<T, 3, 1>;
        using Vector4T =  Eigen::Matrix<T, 4, 1>;
        using VectorXT =  Eigen::Matrix<T, Eigen::Dynamic, 1>;
        using Matrix3T =  Eigen::Matrix<T, 3, 3>;
        using Matrix34T =  Eigen::Matrix<T, 3, 4>;
        using Matrix4T =  Eigen::Matrix<T, 4, 4>;
        using MatrixXT =  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
        using Point =  Coordinate<T>;

        PinholeCameraModel();                              ///< Constructor.
        virtual ~PinholeCameraModel();                     ///< Destructor.

        // Getters

        Vector3T getCameraPosition() const;         ///< Returns the camera's position in world coordinates.
        Matrix3T getCameraOrientation() const;      ///< Returns the camera's orientation in world coordinates.

        Matrix4T getExtrinsicParameters() const;    ///< Returns the extrinsic camera parameters (position and orientation) as homogeneous matrix.
        Matrix34T getIntrinsicParameters() const;   ///< Returns the intrinsic camera parameters.
        Matrix34T getCameraParameters() const;      ///< Returns the projection matrix.

        std::tuple<T, T> getFocalLengths() const;   ///< Returns the focal lengths in x- and y-direction.
        std::tuple<T, T> getImageCenter() const;    ///< Returns the image center a.k.a. principal point.
        T getSkew() const;                          ///< Returns the skew in the image plane.

        // Setters

        void setCameraPosition(const Vector3T & position);          ///< Set the camera's position in world coordinates.
        void setCameraOrientation(const Matrix3T & orientation);    ///< Set the camera's orientation in world coordinates.

        void setExtrinsicParameters(const Matrix4T & parameters);   ///< Return the extrinsic camera parameters (position and orientation) as homogeneous matrix.
        void setIntrinsicParameters(const Matrix34T & parameters);  ///< Return the intrinsic camera parameters.

        void setFocalLengths(T f_x, T f_y);                         ///< Set the focal lengths in x- and y-direction.
        void setImageCenter(T c_x, T c_y);                          ///< Set the image center a.k.a. principal point.
        void setSkew(T skew);                                       ///< Return the skew in the image plane.

        // Other methods

        Vector2T operator*(const Vector3T & point) const;           ///< Transforms / projects a 3D point onto the image plane.
        Vector2T transform(const Vector3T & point) const;           ///< Transforms / projects a 3D point onto the image plane.

        T estimate(const Range<Vector3T> & points_world, const Range<Vector2T> & points_display);   ///< Estimates the intrinsic and extrinsic camera parameters from two corresponding point sets.

        #ifdef CPPOPTLIB_FOUND
            T estimateWithConstraints(const Range<Vector3T> & points_world, const Range<Vector2T> & points_display, Constraints constraints = SAME_FOCAL_LENGTHS_AND_NO_SKEW, bool initialize = true, bool constrained_decomposition = true);   ///< Estimates the intrinsic and extrinsic camera parameters from two corresponding point sets.
        #endif // CPPOPTLIB_FOUND

    private:
        Matrix4T T_pose = Matrix4T::Identity();
        Matrix34T T_proj = Matrix34T::Identity();
    };


    template <class T>
    PinholeCameraModel<T>::PinholeCameraModel()
    {
    }


    template <class T>
    PinholeCameraModel<T>::~PinholeCameraModel()
    {
    }


    /**
     * \param [in] points_world         3d points in the world coordinate system.
     * \param [in] points_display       Corresponding 2d points in the image plane.
     *
     * The intrinsic and extrinsic camera parameters are estimated from two
     * corresponding point sets (i.e., the i-th elements of both point sets
     * correspond to each other). It is assumed that the camera is fixed in
     * the world coordinate system (COS) and that the transition from the world
     * COS into the camera COS is described by the pose matrix.
     *
     * The 2d points \f$ (x'_i, y'_i)^T \f$ in \p points_display are assumed to
     * be normalized:
     *
     * \f[
     *     \begin{pmatrix}
     *         x'_i \\
     *         y'_i
     *     \end{pmatrix}
     *     =
     *     \begin{pmatrix}
     *         u_i / w_i \\
     *         v_i / w_i
     *     \end{pmatrix},
     *
     *     \quad
     *
     *     \begin{pmatrix}
     *         u_i \\
     *         v_i \\
     *         w_i
     *     \end{pmatrix}
     *     =
     *     T_{proj} T_{pose}
     *     \begin{pmatrix}
     *         x_i \\
     *         y_i \\
     *         z_i \\
     *         1
     *     \end{pmatrix}
     * \f]
     *
     * At least 6 corresponding point pairs must be given. The focal lengths are
     * assumed to be positive.
     *
     * \returns
     *
     * If \f$ f \f$ describes the above mapping, i.e.,
     * \f$ p' = f(p) \f$ with \f$ p' = (x', y')^T \f$ and \f$ p = (x, y, z)^T \f$,
     * then this function returns the root mean square error
     *
     * \f[
     *     rmse = \sqrt{ \frac{1}{N} \sum_{i=1}^N \left\Vert \hat f(p_i) - p'_i \right\Vert^2 }
     * \f]
     *
     * where \f$ \hat f \f$ is the function estimate.
     *
     * \throw ErrorObj  If the input set sizes do not match, an error object is thrown and
     *                  its error code is set to \c UNEQUAL_CARDINALITY_OF_INPUT_SETS.
     *
     * \throw ErrorObj  If there are not enough point pairs (at least six), an error
     *                  object is thrown and its error code is set to \c NOT_ENOUGH_POINTS.
     */

    template <class T>
    T PinholeCameraModel<T>::estimate(const Range<Vector3T> & points_world, const Range<Vector2T> & points_display)
    {
        /*

        Three different implementations were tested. First variant 1 as described in [4] which
        worked fine for noise-free data but was very unstable for noisy data---especially w.r.t.
        the decomposition. Both variants 2 and 3 were described in [5] and are much more stable
        in the presence of noise. Also the RMSE is much lower and the decomposition yields
        reasonable values. Variant 3 is possibly numerically slightly less stable than variant 2
        but is much faster and thus the final choice.

        All three implementation are kept in this header file for documentation purposes.


        References
        ==========

        [1] Hartley and Zisserman, "Multiple view geometry in computer vision", 2003

        [2] Szeliski, "Computer Vision - Algorithms and Applications", 2011, Springer

        [3] Sutherland, "Three-Dimensional Data Input by Tablet", 1974, IEEE

        [4] Tuceryan et al., "Single point active alignment method (SPAAM) for optical
            see-through HMD calibration for AR", 2000, IEEE

        [5] Carl Olsson, Computer Vision, "Lecture 3: Camera Calibration, DLT, SVD", 2014,
            http://www.maths.lth.se/matematiklth/personal/calle/datorseende14/notes/forelas3.pdf

        */

        using namespace std;
        using namespace Eigen;

        // Check for assertions.

        int number_of_points = (int) points_world.size();

        if (points_world.size() != points_display.size())
        {
            ErrorObj error;
            error.setClassName("PinholeCameraModel<T>");
            error.setFunctionName("estimate");
            error.setErrorMessage("Set sizes do not match.");
            error.setErrorCode(UNEQUAL_CARDINALITY_OF_INPUT_SETS);
            throw error;
        }

        if (number_of_points < 6)
        {
            ErrorObj error;
            error.setClassName("PinholeCameraModel<T>");
            error.setFunctionName("estimate");
            error.setErrorMessage("Not enough points (6) to estimate the camera parameters.");
            error.setErrorCode(NOT_ENOUGH_INPUT_DATA);
            throw error;
        }

        enum Variants
        {
            VARIANT1,   // Tuceryan (or Sutherland but formulated as a null-space problem)
            VARIANT2,   // Olsson (estimation of scaling factors)
            VARIANT3    // Olsson (cross product)
        } variant = VARIANT3;

        Matrix34T A; // camera matrix to be estimated by one of the following variants and then to be decomposed

        switch(variant)
        {
            case VARIANT1:
            {
                /*

                This implementation is based on the work of Tuceryan et al. [4]. The QR decomposition
                is not mentioned in the paper, but solving for the equations manually as described in
                [5] yields the exact same results (this was thoroughly tested). Note, the parameter
                names and variable names do slightly vary from this article. The algorithm is
                essentially the same as the direct linear transformation algorithm as described in [3].


                Algorithm
                =========

                Given: N corresponding point pairs (x'_i, y'_i) and (x_i, y_i, z_i)

                The projection can be described with the following relation

                    p' ~ T_proj * T_pose * p = A * p    (where A = [a_ij])

                or written-out

                    | x' |     | u |     | fx  tau  cx  0 |     | r11  r12  r13  t1 |     | x |
                    | y' |  ~  | v |  =  | 0   fy   cy  0 |  *  | r21  r22  r23  t2 |  *  | y |     (1)
                    | 1  |     | w |     | 0   0    1   0 |     | r31  r32  r33  t3 |     | z |
                                                                | 0    0    0    1  |     | 1 |.

                The resulting vector (x', y', 1) equals the right side of the relation up
                to a scaling factor. Due to the normalization properties of homogeneous
                coordinates it holds that

                    x' = u / w      (2)
                    y' = v / w.

                Rearranging equation (2) and inserting equation (1) into it yields

                    u = x' * w   <==>   a11 * x + a12 * y + a13 * z + a14 = x' * (a31 * x + a32 * y + a33 * z + a34)    (3)
                    v = y' * w   <==>   a21 * x + a22 * y + a23 * z + a24 = y' * (a31 * x + a32 * y + a33 * z + a34)

                By defining a vector c := (a11, a12, ..., a34)^T with the sough coefficients
                equation (3) can again be rewritten to

                    B * c = 0

                where

                    B  =  | x  y  z  1  0  0  0  0  -x'x  -x'y  -x'z  -x' |     (4)
                          | 0  0  0  0  x  y  z  1  -y'x  -y'y  -y'z  -y' |.

                Actually, B must be formed from all measurements B = [B_1^T, B_2^T, ..., B_N^T]^T.
                Here, the indices were dropped for ease in notation.

                Now, the sought parameters can be found as the null-space of B. For this, a
                Lagrange function L := || B * c ||^2 - lambda (|| c ||^2  - 1) is defined and
                minimized. This can be justified with the fact that projection matrices (and
                thus also the vector c) may be arbitrarily scaled without changing their
                properties. Hence, c might be constrained to have a magnitude of 1 to avoid the
                trivial solution. This leads to an eigenvalue problem which can be solved in a
                numerically more stable way using a singular value decomposition (SVD). Then, c
                is the singular vector associated with the smallest singular value of V with
                B = U * S * V^T.

                Finally, the matrix A must be decomposed into the projection matrix T_proj and
                the pose matrix T_pose. Using block matrices equation (1) can also be written as

                    | U  0 |  *  | R    t |  =  | UR  Ut |  =  A      (5)
                                 | 0^T  1 |

                where U is the left part of T_proj (which is an upper triangular 3-by-3 matrix)
                and where R is the upper left 3-by-3 sub-matrix of T_pose which is orthogonal; t
                is the 3d translation vector of T_pose. That is, U * R is the left 3-by-3 matrix
                of A and a product of an upper triangular matrix and an orthogonal matrix. This
                can be factorized with the RQ algorithm (similar to the QR algorithm). The
                remaining variables can then be easily computed.

                Note, not every implementation of the QR / RQ algorithm returns a unique upper
                triangular matrix. The above implementation does this by returning a triangular
                matrix whose diagonals are always positive. This is in accordance with the
                projection matrix of the pinhole camera model where both focal lengths are
                assumed to be positive.

                Note, U may be arbitrarily scaled and should be normalized in such a way that
                the coefficient u33 is equal to 1 (as is the case in the above definition of
                the projection matrix).

                */

                // Form the matrix B.

                MatrixXT B;
                B.resize(2 * number_of_points, 12);
                int i = 0;

                for (auto const & p_W : points_world)
                {
                    auto & x = p_W.x();
                    auto & y = p_W.y();
                    auto & z = p_W.z();

                    auto const & p_D = points_display.currentItem();
                    auto & x_dash = p_D.x();
                    auto & y_dash = p_D.y();
                    points_display.next();

                    B(i,  0) = x;
                    B(i,  1) = y;
                    B(i,  2) = z;
                    B(i,  3) = 1;
                    B(i,  4) = 0;
                    B(i,  5) = 0;
                    B(i,  6) = 0;
                    B(i,  7) = 0;
                    B(i,  8) = -x_dash * x;
                    B(i,  9) = -x_dash * y;
                    B(i, 10) = -x_dash * z;
                    B(i, 11) = -x_dash;

                    B(i+1,  0) = 0;
                    B(i+1,  1) = 0;
                    B(i+1,  2) = 0;
                    B(i+1,  3) = 0;
                    B(i+1,  4) = x;
                    B(i+1,  5) = y;
                    B(i+1,  6) = z;
                    B(i+1,  7) = 1;
                    B(i+1,  8) = -y_dash * x;
                    B(i+1,  9) = -y_dash * y;
                    B(i+1, 10) = -y_dash * z;
                    B(i+1, 11) = -y_dash;

                    i = i + 2;
                }

                // Compute the parameter vector c and rearrange c to A.

                JacobiSVD<MatrixXT> svd(B, ComputeThinV);
                VectorXT c = svd.matrixV().col(svd.matrixV().cols() - 1);
                A = Map<Matrix<T, 3, 4, RowMajor>>(c.data());

                break;
            }

            case VARIANT2: // estimate projection matrix and scale factors
            {
                /*

                This implementation is based on the computer vision lecture notes of Carl Olsson [5].
                The RQ decomposition differs from the description found in the lecture notes, but
                solving for the equations manually as described in [5] yields the exact same results
                (this was thoroughly tested). Note, the parameter names and variable names may vary
                from the reference.


                Algorithm
                =========

                Given: N corresponding point pairs (x'_i, y'_i) and (x_i, y_i, z_i)

                The projection can be described with the following relation

                    p'_i ~ T_proj * T_pose * p_i = A * p_i    (where A = [a_kl])

                or

                    lambda_i * p'_i = A * p_i       (1)

                using an unknown scaling factor lambda_i. With

                        | A1^T |                    | x'_i |
                    A = | A2^T |      and    p'_i = | y'_i |
                        | A3^T |                    | z'_i |

                equation (1) can be rewritten as

                    p_i^T * A1 - lambda_i * x'_i = 0
                    p_i^T * A2 - lambda_i * y'_i = 0
                    p_i^T * A3 - lambda_i * z'_i = 0

                or in matrix notation  (z'_i = 1 due to the normalization)

                    | p_i^T   0       0       -x'_i |     | A1       |     | 0 |
                    | 0       p_i^T   0       -y'_i |  *  | A2       |  =  | 0 |
                    | 0       0       p_i^T   -1    |     | A3       |     | 0 |.
                                                          | lambda_i |

                For multiple entries we then have

                    | p_1^T   0       0       -x'_1   0       0       ... |
                    | 0       p_1^T   0       -y'_1   0       0       ... |     | A1       |
                    | 0       0       p_1^T   -1      0       0       ... |     | A2       |
                    | p_2^T   0       0       0       -x'_2   0       ... |     | A3       |
                    | 0       p_2^T   0       0       -y'_2   0       ... |  *  | lambda_1 |  =  0.
                    | 0       0       p_2^T   0       -1      0       ... |     | lambda_2 |
                    | p_3^T   0       0       0       0       -x'_3   ... |     | lambda_3 |
                    | 0       p_3^T   0       0       0       -y'_3   ... |     | ...      |
                    | 0       0       p_3^T   0       0       -1      ... |
                    | ...     ...     ...     ...     ...     ...     ... |

                Or

                    M * c = 0.

                Now, the sought parameters can be found as the null-space of M. For this, a
                Lagrange function L := || M * c ||^2 - lambda (|| c ||^2  - 1) is defined and
                minimized. This can be justified with the fact that projection matrices (and
                thus also the vector c) may be arbitrarily scaled without changing their
                properties. Hence, c might be constrained to have a magnitude of 1 to avoid the
                trivial solution. This leads to an eigenvalue problem which can be solved in a
                numerically more stable way using a singular value decomposition (SVD). Then, c
                is the singular vector associated with the smallest singular value of V with
                B = U * S * V^T.

                Finally, the matrix A must be decomposed into the projection matrix T_proj and
                the pose matrix T_pose. Using block matrices equation (1) can also be written as

                    | U  0 |  *  | R    t |  =  | UR  Ut |  =  A      (5)
                                 | 0^T  1 |

                where U is the left part of T_proj (which is an upper triangular 3-by-3 matrix)
                and where R is the upper left 3-by-3 sub-matrix of T_pose which is orthogonal; t
                is the 3d translation vector of T_pose. That is, U * R is the left 3-by-3 matrix
                of A and a product of an upper triangular matrix and an orthogonal matrix. This
                can be factorized with the RQ algorithm (similar to the QR algorithm). The
                remaining variables can then be easily computed.

                Note, not every implementation of the QR / RQ algorithm returns a unique upper
                triangular matrix. The above implementation does this by returning a triangular
                matrix whose diagonals are always positive. This is in accordance with the
                projection matrix of the pinhole camera model where both focal lengths are
                assumed to be positive.

                Note, U may be arbitrarily scaled and should be normalized in such a way that
                the coefficient u33 is equal to 1 (as is the case in the above definition of
                the projection matrix).

                */

                // Form the matrix M.

                MatrixXT M = MatrixXT::Zero(3 * number_of_points, 12 + number_of_points);
                int i = 0;

                for (auto const & p_W : points_world)
                {
                    auto const & p_D = points_display.currentItem();
                    auto & x = p_D.x();
                    auto & y = p_D.y();
                    points_display.next();

                    M.block<1, 4>(3 * i + 0, 0) = p_W.homogeneous();
                    M.block<1, 4>(3 * i + 1, 4) = p_W.homogeneous();
                    M.block<1, 4>(3 * i + 2, 8) = p_W.homogeneous();

                    M(3 * i + 0, 12 + i) = -x;
                    M(3 * i + 1, 12 + i) = -y;
                    M(3 * i + 2, 12 + i) = -1;

                    ++i;
                }

                // Compute the parameter vector c and rearrange c to A.

                BDCSVD<MatrixXT> svd(M, ComputeThinV);
                VectorXT c = svd.matrixV().col(svd.matrixV().cols() - 1);
                A = Map<Matrix<T, 3, 4, RowMajor>>(c.data());

                break;
            }

            case VARIANT3: // cross-product
            {
                /*

                This implementation is based on the computer vision lecture notes of Carl Olsson [5].
                The RQ decomposition differs from the description found in the lecture notes, but
                solving for the equations manually as described in [5] yields the exact same results
                (this was thoroughly tested). Note, the parameter names and variable names may vary
                from the reference.


                Algorithm
                =========

                Given: N corresponding point pairs (x'_i, y'_i) and (x_i, y_i, z_i)

                The projection can be described with the following relation

                    p'_i ~ T_proj * T_pose * p_i = A * p_i    (where A = [a_kl])

                or

                    lambda_i * p'_i = A * p_i       (1)

                using an unknown scaling factor lambda_i. Stated in other words, the vectors p'_i and
                A * p_i are parallel to each other. This can be stated more formally using the cross
                product

                    p'_i  x  A * p_i  =  0.         (2)

                By setting

                        | A1^T |                    | x'_i |
                    A = | A2^T |      and    p'_i = | y'_i |
                        | A3^T |                    | z'_i |

                equation (2) can be rewritten as

                    |  y'_i * A3^T * p_i  -  z'_i * A2^T * p_i  |
                    |  z'_i * A1^T * p_i  -  x'_i * A3^T * p_i  |  =  0.
                    |  x'_i * A2^T * p_i  -  y'_i * A1^T * p_i  |

                With z_i = 1 it can again be rewritten as

                    |  0           -1 * p_i     y' * p_i   |     | A1 |
                    |  1 * p_i      0           -x' * p_i  |  *  | A2 |  =  0
                    |  -y' * p_i    x' * p_i    0          |     | A3 |

                or

                    M_i * c = 0.

                For multiple entries we then have

                        | M1  |
                    M = | M2  |.
                        | M3  |
                        | ... |

                Now, the sought parameters can be found as the null-space of M. For this, a
                Lagrange function L := || M * c ||^2 - lambda (|| c ||^2  - 1) is defined and
                minimized. This can be justified with the fact that projection matrices (and
                thus also the vector c) may be arbitrarily scaled without changing their
                properties. Hence, c might be constrained to have a magnitude of 1 to avoid the
                trivial solution. This leads to an eigenvalue problem which can be solved in a
                numerically more stable way using a singular value decomposition (SVD). Then, c
                is the singular vector associated with the smallest singular value of V with
                B = U * S * V^T.

                Finally, the matrix A must be decomposed into the projection matrix T_proj and
                the pose matrix T_pose. Using block matrices equation (1) can also be written as

                    | U  0 |  *  | R    t |  =  | UR  Ut |  =  A      (5)
                                 | 0^T  1 |

                where U is the left part of T_proj (which is an upper triangular 3-by-3 matrix)
                and where R is the upper left 3-by-3 sub-matrix of T_pose which is orthogonal; t
                is the 3d translation vector of T_pose. That is, U * R is the left 3-by-3 matrix
                of A and a product of an upper triangular matrix and an orthogonal matrix. This
                can be factorized with the RQ algorithm (similar to the QR algorithm). The
                remaining variables can then be easily computed.

                Note, not every implementation of the QR / RQ algorithm returns a unique upper
                triangular matrix. The above implementation does this by returning a triangular
                matrix whose diagonals are always positive. This is in accordance with the
                projection matrix of the pinhole camera model where both focal lengths are
                assumed to be positive.

                Note, U may be arbitrarily scaled and should be normalized in such a way that
                the coefficient u33 is equal to 1 (as is the case in the above definition of
                the projection matrix).

                */

                // Form the matrix M.

                MatrixXT M = MatrixXT(3 * number_of_points, 12);
                int i = 0;

                for (auto const & p_W : points_world)
                {
                    auto const & p_D = points_display.currentItem();
                    auto & x = p_D.x();
                    auto & y = p_D.y();
                    points_display.next();

                    M.block<1, 4>(3 * i + 0, 0).fill(0);
                    M.block<1, 4>(3 * i + 0, 4) = -1 * p_W.homogeneous();
                    M.block<1, 4>(3 * i + 0, 8) = y * p_W.homogeneous();

                    M.block<1, 4>(3 * i + 1, 0) = 1 * p_W.homogeneous();
                    M.block<1, 4>(3 * i + 1, 4).fill(0);
                    M.block<1, 4>(3 * i + 1, 8) = -x * p_W.homogeneous();

                    M.block<1, 4>(3 * i + 2, 0) = -y * p_W.homogeneous();
                    M.block<1, 4>(3 * i + 2, 4) = x * p_W.homogeneous();
                    M.block<1, 4>(3 * i + 2, 8).fill(0);

                    ++i;
                }

                // Compute the parameter vector c and rearrange c to A.

                JacobiSVD<MatrixXT> svd(M, ComputeThinV);
                VectorXT c = svd.matrixV().col(svd.matrixV().cols() - 1);
                A = Map<Matrix<T, 3, 4, RowMajor>>(c.data());

                break;
            }

            default:
            {
                ErrorObj error;
                error.setClassName("PinholeCameraModel<T>");
                error.setFunctionName("estimate");
                error.setErrorMessage("Unknown estimation method.");
                error.setErrorCode(UNKNOWN_ESTIMATION_METHOD);
                throw error;
            }

        } // switch(method)

        // Decompose A and form T_proj and T_pose from U and R according to (5).

        auto [U, R] = computeRQDecomposition<T>(A.block<3, 3>(0, 0));

        // T_proj
        T_proj.block<3, 3>(0, 0) = U / U(2, 2);
        T_proj(0, 3) = 0;
        T_proj(1, 3) = 0;
        T_proj(2, 3) = 0;

        // T_pose
        MatrixXT Ut = A.block<3, 1>(0, 3);
        MatrixXT t = U.inverse() * Ut;
        T_pose.block<3, 3>(0, 0) = R;
        T_pose.block<3, 1>(0, 3) = t;

        // Compute the root-mean square error.

        T sum_of_squared_error = 0;

        points_world.first();
        points_display.first();

        while (!points_world.isDone())
        {
            auto const & p_W = points_world.currentItem();
            auto const & p_D = points_display.currentItem();

            sum_of_squared_error += (this->transform(p_W) - p_D).squaredNorm();

            points_world.next();
            points_display.next();
        }

        using std::sqrt;
        return ::sqrt(sum_of_squared_error / number_of_points);
    }


#ifdef CPPOPTLIB_FOUND

    /**
     * \param [in] points_world                 3d points in the world coordinate system.
     * \param [in] points_display               Corresponding 2d points in the image plane.
     * \param [in] constraints                  Constraints with respect to the projection matrix.
     * \param [in] initialize                   See below for more details.
     * \param [in] constrained_decomposition    See below for more details.
     *
     * This function estimates the intrinsic and extrinsic camera parameters from two
     * corresponding point sets (i.e., the i-th elements of both point sets correspond
     * to each other). The estimation is constrained in such a way that the focal lengths
     * \f$ f_x \f$ and \f$ f_y \f$ are equal and/or the skew parameter \f$ \tau \f$ is
     * zero.
     *
     * It is assumed that the camera is fixed in the world coordinate system (COS) and
     * that the transition from the world COS into the camera COS is described by the pose
     * matrix. The 2d points \f$ (x'_i, y'_i)^T \f$ in \p points_display are assumed to
     * be normalized:
     *
     * \f[
     *     \begin{pmatrix}
     *         x'_i \\
     *         y'_i
     *     \end{pmatrix}
     *     =
     *     \begin{pmatrix}
     *         u_i / w_i \\
     *         v_i / w_i
     *     \end{pmatrix},
     *
     *     \quad
     *
     *     \begin{pmatrix}
     *         u_i \\
     *         v_i \\
     *         w_i
     *     \end{pmatrix}
     *     =
     *     T_{proj} T_{pose}
     *     \begin{pmatrix}
     *         x_i \\
     *         y_i \\
     *         z_i \\
     *         1
     *     \end{pmatrix}
     * \f]
     *
     * At least 6 corresponding point pairs must be given. The focal lengths are
     * assumed to be positive.
     *
     * If \c initialize is set to true, the camera parameters are estimated by calling
     * estimate(). Thereby, the internal parameters are overwritten. If the flag is set
     * to false, the currently set camera parameters are used during the optimization.
     *
     * The camera parameters are decomposed into the intrinsic and extrinsic camera
     * parameters using the RQ decomposition. Then, these two matrices are modified
     * during the iterative optimization procedure in order to reduce the root mean
     * square error. The user-specified constraints are taken into account during the
     * optimization but not when decomposing the camera parameters. By setting the
     * \c constrained_decomposition flag to true, the constraints are also taken into
     * account when performing the decomposition. Depending on the nature of the data
     * the one or other may be more advantageous.
     *
     * \returns
     *
     * If \f$ f \f$ describes the above mapping, i.e.,
     * \f$ p' = f(p) \f$ with \f$ p' = (x', y')^T \f$ and \f$ p = (x, y, z)^T \f$,
     * then this function returns the root mean square error
     *
     * \f[
     *     rmse = \sqrt{ \frac{1}{N} \sum_{i=1}^N \left\Vert \hat f(p_i) - p'_i \right\Vert^2 }
     * \f]
     *
     * where \f$ \hat f \f$ is the function estimate.
     *
     * \throw ErrorObj  If the input set sizes do not match, an error object is thrown and
     *                  its error code is set to \c UNEQUAL_CARDINALITY_OF_INPUT_SETS.
     *
     * \throw ErrorObj  If there are not enough point pairs (at least six), an error
     *                  object is thrown and its error code is set to \c NOT_ENOUGH_POINTS.
     *
     * \par Important note
     *
     * This function is only available if the \macro{CPPOPTLIB_FOUND} macro is defined. Then,
     * the include directory of the CppOptimizationLibrary must be set appropriately. In CMake
     * you might want to insert the following lines in your \c CMakeLists.txt file.
     *
     * \verbatim
       find_path(CPPOPTLIB NAMES cppoptlib cppoptlib-1.0.0)
       if(CPPOPTLIB)
           set(CPPOPTLIB_FOUND 1)
           include_directories(${CPPOPTLIB})
           add_definitions(-DCPPOPTLIB_FOUND)
       endif()
       \endverbatim
     *
     * The library can be found at https://github.com/PatWie/CppNumericalSolvers .
     *
     */

    template <class T>
    T PinholeCameraModel<T>::estimateWithConstraints(const Range<Vector3T> & points_world,
                                                     const Range<Vector2T> & points_display,
                                                     Constraints constraints,
                                                     bool initialize,
                                                     bool constrained_decomposition)
    {
        using namespace std;
        using namespace Eigen;
        using namespace TRTK::Tools;

        T rmse = std::numeric_limits<T>::max();

        if (initialize)
        {
            // Estimate the camera parameters.
            T rmse = estimate(points_world, points_display);
        }

        // Store the input point pairs in two matrices where each column represents one point.
        // This makes the computation of the objective function much easier and faster (see below).

        int n = (int) points_world.size();
        MatrixXT P_world(4, n);
        MatrixXT P_display(2, n);
        auto iter_world = points_world.begin();
        auto iter_display = points_display.begin();
        for (int i = 0; i < n; ++i)
        {
            P_world.col(i) = (*(iter_world++)).homogeneous();
            P_display.col(i) = *(iter_display++);
        }

        switch (constraints)
        {
            case NO_SKEW:
            {
                // First decompose the camera matrix into the projection matrix and the pose matrix, respectively.

                VectorXT optimization_parameters;
                optimization_parameters.resize(10);

                if (constrained_decomposition)
                {
                    /*

                    Decompose T_camera into T_proj and T_pose. While doing so, assume that tau = 0. Since, this
                    assumption does no hold (due to noise, etc.), the below equation only provide some
                    rough estimates which are then used as the initial values for the non-linear optimization.


                    | T11  T12  T13  T14 |     | fx  tau  cx  0 |     | R  t |
                    | T21  T22  T23  T24 |  =  | 0   fy   cy  0 |  *  | 0  1 |
                    | T31  T32  T33  T34 |     | 0   0    1   0 |

                         | T11  T12  T13 |     | T1^T |     | fx  0   cx  |     | R1^T |
                    ==>  | T21  T22  T23 |  =  | T2^T |  =  | 0   fy  cy  |  *  | R2^T |          (*)
                         | T31  T32  T33 |     | T3^T |     | 0   0   1   |     | R3^T |

                         fx * R1 + cx * R3 = T1
                    ==>  fy * R2 + cy * R3 = T2
                         R3 = T3

                    Normalize T with respect to the last row: T = T / || T3 ||.

                    With some easy algebraic manipulations we have (R1, R2, and R3 are orthogonal to each other)

                    cx = T3^T * T1
                    cy = T3^T * T2

                    fx * R1 = T1 - cx * T3
                    ==>  fx = || T1 - cx * T3 ||
                    ==> R1 = (T1 - cx * T3) / fx

                    fy * R2 = T2 - cy * T3
                    ==>  fy = || T2 - cy * T3 ||
                    ==> R2 = (T2 - cy * T3) / fy

                    R = [R1, R2, R3]^T is not necessarily orthogonal, thus a non-linear optimization has to follow.

                    */

                    auto T_camera = getCameraParameters();

                    T_camera /= T_camera.block<1, 3>(2, 0).norm(); // the last row vector shall be a unit vector
                    Vector3T T1 = T_camera.block<1, 3>(0, 0);
                    Vector3T T2 = T_camera.block<1, 3>(1, 0);
                    Vector3T T3 = T_camera.block<1, 3>(2, 0);
                    Vector3T R3 = T3;
                    T cx = T3.transpose() * T1;
                    T cy = T3.transpose() * T2;

                    T fx = (T1 - cx * T3).norm();
                    Vector3T R1 = (T1 - cx * T3) / fx;

                    T fy = (T2 - cy * T3).norm();
                    Vector3T R2 = (T2 - cy * T3) / fy;

                    Matrix3T R;
                    R.block<1, 3>(0, 0) = R1;
                    R.block<1, 3>(1, 0) = R2;
                    R.block<1, 3>(2, 0) = R3;

                    // Orthogonalize R (guarantee right-handedness).

                    auto svd = JacobiSVD<Matrix3T>(R, ComputeFullU | ComputeFullV);
                    R = svd.matrixU() * svd.matrixV().transpose();
                    if (R.determinant() < 0) // left-handed?
                    {
                        MatrixXT D = MatrixXT::Identity(3, 3);
                        D(2, 2) = -1;
                        R = svd.matrixU() * D * svd.matrixV().transpose();
                    }

                    // Now, do the non-linear optimization. The RMSE is computed over the function (*)
                    // where the rotation matrix is constructed from an axis-angle representation; the
                    // unit axis is represented in spherical coordinates with only two angles/parameters.
                    // Constructing a rotation matrix has the advantage that the orthogonality constraint
                    // is always fulfilled.

                    auto [axis, rotation_angle] = axisAngleFromRotationMatrix(R);
                    auto [__, axis_theta, axis_phi] = cartesian2Spherical(axis.x(), axis.y(), axis.z());

                    optimization_parameters(0) = fx;
                    optimization_parameters(1) = fy;
                    optimization_parameters(2) = cx;
                    optimization_parameters(3) = cy;
                    optimization_parameters(4) = axis_theta;
                    optimization_parameters(5) = axis_phi;
                    optimization_parameters(6) = rotation_angle;
                    optimization_parameters(7) = this->getCameraPosition().x();
                    optimization_parameters(8) = this->getCameraPosition().y();
                    optimization_parameters(9) = this->getCameraPosition().z();
                }
                else
                {
                    Matrix3T R = T_pose.block<3, 3>(0, 0);
                    auto [axis, rotation_angle] = axisAngleFromRotationMatrix(R);
                    auto [__, axis_theta, axis_phi] = cartesian2Spherical(axis.x(), axis.y(), axis.z());

                    optimization_parameters(0) = T_proj(0, 0);  // fx
                    optimization_parameters(1) = T_proj(1, 1);  // fy
                    optimization_parameters(2) = T_proj(0, 2);  // cx
                    optimization_parameters(3) = T_proj(1, 2);  // cy
                    optimization_parameters(4) = axis_theta;
                    optimization_parameters(5) = axis_phi;
                    optimization_parameters(6) = rotation_angle;
                    optimization_parameters(7) = T_pose(0, 3);  // tx
                    optimization_parameters(8) = T_pose(1, 3);  // ty
                    optimization_parameters(9) = T_pose(2, 3);  // tz
                }

                // Now compute the constrained optimization.

                class ObjectiveFunction : public cppoptlib::Problem<double>
                {
                public:
                    using typename cppoptlib::Problem<double>::Scalar;
                    using typename cppoptlib::Problem<double>::TVector;

                    ObjectiveFunction(const MatrixXT & P_world, const MatrixXT & P_display)
                    {
                        this->P_world = P_world;
                        this->P_display = P_display;
                    }

                    double value(const TVector & x)
                    {
                        T_proj = Matrix34T::Identity();
                        T_proj(0, 0) = x(0);
                        T_proj(1, 1) = x(1);
                        T_proj(0, 2) = x(2);
                        T_proj(1, 2) = x(3);

                        T_pose = Matrix4T::Identity();
                        auto axis = spherical2Cartesian(Vector3T(1, x(4), x(5)));
                        T_pose.block<3, 3>(0, 0) = rotationMatrix(axis, x(6));
                        T_pose(0, 3) = x(7);
                        T_pose(1, 3) = x(8);
                        T_pose(2, 3) = x(9);

                        using ::sqrt;
                        MatrixXT T_camera = T_proj * T_pose;
                        MatrixXT P_projected = (T_camera * P_world).colwise().hnormalized();
                        T rmse = sqrt((P_display - P_projected).colwise().squaredNorm().mean());
                        return rmse;
                    }

                public:
                    MatrixXT P_world;
                    MatrixXT P_display;
                    Matrix34T T_proj;
                    Matrix4T T_pose;
                };

                auto func = ObjectiveFunction(P_world, P_display);
                cppoptlib::BfgsSolver<ObjectiveFunction> solver;
                solver.minimize(func, optimization_parameters);
                T_proj = func.T_proj;
                T_pose = func.T_pose;
                rmse = func.value(optimization_parameters);

                break;
            }

            case SAME_FOCAL_LENGTHS:
            {
                // First decompose the camera matrix into the projection matrix and the pose matrix, respectively.

                VectorXT optimization_parameters;
                optimization_parameters.resize(10);

                if (constrained_decomposition)
                {
                    /*

                    Decompose T_camera into T_proj and T_pose. While doing so, assume that f := f_x = f_y.
                    Since, this assumption does no hold (due to noise, etc.), the below equations only provide
                    some rough estimates which are then used as the initial values for the non-linear optimization.


                    | T11  T12  T13  T14 |     | fx  tau  cx  0 |     | R  t |
                    | T21  T22  T23  T24 |  =  | 0   fy   cy  0 |  *  | 0  1 |
                    | T31  T32  T33  T34 |     | 0   0    1   0 |

                         | T11  T12  T13 |     | T1^T |     | f  tau  cx  |     | R1^T |
                    ==>  | T21  T22  T23 |  =  | T2^T |  =  | 0  f    cy  |  *  | R2^T |          (***)
                         | T31  T32  T33 |     | T3^T |     | 0  0    1   |     | R3^T |

                         f * R1 + tau * R2 + cx * R3 = T1
                    ==>  f * R2 + cy * R3 = T2
                         R3 = T3

                    Normalize T with respect to the last row: T = T / || T3 ||.

                    With some easy algebraic manipulations we have (R1, R2, and R3 are orthogonal to each other)

                    cx = T3^T * T1
                    cy = T3^T * T2

                    f * R2 = T2 - cy * T3
                    ==>  f = || T2 - cy * T3 ||
                    ==> R2 = (T2 - cy * T3) / f

                    tau * R2 = T1 - f * R1 - cx * T3
                    ==> tau = R2^T * (T1 - f * R1 - cx * T3) = R2^T * T1
                    ==> R1 = T1 - tau * R2 - cx * T3    (also normalize R1..)

                    R = [R1, R2, R3]^T is not necessarily orthogonal, thus a non-linear optimization is still necessary.

                    */

                    auto T_camera = getCameraParameters();

                    T_camera /= T_camera.block<1, 3>(2, 0).norm(); // the last row vector shall be a unit vector
                    Vector3T T1 = T_camera.block<1, 3>(0, 0);
                    Vector3T T2 = T_camera.block<1, 3>(1, 0);
                    Vector3T T3 = T_camera.block<1, 3>(2, 0);
                    Vector3T R3 = T3;
                    T cx = T3.transpose() * T1;
                    T cy = T3.transpose() * T2;

                    T f = (T2 - cy * T3).norm();
                    Vector3T R2 = (T2 - cy * T3) / f;

                    T tau = R2.transpose() * T1;
                    Vector3T R1 = (T1 - tau * R2 - cx * T3).normalized();

                    Matrix3T R;
                    R.block<1, 3>(0, 0) = R1;
                    R.block<1, 3>(1, 0) = R2;
                    R.block<1, 3>(2, 0) = R3;

                    // Orthogonalize R (guarantee right-handedness).

                    auto svd = JacobiSVD<Matrix3T>(R, ComputeFullU | ComputeFullV);
                    R = svd.matrixU() * svd.matrixV().transpose();
                    if (R.determinant() < 0) // left-handed?
                    {
                        MatrixXT D = MatrixXT::Identity(3, 3);
                        D(2, 2) = -1;
                        R = svd.matrixU() * D * svd.matrixV().transpose();
                    }

                    // Now, do the non-linear optimization. The RMSE is computed over the function (***)
                    // where the rotation matrix is constructed from an axis-angle representation; the
                    // unit axis is represented in spherical coordinates with only two angles/parameters.
                    // Constructing a rotation matrix has the advantage that the orthogonality constraint
                    // is always fulfilled.

                    auto [axis, rotation_angle] = axisAngleFromRotationMatrix(R);
                    auto [__, axis_theta, axis_phi] = cartesian2Spherical(axis.x(), axis.y(), axis.z());

                    optimization_parameters(0) = f;
                    optimization_parameters(1) = tau;
                    optimization_parameters(2) = cx;
                    optimization_parameters(3) = cy;
                    optimization_parameters(4) = axis_theta;
                    optimization_parameters(5) = axis_phi;
                    optimization_parameters(6) = rotation_angle;
                    optimization_parameters(7) = this->getCameraPosition().x();
                    optimization_parameters(8) = this->getCameraPosition().y();
                    optimization_parameters(9) = this->getCameraPosition().z();
                }
                else
                {
                    Matrix3T R = T_pose.block<3, 3>(0, 0);
                    auto [axis, rotation_angle] = axisAngleFromRotationMatrix(R);
                    auto [__, axis_theta, axis_phi] = cartesian2Spherical(axis.x(), axis.y(), axis.z());

                    optimization_parameters(0) = 0.5 * (T_proj(0, 0) + T_proj(1, 1));  // f = (fx + fy) / 2
                    optimization_parameters(1) = T_proj(0, 1);  // tau
                    optimization_parameters(2) = T_proj(0, 2);  // cx
                    optimization_parameters(3) = T_proj(1, 2);  // cy
                    optimization_parameters(4) = axis_theta;
                    optimization_parameters(5) = axis_phi;
                    optimization_parameters(6) = rotation_angle;
                    optimization_parameters(7) = T_pose(0, 3);  // tx
                    optimization_parameters(8) = T_pose(1, 3);  // ty
                    optimization_parameters(9) = T_pose(2, 3);  // tz
                }

                // Now compute the constrained optimization.

                class ObjectiveFunction : public cppoptlib::Problem<double>
                {
                public:
                    using typename cppoptlib::Problem<double>::Scalar;
                    using typename cppoptlib::Problem<double>::TVector;

                    ObjectiveFunction(const MatrixXT & P_world, const MatrixXT & P_display)
                    {
                        this->P_world = P_world;
                        this->P_display = P_display;
                    }

                    double value(const TVector & x)
                    {
                        T_proj = Matrix34T::Identity();
                        T_proj(0, 0) = x(0);
                        T_proj(0, 1) = x(1);
                        T_proj(0, 2) = x(2);
                        T_proj(1, 1) = x(0);
                        T_proj(1, 2) = x(3);

                        T_pose = Matrix4T::Identity();
                        auto axis = spherical2Cartesian(Vector3T(1, x(4), x(5)));
                        T_pose.block<3, 3>(0, 0) = rotationMatrix(axis, x(6));
                        T_pose(0, 3) = x(7);
                        T_pose(1, 3) = x(8);
                        T_pose(2, 3) = x(9);

                        using ::sqrt;
                        MatrixXT T_camera = T_proj * T_pose;
                        MatrixXT P_projected = (T_camera * P_world).colwise().hnormalized();
                        T rmse = sqrt((P_display - P_projected).colwise().squaredNorm().mean());
                        return rmse;
                    }

                public:
                    MatrixXT P_world;
                    MatrixXT P_display;
                    Matrix34T T_proj;
                    Matrix4T T_pose;
                };

                auto func = ObjectiveFunction(P_world, P_display);
                cppoptlib::BfgsSolver<ObjectiveFunction> solver;
                solver.minimize(func, optimization_parameters);
                T_proj = func.T_proj;
                T_pose = func.T_pose;
                rmse = func.value(optimization_parameters);

                break;
            }

            case SAME_FOCAL_LENGTHS_AND_NO_SKEW:
            {
                // First decompose the camera matrix into the projection matrix and the pose matrix, respectively.

                VectorXT optimization_parameters;
                optimization_parameters.resize(9);

                if (constrained_decomposition)
                {
                    /*

                    Decompose T_camera into T_proj and T_pose. While doing so, assume that f := f_x = f_y and tau = 0.
                    Since, these assumptions do no hold (due to noise, etc.), the below equation only provide some
                    rough estimates which are then used as the initial values for the non-linear optimization.


                    | T11  T12  T13  T14 |     | fx  tau  cx  0 |     | R  t |
                    | T21  T22  T23  T24 |  =  | 0   fy   cy  0 |  *  | 0  1 |
                    | T31  T32  T33  T34 |     | 0   0    1   0 |

                         | T11  T12  T13 |     | T1^T |     | f  0  cx  |     | R1^T |
                    ==>  | T21  T22  T23 |  =  | T2^T |  =  | 0  f  cy  |  *  | R2^T |          (***)
                         | T31  T32  T33 |     | T3^T |     | 0  0  1   |     | R3^T |

                         f * R1 + cx * R3 = T1
                    ==>  f * R2 + cy * R3 = T2
                         R3 = T3

                    Normalize T with respect to the last row: T = T / || T3 ||.

                    With some easy algebraic manipulations we have (R1, R2, and R3 are orthogonal to each other)

                    cx = T3^T * T1
                    cy = T3^T * T2

                    f * R1 = T1 - cx * T3
                    ==>  f = || T1 - cx * T3 ||
                    ==> R1 = (T1 - cx * T3) / f

                    f * R2 = T2 - cy * T3
                    ==>  f = || T2 - cy * T3 ||
                    ==> R2 = (T2 - cy * T3) / f

                    Both former computations for determining f may conflict with each other. Thus, we just
                    average both values of f. Also, R = [R1, R2, R3]^T is not necessarily orthogonal.

                    */

                    auto T_camera = getCameraParameters();

                    T_camera /= T_camera.block<1, 3>(2, 0).norm(); // the last row vector shall be a unit vector
                    Vector3T T1 = T_camera.block<1, 3>(0, 0);
                    Vector3T T2 = T_camera.block<1, 3>(1, 0);
                    Vector3T T3 = T_camera.block<1, 3>(2, 0);
                    Vector3T R3 = T3;
                    T cx = T3.transpose() * T1;
                    T cy = T3.transpose() * T2;

                    T fx = (T1 - cx * T3).norm();
                    Vector3T R1 = (T1 - cx * T3) / fx;

                    T fy = (T2 - cy * T3).norm();
                    Vector3T R2 = (T2 - cy * T3) / fy;

                    T f = (fx + fy) / 2;

                    Matrix3T R;
                    R.block<1, 3>(0, 0) = R1;
                    R.block<1, 3>(1, 0) = R2;
                    R.block<1, 3>(2, 0) = R3;

                    // Orthogonalize R (guarantee right-handedness).

                    auto svd = JacobiSVD<Matrix3T>(R, ComputeFullU | ComputeFullV);
                    R = svd.matrixU() * svd.matrixV().transpose();
                    if (R.determinant() < 0) // left-handed?
                    {
                        MatrixXT D = MatrixXT::Identity(3, 3);
                        D(2, 2) = -1;
                        R = svd.matrixU() * D * svd.matrixV().transpose();
                    }

                    // Now, do the non-linear optimization. The RMSE is computed over the function (***)
                    // where the rotation matrix is constructed from an axis-angle representation; the
                    // unit axis is represented in spherical coordinates with only two angles/parameters.
                    // Constructing a rotation matrix has the advantage that the orthogonality constraint
                    // is always fulfilled.

                    auto [axis, rotation_angle] = axisAngleFromRotationMatrix(R);
                    auto [__, axis_theta, axis_phi] = cartesian2Spherical(axis.x(), axis.y(), axis.z());

                    optimization_parameters(0) = f;
                    optimization_parameters(1) = cx;
                    optimization_parameters(2) = cy;
                    optimization_parameters(3) = axis_theta;
                    optimization_parameters(4) = axis_phi;
                    optimization_parameters(5) = rotation_angle;
                    optimization_parameters(6) = this->getCameraPosition().x();
                    optimization_parameters(7) = this->getCameraPosition().y();
                    optimization_parameters(8) = this->getCameraPosition().z();
                }
                else
                {
                    Matrix3T R = T_pose.block<3, 3>(0, 0);
                    auto [axis, rotation_angle] = axisAngleFromRotationMatrix(R);
                    auto [__, axis_theta, axis_phi] = cartesian2Spherical(axis.x(), axis.y(), axis.z());

                    optimization_parameters(0) = 0.5 * (T_proj(0, 0) + T_proj(1, 1));  // f = (fx + fy) / 2
                    optimization_parameters(1) = T_proj(0, 2);  // cx
                    optimization_parameters(2) = T_proj(1, 2);  // cy
                    optimization_parameters(3) = axis_theta;
                    optimization_parameters(4) = axis_phi;
                    optimization_parameters(5) = rotation_angle;
                    optimization_parameters(6) = T_pose(0, 3);  // tx
                    optimization_parameters(7) = T_pose(1, 3);  // ty
                    optimization_parameters(8) = T_pose(2, 3);  // tz
                }

                // Now compute the constrained optimization.

                class ObjectiveFunction : public cppoptlib::Problem<double>
                {
                public:
                    using typename cppoptlib::Problem<double>::Scalar;
                    using typename cppoptlib::Problem<double>::TVector;

                    ObjectiveFunction(const MatrixXT & P_world, const MatrixXT & P_display)
                    {
                        this->P_world = P_world;
                        this->P_display = P_display;
                    }

                    double value(const TVector & x)
                    {
                        T_proj = Matrix34T::Identity();
                        T_proj(0, 0) = x(0);
                        T_proj(1, 1) = x(0);
                        T_proj(0, 2) = x(1);
                        T_proj(1, 2) = x(2);

                        T_pose = Matrix4T::Identity();
                        auto axis = spherical2Cartesian(Vector3T(1, x(3), x(4)));
                        T_pose.block<3, 3>(0, 0) = rotationMatrix(axis, x(5));
                        T_pose(0, 3) = x(6);
                        T_pose(1, 3) = x(7);
                        T_pose(2, 3) = x(8);

                        using ::sqrt;
                        MatrixXT T_camera = T_proj * T_pose;
                        MatrixXT P_projected = (T_camera * P_world).colwise().hnormalized();
                        T rmse = sqrt((P_display - P_projected).colwise().squaredNorm().mean());
                        return rmse;
                    }

                public:
                    MatrixXT P_world;
                    MatrixXT P_display;
                    Matrix34T T_proj;
                    Matrix4T T_pose;
                };

                auto func = ObjectiveFunction(P_world, P_display);
                cppoptlib::BfgsSolver<ObjectiveFunction> solver;
                solver.minimize(func, optimization_parameters);
                T_proj = func.T_proj;
                T_pose = func.T_pose;
                rmse = func.value(optimization_parameters);

                break;
            }

            default:
            {
                ErrorObj error;
                error.setClassName("PinholeCameraModel<T>");
                error.setFunctionName("estimate");
                error.setErrorMessage("Unknown estimation method.");
                error.setErrorCode(UNKNOWN_ESTIMATION_METHOD);
                throw error;
            }

        } // switch(method)


        return rmse;
    }

#endif // CPPOPTLIB_FOUND


    template <class T>
    typename PinholeCameraModel<T>::Matrix34T PinholeCameraModel<T>::getCameraParameters() const
    {
        return T_proj * T_pose;
    }


    template <class T>
    typename PinholeCameraModel<T>::Matrix3T PinholeCameraModel<T>::getCameraOrientation() const
    {
        return T_pose.block<3, 3>(0, 0);
    }


    template <class T>
    typename PinholeCameraModel<T>::Vector3T PinholeCameraModel<T>::getCameraPosition() const
    {
        return T_pose.block<3, 1>(0, 3);
    }


    template <class T>
    typename PinholeCameraModel<T>::Matrix4T PinholeCameraModel<T>::getExtrinsicParameters() const
    {
        return T_pose;
    }


    template <class T>
    std::tuple<T, T> PinholeCameraModel<T>::getFocalLengths() const
    {
        return std::make_tuple(T_proj(0, 0), T_proj(1, 1));
    }


    template <class T>
    std::tuple<T, T> PinholeCameraModel<T>::getImageCenter() const
    {
        return std::make_tuple(T_proj(0, 2), T_proj(1, 2));
    }


    template <class T>
    typename PinholeCameraModel<T>::Matrix34T PinholeCameraModel<T>::getIntrinsicParameters() const
    {
        return T_proj;
    }


    template <class T>
    T PinholeCameraModel<T>::getSkew() const
    {
        return T_proj(0, 1);
    }


    template <class T>
    typename PinholeCameraModel<T>::Vector2T PinholeCameraModel<T>::operator*(const Vector3T & point) const
    {
        return transform(point);
    }


    template <class T>
    typename PinholeCameraModel<T>::Vector2T PinholeCameraModel<T>::transform(const Vector3T & point) const
    {
        return (T_proj * T_pose * point.homogeneous()).hnormalized();
    }


    template <class T>
    void PinholeCameraModel<T>::setCameraOrientation(const Matrix3T & orientation)
    {
        assert((orientation * orientation.transpose() - Matrix3T::Identity()).norm() < 1e-7); // orthogonal?
        // assert(abs(orientation.determinant() - 1) < 1e-7); // right-handed without scaling? <-- nope, we should not unnecessarily restrict the mapping

        T_pose.block<3, 3>(0, 0) = orientation;
    }


    template <class T>
    void PinholeCameraModel<T>::setCameraPosition(const Vector3T & position)
    {
        T_pose.block<3, 1>(0, 3) = position;
    }


    template <class T>
    void PinholeCameraModel<T>::setExtrinsicParameters(const Matrix4T & parameters)
    {
        auto & R = parameters.block<3, 3>(0, 0);
        const bool ORIENTATION_MATRIX_IS_ORTHOGONAL = ((R * R.transpose()) - Matrix3T::Identity()).norm() < 1e-7;
        assert(ORIENTATION_MATRIX_IS_ORTHOGONAL);

        T_pose = parameters;
    }


    template <class T>
    void PinholeCameraModel<T>::setFocalLengths(T f_x, T f_y)
    {
        T_proj(0, 0) = f_x;
        T_proj(1, 1) = f_y;
    }


    template <class T>
    void PinholeCameraModel<T>::setImageCenter(T c_x, T c_y)
    {
        T_proj(0, 2) = c_x;
        T_proj(1, 2) = c_y;
    }


    template <class T>
    void PinholeCameraModel<T>::setIntrinsicParameters(const Matrix34T & parameters)
    {
        T_proj = parameters;
    }


    template <class T>
    void PinholeCameraModel<T>::setSkew(T skew)
    {
        T_proj(0, 1) = skew;
    }


} // namespace TRTK


#endif // PINHOLE_CAMERA_MODEL_HPP_4231789408
