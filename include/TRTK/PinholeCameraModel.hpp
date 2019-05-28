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


#include <iostream> // TODO: DELETE ME
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>

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
     * The camera is modeled by a projection matrix \f$ T_{proj} \f$ which maps
     * a three-dimensional point \f$ p \f$ to the two-dimensional point
     * \f$ p' = T_{proj} * p \f$ that lies in the image plane of the camera. The
     * optical properties of the camera are defined by the focal lengths \f$ f_x \f$
     * and \f$ f_y \f$, the image center \f$ (c_x, c_y) \f$ where the optical axis
     * intersects the image plane, and a parameter \f$ \tau \f$ which models the
     * skew of the image plane axes. The camera's pose (i.e., its location and
     * orientation) in world coordinates is described by the mapping \f$ T_{pose} \f$
     * which maps from the world coordinate system to the camera coordinate system.
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
     * For more details, please be referred to [1], [2], and [3].
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
     * \references
     *
     * [1] Hartley and Zisserman, "Multiple view geometry in computer vision", 2003
     *
     * [2] Szeliski, "Computer Vision - Algorithms and Applications", 2011, Springer
     *
     * [3] Tuceryan et al., "Single point active alignment method (SPAAM) for optical
     *     see-through HMD calibration for AR", 2000, IEEE
     *
     * \see TRTK::Transform3D
     *
     * \author Christoph Hänisch
     * \version 1.0.0
     * \date last changed on 2019-05-28
     */

    template <class T>
    class PinholeCameraModel
    {
    public:
        enum Error {
            NOT_ENOUGH_INPUT_DATA,
            UNEQUAL_CARDINALITY_OF_INPUT_SETS,
            UNKNOWN_ERROR
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

        double estimate(const Range<Vector3T> & points_world, const Range<Vector2T> & points_display);   ///< Estimates the intrinsic and extrinsic camera parameters from two corresponding point sets.

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
     * correpond to each other). It is assumed that the camera is fixed in
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
    double PinholeCameraModel<T>::estimate(const Range<Vector3T> & points_world, const Range<Vector2T> & points_display)
    {
        /*

        This implementation is loosely based on the work of Tuceryan et al. [1]. The
        QR decomposition is not mentioned in the paper. Note, the parameter names
        and variable names may slightly vary from this article. The algorithm is
        essentially the same as the direct linear transformation algorithm
        as described in [2] or [3].


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
        B = U * S * V^T; see also [1].

        Finally, the matrix A must be decomposed into the projection matrix T_proj and
        the pose matrix T_pose. Using block matrices equation (1) can also be written as

            | U  0 |  *  | R  t |  =  | UR  Ut |  =  A      (5)
                         | 0  1 |

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


        References
        ==========

        [1] Tuceryan et al., "Single point active alignment method (SPAAM) for
            optical see-through HMD calibration for AR", 2000

        [2] Hartley and Zisserman, "Multiple view geometry in computer vision", 2003

        [3] Sutherland, "Three-dimensional data input by tablet", 1974

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
        Map<Matrix<T, 3, 4, RowMajor>> A(c.data());

        // Decompose A and form T_proj and T_pose from U and R according to (5).

        MatrixXT U, R;
        tie(U, R) = computeRQDecomposition<T>(A.block<3, 3>(0, 0));

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

            sum_of_squared_error += (this->transform(p_W) - p_D).norm();

            points_world.next();
            points_display.next();
        }

        using std::sqrt;
        return ::sqrt(sum_of_squared_error / number_of_points);
    }


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
