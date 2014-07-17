/*
    Estimation and computation of a multivariate multidimensional polynomial.

    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.1.0 (2013-06-06)
*/

/** \file GenericPolynomial.hpp
  * \brief This file contains the \ref TRTK::GenericPolynomial "GenericPolynomial" class.
  */


#ifndef GENERIC_POLYNOMIAL_HPP_7477314703
#define GENERIC_POLYNOMIAL_HPP_7477314703


#include <cassert>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Coordinate.hpp"
#include "ErrorObj.hpp"
#include "Polynomial.hpp"
#include "Range.hpp"


namespace TRTK
{


//////////////////////////////////////////////////////////////////////////////////////////////
//                                     Helper Functions                                     //
//////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
class GenericPolynomial;


/** \brief Returns the binomial coefficient "n choose k". */

long long binomial(int n, int k)
{
    if (k < 0 || k > n) return 0;

    if (k > n - k)
    {
        k = n - k; // take advantage of symmetry
    }

    long long result = 1;

    for (int i = 1; i <= k; ++i)
    {
        result *= n - (k - i);
        result /= i;
    }

    return result;
}


/** \relates GenericPolynomial
  *
  * \internal
  *
  * \brief Returns the number of monomials of a polynomial of degree r in n variables.
  */

unsigned numberMonomials(unsigned n, unsigned r)
{
    // Monomials of constant degree (i.e. r = 3) form a multiset, that is a set
    // where members are allowed to appear more than once (example: x^2 y = [x x y]).
    // In contrast to tuples, the order of elements is irrelevant. The number of
    // monomials of degree r in n variables is the number of multicombinations of r
    // elements chosen among the n variables: binomial(n + r - 1, r). The total
    // number of monomials of a polynomial of degree r in n variables is just the sum
    // of the numbers of monomials of constant degree.

    unsigned result = 1;

    for (unsigned i = 1; i <= r; ++i)
    {
        result += (int) binomial(n + i - 1, i);
    }

    return result;
}


/** \relates GenericPolynomial
  *
  * \internal
  *
  * \brief Computes all possible monomials of a multivariate polynomial of degree r
  *        in n variables (i.e. a polynomial which maps from R^n to R).
  *
  * \param [in] n                   Number of input variables (dimension of the domain).
  * \param [in] r                   Degree of the polynomial.
  * \param [in] number_points       Number of source points.
  * \param [in] it_source_points    Degree of the polynomial.
  * \param [in,out] monomials       Empty STL vector used to store the monomials.
  */

template <class T>
void computeMonomials(unsigned n,
                      unsigned r,
                      unsigned number_points,
                      Iterator<typename GenericPolynomial<T>::Point> it_source_points,
                      std::vector<Eigen::Matrix<T, Eigen::Dynamic, 1> > & monomials)
{
    /*
    How does the algorithm work?

    The aim of the algorithm is to avoid unnecessary multiplications, i.e. if we want
    to compute x^3 and already computed x^2 we can reuse the former result. Consequently,
    computing the monomials we start with the lowest degree.

    Since the monomials xyz and zyx are equivalent, we cannot just compute all permutations
    of products of x, y, and z (which was quite easy). However, a systematic approach for
    computing all monomials of a polynomial of degree r in n variables is as follows:

    Example in n = 3 variables x, y, and z (note: the parentheses form exactly n groups):

    r = 0       1

    r = 1       (x)  (y)  (z)

    r = 2       (xx xy xz)  (yy yz)  (zz)

    r = 3       (xxx xxy xxz xyy xyz xzz)  (yyy yyz yzz)  (zzz)

    r = 4       ...

    Each single line holds all monomials of a certain degree (denoted by r). These are formed
    using the monomials of the line above (that is, from those with a degree of r - 1)· This
    is done by multiplying the first variable x with all n groups. Then by multiplying the
    second variable y by the last n - 1 groups, then by multiplying the third variable z by
    the last n - 2 groups and so on. This is done n times, each time forming a new group.
    This procedure is repeated until the desired degree is reached. Since building up the
    monomials is done recursively, we provide the monomials for the degrees r = 0 and r = 1.

    The computation can be done iteratively using n + 1 pointers, where each pointer shows to
    the start of one of the groups and the last pointer shows behind the end of the last group;
    each pointer corresponds to one of the input variables. To make it more clear, if the
    monomials of degree r are computed the pointers show to the groups of monomials of degree
    r - 1. While computing each new group of monomials, the current pointer is set to the
    beginning of this new group, which can be safely done since computing new subsequent groups
    only involve using old subsequent groups. The beginning of a new group is the container's
    end at the moment of creating a new group.
    */

    assert(n > 0);

    if (number_points == 0) return;


    // Initialization.

    typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXT;

    unsigned number_of_monomials = numberMonomials(n, r);

    monomials = std::vector<VectorXT>(number_of_monomials);

    std::vector<unsigned> begin_of_group(n); // these indeces denote the start of certain groups (or ranges) of monomials of constant degree
    unsigned end; // index denoting the common end of the above ranges


    // Construct the first n + 1 monomials from the user data.

    if (monomials.size() == 0) return; // nothing to do

    // r = 0

    monomials[0] = VectorXT::Ones(number_points);

    // r = 1 (e.g. monomials[1] = x, monomials[2] = y, monomials[3] = z, etc.)

    if (monomials.size() == 1) return; // we are already done

    for (unsigned i = 0; i < n; ++i)
    {
        monomials[i + 1].resize(number_points); // reserve memory
    }

    for (unsigned i = 0; i < number_points; ++i, ++it_source_points)
    {
        typename GenericPolynomial<T>::Point point = *it_source_points;

        for (unsigned j = 0; j < point.size(); ++j)
        {
            monomials[j + 1](i) = point[j];
        }
    }


    // Set up the indeces of the particular ranges.

    for (unsigned i = 0; i < n; ++i)
    {
        begin_of_group[i] = i + 1;
    }

    end = n + 1;


    // Compute the remaining monomials.

    for (unsigned degree = 2; degree <= r; ++degree)
    {
        unsigned end_of_groups = end;

        for (unsigned variable = 0; variable < n; ++variable) // input variables x, y, z, ...
        {
            unsigned index = begin_of_group[variable];

            begin_of_group[variable] = end; // sets the begin of the currently newly generated group

            while (index < end_of_groups)
            {
                // Multiply each element of the range [begin_of_group[variable]; end_of_groups)
                // with the current input variable and store it in the container of monomials.
                // Update the end index.
                //
                // Example: Multiply the range [xx xy xz yy yz zz] with x which
                //          yields [xxx xxy xxz xyy xyz xzz]; in a second iteration
                //          do this with [yy yz zz] and the factor y and so on.

                monomials[end++] = monomials[index++].array() * monomials[variable + 1].array(); // second term is a monomial of first degree (i.e. x, y, or z, ...)
            }
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//                                     GenericPolynomial                                    //
//////////////////////////////////////////////////////////////////////////////////////////////

/** \class GenericPolynomial
  *
  * \brief Multivariate multidimensional polynomial.
  *
  * This class provides means to estimate and compute a multivariate multidimensional
  * polynomial of arbitrary degree.
  *
  * The transformation is computed as follows (here, shown for a polynomial of degree
  * two in three variables):
  *
  * \f[
  * \begin{pmatrix}
  *     x'  \\  y'  \\  z'
  * \end{pmatrix}
  * =
  * f(x, y ,z)
  * =
  * \begin{pmatrix}
  *     a_{1,1} + a_{1,2} x + a_{1,3} y + \cdots + a_{1,8} y^2 + a_{1,9} yz + a_{1,10} z^2  \\
  *     a_{2,1} + a_{2,2} x + a_{2,3} y + \cdots + a_{2,8} y^2 + a_{2,9} yz + a_{2,10} z^2  \\
  *     a_{3,1} + a_{3,2} x + a_{3,3} y + \cdots + a_{3,8} y^2 + a_{3,9} yz + a_{3,10} z^2  \\
  * \end{pmatrix}
  * \qquad
  * f : \mathbb{R}^n \rightarrow \mathbb{R}^m   \quad   n = 3, \, m = 3, \, r = 2
  * \f]
  *
  * \Note
  *
  * The number \f$ N \f$ of estimated coefficients is \f$ N = m \sum_{i=0}^{r}
  * \binom{n + i - 1}{i} \f$ where \f$ n \f$ is the number of input variables,
  * \f$ m \f$ is the dimension of the polynomial, and \f$ r \f$ is the degree
  * of the polynomial. You will need at least as much as \f$ N / m \f$ point
  * pairs to estimate the polynomial, otherwise the internal computation might
  * suffer from a rank deficient matrix.
  *
  * Most functions check for certain assertions (e.g., if the coordinate size is
  * valid) and trigger an assertion failure if the assumption does not hold.
  * This is meant for debugging purposes only and can be disabled by defining
  * the macro \macro{NDEBUG}.
  *
  * \par Examples
  *
  * Here is an example of how to use the GenericPolynomial class:
  *
  * \code
  * #include <iomanip>
  * #include <iostream>
  * #include <vector>
  *
  * #include <TRTK/Tools.hpp>
  * #include <TRTK/GenericPolynomial.hpp>
  *
  * using namespace std;
  * using namespace TRTK;
  * using namespace TRTK::Tools;
  *
  * typedef TRTK::Coordinate<double> Point;
  *
  * Point f(const Point & point)
  * {
  *     double x = point.x();
  *     double y = point.y();
  *     double z = point.z();
  *
  *     return Point(-x, x + 20 * y, z + 5 * x * z);
  * };
  *
  * int main()
  * {
  *     // Generate two point sets that relate to each other via the
  *     // trivariate quadratic polynomial function f.
  *
  *     vector<Point> source_points;
  *     vector<Point> target_points;
  *
  *     for (unsigned i = 0; i < 100; ++i)
  *     {
  *         Point source_point(0, 0, 0);
  *
  *         source_point.x() = rand(-10.0, 10.0);
  *         source_point.y() = rand(-10.0, 10.0);
  *         source_point.z() = rand(-10.0, 10.0);
  *
  *         Point target_point = f(source_point);
  *
  *         // Add noise.
  *
  *         target_point.x() += 0.1 * randn();
  *         target_point.y() += 0.1 * randn();
  *         target_point.z() += 0.1 * randn();
  *
  *         source_points.push_back(source_point);
  *         target_points.push_back(target_point);
  *     }
  *
  *     // Estimate the polynomial function f from the above two sets.
  *
  *     GenericPolynomial<double> polynomial(3, 3, 2);
  *
  *     // double rmse = polynomial.estimate(make_iterator(source_points.begin()),
  *     //                                   make_iterator(source_points.end()),
  *     //                                   make_iterator(target_points.begin()),
  *     //                                   make_iterator(target_points.end()));
  *     //
  *     // or simply:
  *
  *     double rmse = polynomial.estimate(make_range(source_points),
  *                                       make_range(target_points));
  *
  *     // Print out the results.
  *
  *     cout << "Root Mean Square Error: " << rmse << endl;
  *     cout << "Coefficients: " << endl << setprecision(4) << fixed
  *          << polynomial.getCoefficients().transpose() << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  * Root Mean Square Error: 0.162183
  * Coefficients:
  *        0.0409        0.0047        0.0017
  *       -0.9991        0.9957        0.0000
  *       -0.0019       19.9994        0.0009
  *       -0.0006       -0.0000        1.0001
  *       -0.0000        0.0002       -0.0002
  *       -0.0002       -0.0001       -0.0000
  *       -0.0005        0.0001        5.0002
  *       -0.0008       -0.0005        0.0002
  *        0.0001       -0.0001       -0.0004
  *        0.0003        0.0001        0.0002
  * \endcode
  *
  * \par Algorithm
  *
  * This algorithm estimates a linear model
  *
  * \f[
  * y_i = x_i^T \beta + \epsilon_i
  * \f]
  *
  * where the parameter vector \f$ \beta \f$ contains the sought coefficients and
  * where \f$ \epsilon_i \f$ represents the unknown error. The parameter vector is
  * estimated using weighted least squares
  *
  * \f[
  * \begin{align}
  *     \hat\beta &= \arg_{\beta}\min \sum_i w_i (x_i^T \beta - y_i)^2 \\
  *               &= \arg_{\beta}\min || W^{1/2} (A \beta - y) ||^2 \\[0.75em]
  *               &= [A^T W A]^{-1} A^T W y
  * \end{align}
  * \f]
  *
  * If no weights are given \f$ w_i \f$ is assumed to be equal to 1.
  *
  * \Note
  *
  * The algorithm returns the (weighted) root-mean-square error RMSE
  * \f$ = \sqrt{\frac{1}{N} \sum_{i=1}^N w_i (f(x_i) - y_i)^2} \f$ of the
  * estimation.
  *
  * Given constant weights \f$ w_i = w \f$ the RMSE is proportional to the RMSE
  * of an ordinary least squares estimation (\f$ w_i = 1 \f$) by a factor of
  * \f$ w^{-1/2} \f$. It follows, a smaller RMSE does not necessarily imply a better
  * estimation result. You might want to compare results by scaling the RMSE with
  * \f$ \bar{w}^{-1/2} \f$ where \f$ \bar{w} \f$ is the mean of all weights.
  *
  * \see Coordinate, Iterator
  *
  * \author Christoph Haenisch
  * \version 0.1.0
  * \date last changed on 2013-06-06
  */

template <class T>
class GenericPolynomial : public Polynomial<T>
{
private:

    typedef Polynomial<T> super;

protected:

    typedef typename super::VectorXT VectorXT;
    typedef typename super::Vector3T Vector3T;
    typedef typename super::coordinate_type coordinate_type;

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef T value_type;
    typedef typename super::MatrixXT MatrixXT;
    typedef typename super::Point Point;

    using super::DIVISION_BY_ZERO;
    using super::INVALID_ARGUMENT;
    using super::NOT_ENOUGH_POINTS;
    using super::UNEQUAL_NUMBER_OF_POINTS;
    using super::UNKNOWN_ERROR;
    using super::WRONG_COORDINATE_SIZE;

    GenericPolynomial(unsigned n, unsigned m, unsigned r);

    GenericPolynomial(const MatrixXT &, unsigned n, unsigned m, unsigned r);

    GenericPolynomial(const GenericPolynomial &);

    template <class U>
    GenericPolynomial(const GenericPolynomial<U> &, unsigned n, unsigned m, unsigned r);

    virtual ~GenericPolynomial();

    MatrixXT & getCoefficients();
    const MatrixXT & getCoefficients() const;

    T estimate(Range<Point> source_points,
               Range<Point> target_points,
               Range<T> weights = Range<T>());

    T estimate(Iterator<Point> source_points_first,
               Iterator<Point> source_points_last,
               Iterator<Point> target_points_first,
               Iterator<Point> target_points_last,
               Iterator<T> weights_first = Iterator<T>(),
               Iterator<T> weights_last = Iterator<T>());

    Point operator*(const Point &) const;
    Point transform(const Point &) const;

    Polynomial<T> & reset();

    void setDegree(unsigned degree);

private:

    MatrixXT m_matrix;

    unsigned number_input_variables;
    unsigned dimension; // of the output vector
    unsigned degree;

    friend void computeMonomials<T>(unsigned, unsigned, unsigned, Iterator<Point>, std::vector<VectorXT> &);

};


/** \tparam T scalar type
  *
  * \brief Constructs an instance of GenericPolynomial.
  *
  * \param [in] n    Number of input variables (dimension of the domain).
  * \param [in] m    Dimension of the output vector (dimension of the codomain).
  * \param [in] r    Degree of the polynomial.
  *
  * The transformation is initialized to be the identity function.
  *
  * \see reset()
  */

template <class T>
GenericPolynomial<T>::GenericPolynomial(unsigned n, unsigned m, unsigned r) :
    m_matrix(MatrixXT(m, numberMonomials(n, r))),
    number_input_variables(n),
    dimension(m),
    degree(r)
{
    assert(n > 0);
    assert(m > 0);
    reset();
}


/** \tparam T scalar type
  *
  * \param [in] matrix  A matrix containing the coefficients.
  * \param [in] n       Number of input variables (dimension of the domain).
  * \param [in] m       Dimension of the output vector (dimension of the codomain).
  * \param [in] r       Degree of the polynomial.
  *
  * \brief Constructs an instance of GenericPolynomial.
  *
  * Sets the internal coefficients to the given \p matrix. The matrix must have
  * m rows and \f$ \sum_{i=0}^{r} \binom{n + i - 1}{i} \f$ columns.
  *
  * \see reset()
  */

template <class T>
GenericPolynomial<T>::GenericPolynomial(const MatrixXT & matrix, unsigned n, unsigned m, unsigned r) :
    number_input_variables(n),
    dimension(m),
    degree(r)
{
    assert(n > 0);
    assert(m > 0);
    assert(matrix.rows() == m && matrix.cols() == numberMonomials(n, r));
    m_matrix = matrix;
}


/** \tparam T scalar type of the newly created instance
  *
  * \brief Copy constructor.
  *
  * \see reset()
  */

template <class T>
GenericPolynomial<T>::GenericPolynomial(const GenericPolynomial & other)
{
    m_matrix =  other.m_matrix;
    number_input_variables = other.number_input_variables;
    dimension = other.dimension;
    degree = other.degree;
}


/** \tparam T scalar type of the newly created instance
  * \tparam U scalar type of the copied instance
  *
  * \param [in] other   An instance of GenericPolynomial with differing template parameter.
  * \param [in] n       Number of input variables (dimension of the domain).
  * \param [in] m       Dimension of the output vector (dimension of the codomain).
  * \param [in] r       Degree of the polynomial.
  *
  * \brief Copy constructor.
  *
  * \see reset()
  */

template <class T>
template <class U>
GenericPolynomial<T>::GenericPolynomial(const GenericPolynomial<U> & other, unsigned n, unsigned m, unsigned r)
{
    m_matrix = other.getCoefficients().template cast<T>();
    number_input_variables = n;
    dimension = m;
    degree = r;
}


/** \tparam T scalar type
  *
  * \brief Destroys the instance of GenericPolynomial.
  */

template <class T>
GenericPolynomial<T>::~GenericPolynomial()
{
}


/** \tparam T scalar type
  *
  * \brief Returns the internal coefficients.
  */

template <class T>
typename GenericPolynomial<T>::MatrixXT & GenericPolynomial<T>::getCoefficients()
{
    return m_matrix;
}


/** \tparam T scalar type
  *
  * \brief Returns the internal coefficients.
  */

template <class T>
const typename GenericPolynomial<T>::MatrixXT & GenericPolynomial<T>::getCoefficients() const
{
    return m_matrix;
}


/** \tparam T scalar type
  *
  * \brief Estimates the polynomial function from two point sets.
  *
  * The input parameters are ranges <TT>[first, last)</tt> of a sequence (e.g. a
  * STL container like \c vector). The last elements are not included in the
  * ranges which coincides with the convention in the STL.
  *
  * Source and target points as well as weights must correspond to each other.
  * If no weights are given, the weights are assumed to be equal to one.
  *
  * \return Returns the RMSE \f$ = \sqrt{\frac{1}{N} \sum_{i=1}^N w_i
  *         (f(x_i) - y_i)^2} \f$ of the estimation.
  *
  * \throws ErrorObj    An error object is thrown if the point sets differ
  *                     in their sizes.
  * \throws ErrorObj    An error object is thrown if not enough (less than
  *                     ten) points are given.
  */

template <class T>
T GenericPolynomial<T>::estimate(Range<Point> source_points, Range<Point> target_points, Range<T> weights)
{
    return estimate(source_points.begin(),
                    source_points.end(),
                    target_points.begin(),
                    target_points.end(),
                    weights.begin(),
                    weights.end());
}


/** \tparam T scalar type
  *
  * \brief Estimates the polynomial function from two point sets.
  *
  * The input parameters are ranges <TT>[first, last)</tt> of a sequence (e.g. a
  * STL container like \c vector). The last elements are not included in the
  * ranges which coincides with the convention in the STL.
  *
  * Source and target points as well as weights must correspond to each other.
  * If no weights are given, the weights are assumed to be equal to one.
  *
  * \return Returns the RMSE \f$ = \sqrt{\frac{1}{N} \sum_{i=1}^N w_i
  *         (f(x_i) - y_i)^2} \f$ of the estimation.
  *
  * \throws ErrorObj    An error object is thrown if the point sets differ
  *                     in their sizes.
  * \throws ErrorObj    An error object is thrown if not enough (less than
  *                     ten) points are given.
  */

template <class T>
T GenericPolynomial<T>::estimate(Iterator<Point> source_points_first,
                                      Iterator<Point> source_points_last,
                                      Iterator<Point> target_points_first,
                                      Iterator<Point> target_points_last,
                                      Iterator<T> weights_first,
                                      Iterator<T> weights_last)
{
    const int number_points = distance(source_points_first, source_points_last);

    if (distance(target_points_first, target_points_last) != number_points)
    {
        ErrorObj error;
        error.setClassName("GenericPolynomial<T>");
        error.setFunctionName("estimate");
        error.setErrorMessage("Point sets must have the same cardinality.");
        error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
        throw error;
    }

    if (weights_first && weights_last)
    {
        if (distance(weights_first, weights_last) != number_points)
        {
            ErrorObj error;
            error.setClassName("GenericPolynomial<T>");
            error.setFunctionName("estimate");
            error.setErrorMessage("Point sets must have the same cardinality.");
            error.setErrorCode(UNEQUAL_NUMBER_OF_POINTS);
            throw error;
        }
    }

    if (number_points < 10)
    {
        ErrorObj error;
        error.setClassName("GenericPolynomial<T>");
        error.setFunctionName("estimate");
        error.setErrorMessage("Not enough points to estimate the transformation.");
        error.setErrorCode(NOT_ENOUGH_POINTS);
        throw error;
    }

    /*

    Explanation of the algorithm:

    The polynomial can be written as follows (here n=3, m=3, r=2):

    | x' |     | m_1,1  m_1,2  ...  m_1,10 |
    | y  |  =  | m_2,1  m_2,2  ...  m_2,10 |  *  | 1  x  y  ...  y^2  yz  z^2 |^T
    | z' |     | m_3,1  m_3,2  ...  m_3,10 |

    The equation can be rewritten as:

    | x' |     | 1  ...  z^2  0  ...  0    0  ...  0   |     | m_1,1  |
    | y' |  =  | 0  ...  0    1  ...  z^2  0  ...  0   |  *  | ...    |
    | z' |     | 0  ...  0    0  ...  0    1  ...  z^2 |     | m_3,10 |

    Or in short:  b = A * x          (here, x is meant to be (m_1,1, ..., m_3,10)^T)

    By using more and more points (source_1, target_1), ..., (source_n, target_n)

         | A(1) |         | b(1) |
    A := | ...  | ,  b := | ...  |
         | A(n) |         | b(n) |

    this leads to an over-determined system. However, this can be easily solved
    using the pseudo inverse:

    <==>    A * x = b
    <==>    A^T * A * x = A^T * b
    <==>    x = inverse(A^T * A) * A^T * b = pseudo_inverse * b

    Thus, we have found the sought coefficients.

    Update: We now use weighted least squares so that x = inverse(A^T * W * A) * A^T * W * b
            where W is a diagonal matrix

    */

    const unsigned & n = number_input_variables;
    const unsigned & m = dimension;
    const unsigned & r = degree;

    // Compute the monomials (terms like x^2yz)

    unsigned number_of_monomials = numberMonomials(n, r); // number of all possible monomials (1, x, y, ..., x^2, y^2, ...)

    std::vector<VectorXT> monomials; // e.g. monomials[3] might be something like x^2 := (x1^2, x2^2, ..., xn^2)

    computeMonomials(number_input_variables, degree, number_points, source_points_first, monomials);

    // Fill the matrix A with the above described entries (different order as above).

    MatrixXT A(m * number_points, m * number_of_monomials);

    A.setZero();

    for (unsigned j = 0; j < m; ++j) // rows
    {
        for (unsigned i = 0; i < number_of_monomials; ++i) // columns
        {
            int row = j * number_points;
            int col = j * number_of_monomials + i;
            A.block(row, col, number_points, 1) = monomials[i];
        }
    }

    // Fill the vector b.

    VectorXT b(m * number_points);

    int i = 0;
    for (Iterator<Point> it = target_points_first; it != target_points_last; ++it, ++i)
    {
        const Point & point = *it;

        for (unsigned j = 0; j < m; ++j)
        {
            b(j * number_points + i) = point[j];
        }
    }

    // Compute the vector x whose components are the sought coefficients.

    VectorXT x;
    VectorXT W = VectorXT::Ones(m * number_points); // diagonal weighting matrix (W.asDiagonal())

    if (weights_first && weights_last)
    {
        // Weighted Least Squares

        int i = 0;
        for (Iterator<T> it = weights_first; it != weights_last; ++it, ++i)
        {
            T weight = *it;

            for (unsigned j = 0; j < m; ++j)
            {
                W(j * number_points + i) = weight;
            }
        }

        x = (A.transpose() * W.asDiagonal() * A).inverse() * (A.transpose() * (W.asDiagonal() * b));
    }
    else
    {
        // Ordinary Least Squares

        MatrixXT pseudo_inverse = (A.transpose() * A).inverse() * A.transpose();
        x = pseudo_inverse * b;
    }

    // Reinterprete the solution vector as a row-major matrix

    // m_matrix = Eigen::Map<MatrixXT, Eigen::Unaligned, Eigen::Stride<1, 10> >(x.data(), m, 10);
    m_matrix = Eigen::Map<MatrixXT>(x.data(), number_of_monomials, m).transpose();

    // Compute the RMSE.

    using std::sqrt;

    if (weights_first && weights_last)
    {
        return  (W.cwiseSqrt().asDiagonal() * (A * x - b)).norm() / sqrt(T(number_points));
    }
    else
    {
        return  (A * x - b).norm() / sqrt(T(number_points));
    }
}


/** \tparam T scalar type
  *
  * \param [in] point Point to be transformed.
  *
  * \return Transformed point.
  *
  * \brief Transforms a point with the internally saved transformation.
  */

template <class T>
typename GenericPolynomial<T>::Point GenericPolynomial<T>::operator* (const Point & point) const
{
    return transform(point);
}


/** \tparam T scalar type
  *
  * \brief Resets the transformation.
  *
  * Sets the internal coefficients such that the transformation is similar to
  * the identity function. That is, the i-th component of the input vector
  * is mapped to the i-th component of the output vector. If the dimension of
  * the codomain is less than the dimension of the domain, the values are
  * dropped. The other way round the output vector is padded with zeros.
  *
  * Example:
  * \verbatim
    (1, 2, 3) maps to (1, 2)                (n = 3, m = 2)
    (1, 2, 3) maps to (1, 2, 3)             (n = 3, m = 3)
    (1, 2, 3) maps to (1, 2, 3, 0)          (n = 3, m = 4)
    \endverbatim
  *
  * \return \c *this
  */

template <class T>
Polynomial<T> & GenericPolynomial<T>::reset()
{
    m_matrix.setZero();

    for (unsigned i = 0; i < std::min(number_input_variables, dimension); ++i)
    {
        m_matrix(i, i + 1) = 1;
    }

    return *this;
}


/** \tparam T scalar type
  *
  * \param [in] point Point to be transformed.
  *
  * \return Transformed point.
  *
  * \brief Transforms a point with the internally saved transformation.
  */

template <class T>
typename GenericPolynomial<T>::Point GenericPolynomial<T>::transform(const Point & point) const
{
    // See computeMonomials for an explanation of the algorithm.

    assert(point.size() > 0);
    assert(point.size() == number_input_variables);

    // Initialization.

    VectorXT monomials(numberMonomials(number_input_variables, degree));
    std::vector<unsigned> begin_of_group(number_input_variables);
    unsigned end;

    // Construct the first n + 1 monomials.

    monomials(0) = 1;

    for (unsigned i = 0; i < point.size(); ++i)
    {
        monomials(i + 1) = point[i];
    }

    // Set up the indeces of the particular ranges.

    for (unsigned i = 0; i < number_input_variables; ++i)
    {
        begin_of_group[i] = i + 1;
    }

    end = number_input_variables + 1;

    // Compute the remaining monomials.

    for (unsigned i = 2; i <= degree; ++i)
    {
        unsigned end_of_groups = end;
        for (unsigned variable = 0; variable < number_input_variables; ++variable)
        {
            unsigned index = begin_of_group[variable];
            begin_of_group[variable] = end;
            while (index < end_of_groups)
            {
                monomials(end++) = monomials(index++) * monomials(variable + 1);
            }
        }
    }

    // Compute the transformed point.

    return Point(m_matrix * monomials);
}


} // namespace TRTK


#endif // GENERIC_POLYNOMIAL_HPP_7477314703
