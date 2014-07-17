/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.3.2 (2013-08-20)
*/

/** \file CorrelationCircle.hpp
  *
  * \brief This file contains the declaration of the \ref TRTK::CorrelationCircle
  *        "CorrelationCircle" class.
  */


#ifndef CorrelationCircle_HPP_5629756123
#define CorrelationCircle_HPP_5629756123

#include <vector>

#include <cv.h>

#include "Circle.hpp"
#include "ErrorObj.hpp"
#include "Signals.hpp"


namespace TRTK
{


/** \brief Computes the cross correlation between a 2D input image and several
  *        circle images.
  *
  * This class computes the cross correlation between a 2D input image and several
  * images of circles of varying radius. The result is a 3D volume, which is very
  * similar to the parameter space of a generalized Hough transform (in fact, in
  * case of a binary input image and a circle thickness of one, the cross
  * correlation is equivalent to the generalized Hough transform). The locations
  * and radii of the circles contained in the original image can be found by
  * searching for local maxima in the 3D volume. The three-dimensional parameter
  * space is spanned by the location (x, y) and the radius r of the circles.
  *
  * Here is an example of how to use CorrelationCircle:
  *
  * \code
  * #include <iostream>
  *
  * #include <opencv2/cv.h>
  * #include <TRTK/CorrelationCircle.hpp>
  *
  * using namespace std;
  * using namespace cv;
  * using namespace TRTK;
  *
  *
  * int main(int argc, char *argv[])
  * {
  *     Mat image(256, 256, CV_8U, Scalar(0));
  *
  *     circle(image, Point(100, 200), 20, 255);
  *     circle(image, Point(100, 120), 40, 255);
  *     circle(image, Point( 30,  70), 80, 255);
  *
  *     const IplImage & img = image;
  *     CorrelationCircle correlationCircle(&img);
  *
  *     correlationCircle.setRadii(30, 50);
  *     correlationCircle.compute();
  *
  *     Circle circle = correlationCircle.findSingleCircle();
  *
  *     cout << circle.x << endl
  *          << circle.y << endl
  *          << circle.radius << endl;
  *
  *     return 0;
  * }
  * \endcode
  *
  * Output:
  *
  * \code
  *
  * 101
  * 120
  * 39
  *
  * \endcode
  *
  * \note This class depends on OpenCV 2.2.
  *
  * \see HoughCircle
  *
  * \author Christoph Haenisch
  * \version 0.3.2
  * \date last changed on 2013-08-20
  */

class CorrelationCircle
{
public:
    enum Error {UNKNOWN_ERROR};

    CorrelationCircle();
    CorrelationCircle(const IplImage * image);
    virtual ~CorrelationCircle();

    void compute();

    std::vector<IplImage *> & getCorrelations();

    Circle findSingleCircle() const;

    void setImage(const IplImage * image);
    void setRadii(int minRadius, int maxRadius);
    void setThickness(int thickness);

    Signal<int> progress;

private:
    IplImage * image;
    std::vector<IplImage *> correlations;

    int minRadius;
    int maxRadius;
    int thickness;
};


} // namespace TRTK


#endif // CorrelationCircle_HPP_5629756123
