/*
    Copyright (C) 2010 - 2014 Christoph Haenisch

    Chair of Medical Engineering (mediTEC)
    RWTH Aachen University
    Pauwelsstr. 20
    52074 Aachen
    Germany

    See license.txt for more information.

    Version 0.3.1 (2011-11-04)
*/

/** \file CorrelationCircle.cpp
  * \brief This file implements the \ref TRTK::CorrelationCircle
  *        "CorrelationCircle" class.
  */


#include <cassert>
#include <cstddef>

#ifdef _OPENMP
    #include <omp.h>
#endif

#include "TRTK/CorrelationCircle.hpp"
#include "TRTK/ErrorObj.hpp"
#include "TRTK/Tools.hpp"


using namespace cv;


namespace TRTK
{


/////////////////////////////////////////////////////////////////////////////
//        Helper functions (implementations at the end of the file)        //
/////////////////////////////////////////////////////////////////////////////


IplImage * ConvertToGrayScale32F(const IplImage * input);



/////////////////////////////////////////////////////////////////////////////
//                         CorrelationCircle Class                         //
/////////////////////////////////////////////////////////////////////////////


/** Constructs an empty instance of CorrelationCircle. */

CorrelationCircle::CorrelationCircle() :
    image(NULL),
    minRadius(0),
    maxRadius(0),
    thickness(1)
{
}


/** \param [in] image gray scale image
  *
  * Constructs an instance of CorrelationCircle.
  *
  * \note \p image is automatically converted to a gray scale image
  *       if \c image.channels() \c > \c 1.
  *
  * \throw ErrorObj in case the image cannot be appropriately converted.
  */

CorrelationCircle::CorrelationCircle(const IplImage * image) :
    image(NULL),
    minRadius(0),
    maxRadius(0),
    thickness(1)
{
    try
    {
        this->image = ConvertToGrayScale32F(image);
    }
    catch (ErrorObj & err)
    {
        ErrorObj error;
        error.setErrorMessage("From '" + err.getFunctionName() + "': " + err.getErrorMessage());
        error.setClassName("CorrelationCircle");
        error.setFunctionName("CorrelationCircle");
        error.setErrorCode(UNKNOWN_ERROR);
        throw error;
    }
}


/** Destructs the instance of CorrelationCircle. */

CorrelationCircle::~CorrelationCircle()
{
    for (unsigned int i = 0; i < correlations.size(); ++i)
    {
        cvReleaseImage(&correlations[i]);
    }

    if (image) cvReleaseImage(&image);
}


/** \brief Computes the correlation.
  *
  * \see setImage(), setRadii() and getCorrelations()
  */

void CorrelationCircle::compute()
{
    // initialize some variables

    int progress = 0;

    const int deltaRadius = maxRadius - minRadius;

    for (unsigned int i = 0; i < correlations.size(); ++i)
    {
        cvReleaseImage(&correlations[i]);
    }

    correlations.resize(deltaRadius + 1);

    // compute correlation

    #pragma omp parallel for

    for (int r = 0; r < deltaRadius + 1; ++r)
    {
        const int radius = minRadius + r;
        const int diameter = 2 * radius + 1;

        IplImage * result = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);

        // generate kernel containing the circle

        CvMat * kernel = cvCreateMat(diameter, diameter, CV_32F);
        cvSetZero(kernel);
        cvCircle(kernel, cvPoint(radius, radius), radius, cvScalar(1), thickness, CV_AA);

        // correlate

        cvFilter2D(image, result, kernel);

        // save result; 'correlations' forms a 3D parameter space whose
        // local maxima denote the sought circles

        correlations[r] = result;

        // release resources

        cvReleaseMat(&kernel);

        // publish progress

        progress += 100 / (deltaRadius + 1);
        this->progress.send(progress);
    }

    if (progress != 100) this->progress.send(100);
}


/** \brief Finds a single circle in the image.
  *
  * \return radius and center point of the circle
  *
  * \note You must call compute() first.
  */

Circle CorrelationCircle::findSingleCircle() const
{
    int progress = 0;

    int x0 = 0;
    int y0 = 0;
    int r0 = 0;

    // iterate slice by slice through the parameter space and find the maximum
    // value; this value corresponds to the center point of the circle

    double globalMaximum = 0;

    #pragma omp parallel for

    for (int r = 0; r < int(correlations.size()); ++r)
    {
        double sliceMaximum = 0;
        CvPoint location = cvPoint(0, 0);
        cvMinMaxLoc(correlations[r], NULL, &sliceMaximum, NULL, &location);

        if (sliceMaximum > globalMaximum)
        {
            globalMaximum = sliceMaximum;
            x0 = location.x;
            y0 = location.y;
            r0 = r;
        }

        progress += 100 / correlations.size();
        this->progress.send(progress);
    }

    r0 += minRadius;

    if (progress != 100) this->progress.send(100);

    return Circle(x0, y0, r0);
}


/** \brief Returns the the parameter space.
  *
  * \return vector with cross correlations
  *
  * \see compute()
  */

std::vector<IplImage *> & CorrelationCircle::getCorrelations()
{
    return correlations;
}


/** \var CorrelationCircle::progress
  *
  * \brief Current progress during computations.
  *
  * This \ref TRTK::Signals::Signal "signal" notifies subscribers about the
  * current progress during computations. The progress is given in percent,
  * that is, the values range from 0 to 100.
  */


/** \param [in] image image data
  *
  * \brief Sets the 2D binary image.
  *
  * \note \p image is automatically converted to a gray scale image
  */

void CorrelationCircle::setImage(const IplImage * image)
{
    if (image) cvReleaseImage(&this->image);
    this->image = ConvertToGrayScale32F(image);
}


/** \param [in] minRadius minimum radius
  * \param [in] maxRadius maximum radius
  *
  * \brief Sets the range of circle radii to be checked for.
  *
  * The size of the third dimension of the parameter space is equal to
  * \c maxRadius - \c minRadius + \c 1.
  */

void CorrelationCircle::setRadii(const int minRadius, const int maxRadius)
{
    assert(minRadius >= 0);
    assert(maxRadius >= 0);
    assert(maxRadius >= minRadius);

    this->minRadius = minRadius;
    this->maxRadius = maxRadius;
}


/** \param [in] thickness line width of circle pattern
  *
  * \brief Sets the thickness of the circle patterns.
  *
  * If the input image contains circles with a thickness greater
  * than one, than the matching pattern should have about the same
  * thickness. With this method, you can adapt the line width of
  * the circle patterns.
  */

void CorrelationCircle::setThickness(const int thickness)
{
    this->thickness = thickness;
}



/////////////////////////////////////////////////////////////////////////////
//                   Helper functions (implementations).                   //
/////////////////////////////////////////////////////////////////////////////


/** \relates CorrelationCircle
  *
  * \param [in] input   color or grayscale image (8 bit per channel)
  * \returns            grayscale image (pixels represented by doubles)
  *
  * Converts a rgb color or grayscale image with 8 bit per channel into a
  * grayscale image whose pixels are represented by doubles (IPL_DEPTH_32F).
  * The output is normalized, such that the maximum pixel value is less
  * than or equal to 1.
  */

IplImage * ConvertToGrayScale32F(const IplImage * input)
{
    IplImage * output = cvCreateImage(cvGetSize(input), IPL_DEPTH_32F, 1);

    // Is input an RGB color image?

    if (input->nChannels == 3 && (input->depth == IPL_DEPTH_8U || input->depth == IPL_DEPTH_8S))
    {
        // Convert image into a gray scale image...

        IplImage * grayScaleImage = cvCreateImage(cvGetSize(input), IPL_DEPTH_8U, 1);
        cvCvtColor(input, grayScaleImage, CV_RGB2GRAY);

        // ... and convert its pixels into doubles.

        cvConvertScale(grayScaleImage, output, 1.0/255);

        // Release resources.

        cvReleaseImage(&grayScaleImage);
    }
    else if (input->nChannels == 1 && (input->depth == IPL_DEPTH_8U || input->depth == IPL_DEPTH_8S))
    {
        // Already grayscale. Thus, only convert its 8-bit pixels into doubles.

        cvConvertScale(input, output, 1.0/255);
    }
    else
    {
        ErrorObj error;
        error.setErrorMessage("Can't handle image type.");
        error.setFunctionName("ConvertToGrayScale32F");
        throw error;
    }

    return output;
}


} // namespace TRTK
