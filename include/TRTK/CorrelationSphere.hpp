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

/** \file CorrelationSphere.hpp
*
* \brief This file contains the declaration of the \ref TRTK::CorrelationSphere
*        "CorrelationSphere" class.
*/


#ifndef CorrelationSphere_HPP_5629756123
#define CorrelationSphere_HPP_5629756123

#include <vector>
#include <cassert>
#include <cstddef>

#include "TRTK/Tools.hpp"
#include "ErrorObj.hpp"
#include "Signals.hpp"

// If present, CImg can use the fftw3 lib for speed up.
// Also note that CImg's native FFT seems to behave differently, thus
// there is no guarantee this class works properly without fftw3
#if FFTW3_FOUND
    #define cimg_use_fftw3
#endif

#include "CImg.h"
using namespace cimg_library;

namespace TRTK
{

    // Auxiliary type
    struct Sphere
    {
        Sphere() : x(0), y(0), z(0), radius(0)
        {
        }

        Sphere(const double x, const double y, const double z, const double radius) :
            x(x), y(y), z(z), radius(radius)
        {
        }

        double x, y, z, radius;
    };

    /**
    * This class segment circles [or spheres] in 2D [3D] images. Just instantiate a class object, set your
    * image and the radius range of the sought circles/spheres. After calling compute() the highest-weighted
    * circle/sphere can be found either by calling findSingleSphere() or by searching the parameter space yourself.
    * For the latter, getCorrelations() provides the vector of 2D [3D] images.
    *
    * Computation time is linear in delta_r and dimensions d and superlinear [n*log(n)] in the number of pixels per dimension n.
    *
    * Code Example (2D):
    *   \code
    *    // Construct a sample image & segment sphere out of it
    *    CImg<unsigned char> * im = new CImg<unsigned char>(128, 128, 1, 1, 15);
    *    cimg_forXYZ(*im, x, y, z){
    *        float dist = std::sqrt((x - 30)*(x - 30) + (y - 30)*(y - 30));
    *        if ((dist > 9.5) && (dist < 10.5))
    *            (*im)(x, y, z) = 255;
    *    }
    *    TRTK::CorrelationSphere<unsigned char> correlation = TRTK::CorrelationSphere<unsigned char>(im);
    *    delete im;
    *    correlation.compute();
    *    TRTK::Sphere sphere = correlation.findSingleSphere();
    *    std::cout << QString::number(sphere.x).toStdString() << " " << QString::number(sphere.y).toStdString() << " " << QString::number(sphere.z).toStdString() << " " << QString::number(sphere.radius).toStdString() << " " << std::endl;
    *   \endcode
    *
    * Expected output:
    *
    *   30 30 0 10
    */

    template<typename T>
    class CorrelationSphere
    {
    public:
        enum Error { UNKNOWN_ERROR };

        // Helper function
        CImg<double> * ConvertToGrayScaleDouble(const CImg<T>* input);

        // Constructors
        CorrelationSphere();
        CorrelationSphere(const CImg<T> * image);

        // Destructor
        virtual ~CorrelationSphere();

        // Computes the parameter space values
        // (a 4-dimensional space: x, y, z and radius are the parameters)
        void compute();

        // Find the best matching parameters
        Sphere findSingleSphere() const;

        // Setter & Getter
        std::vector<CImg<double> *> & getCorrelations();
        void setImage(const CImg<T> * image);

        void setRadii(int minRadius, int maxRadius);
        void setThickness(int thickness);

        // Signal to output progress
        Signal<int> progress;

    private:
        CImg<double> * image;
        std::vector<CImg<double> *> correlations;

        int minRadius;
        int maxRadius;
        int thickness;
    };



    ////////////////////////////
    /// Function definitions ///
    ////////////////////////////

    template <typename T>
    CImg<double> * CorrelationSphere<T>::ConvertToGrayScaleDouble(const CImg<T>* input)
    {
        CImg<double> * output = new CImg<double>(input->width(), input->height(), input->depth(), 1);

        // Is input an RGB color image?
        if (input->spectrum() == 3)
        {
            cimg_forXYZ(*output, x, y, z){
                // Convert to gray ...
                double value = 0.299 * (*input->data(x, y, z, 0)) + 0.587 * (*input->data(x, y, z, 1)) + 0.114 * (*input->data(x, y, z, 2));

                // And adjust value range
                if (std::is_same<T, unsigned char>::value || std::is_same<T, char>::value || std::is_same<T, int>::value)
                    value = static_cast<double>(value) / 255;

                (*output)(x, y, z) = value;
            }
        }
        else if (input->spectrum() == 1)
        {
            cimg_forXYZ(*output, x, y, z){
                // Get image value
                double value = (*input)(x, y, z);

                // adjust value range
                if (std::is_same<T, unsigned char>::value || std::is_same<T, char>::value || std::is_same<T, int>::value)
                    value = static_cast<double>(value) / 255;

                (*output)(x, y, z) = value;
            }
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

    template<typename T>
    CorrelationSphere<T>::CorrelationSphere() :
        image(nullptr),
        minRadius(10),
        maxRadius(10),
        thickness(1)
    {}


    template<typename T>
    CorrelationSphere<T>::CorrelationSphere(const CImg<T> * image) :
        image(nullptr),
        minRadius(10),
        maxRadius(10),
        thickness(1)
    {
        try
        {
            this->image = ConvertToGrayScaleDouble(image);
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


    template<typename T>
    CorrelationSphere<T>::~CorrelationSphere()
    {
        for (unsigned int i = 0; i < correlations.size(); ++i)
        {
            if (correlations[i]) delete correlations[i];
        }

        if (image) delete image;
    }


    template <typename T>
    void CorrelationSphere<T>::compute()
    {
        // initialize some variables
        int progress = 0;
        const int deltaRadius = maxRadius - minRadius;

        for (unsigned int i = 0; i < correlations.size(); ++i)
        {
            delete correlations.at(i);
        }

        correlations.resize(deltaRadius + 1);

        // compute correlation

        // Precompute FFT of original image as it is used multiple times
        CImg<double> * imReal = new CImg<double>(*image);
        CImg<double> * imImag = new CImg<double>(image->width(), image->height(), image->depth(), 1, 0);

        CImg<double>::FFT(*imReal, *imImag);

        for (int r = 0; r < deltaRadius + 1; ++r)
        {
            const int radius = minRadius + r;
            int weight = 0;

            // Initialize kernel and result images
            CImg<double> * resultReal = new CImg<double>(image->width(), image->height(), image->depth(), 1, 0);
            CImg<double> * resultImag = new CImg<double>(image->width(), image->height(), image->depth(), 1, 0);
            CImg<double> * kernelReal = new CImg<double>(image->width(), image->height(), image->depth(), 1, 0);
            CImg<double> * kernelImag = new CImg<double>(image->width(), image->height(), image->depth(), 1, 0);

            //Construct kernel
            //This may be optimized. For example the bresenham algorithm for spheres could be used
            //another optimization: Chunkwise
            double w = kernelReal->width() / 2.0;
            double h = kernelReal->height() / 2.0;
            double d = kernelReal->depth() / 2.0;

            if (kernelReal->depth() > 1){
                // 3D Case
                cimg_forXYZ(*kernelReal, x, y, z){
                    float dist = std::sqrt((x - w)*(x - w) + (y - h)*(y - h) + (z - d)*(z - d));
                    if ((dist > radius - 0.5 * thickness) && (dist < radius + 0.5 * thickness)){
                        (*kernelReal)(x, y, z) = 1;
                        weight++;
                    }
                }
            }
            else {
                // 2D Case
                cimg_forXY(*kernelReal, x, y){
                    float dist = std::sqrt((x - w)*(x - w) + (y - h)*(y - h));
                    if ((dist > radius - 0.5 * thickness) && (dist < radius + 0.5 * thickness)){
                        (*kernelReal)(x, y) = 1;
                        weight++;
                    }
                }
            }

            // Apply FFT on image (done outside of loop) & kernel
            CImg<double>::FFT(*kernelReal, *kernelImag);

            // Shift kernel such that the DC component is in the center
            cimg_forXYZ(*kernelReal, x, y, z){
                (*kernelReal)(x, y, z) *= std::pow(-1, x + y + z);
                (*kernelImag)(x, y, z) *= std::pow(-1, x + y + z);
            }

            // multiply image with kernel in fourier domain (= convolution in spatial domain)
            // [may be optimized to work on kernel & image only, saving the memory for result]
            cimg_forXYZ(*resultReal, x, y, z){
                (*resultReal)(x, y, z) = (*imReal)(x, y, z) * (*kernelReal)(x, y, z) - (*imImag)(x, y, z) * (*kernelImag)(x, y, z);
                (*resultImag)(x, y, z) = (*imReal)(x, y, z) * (*kernelImag)(x, y, z) + (*imImag)(x, y, z) * (*kernelReal)(x, y, z);
            }

            // Apply Inverse FFT
            CImg<double>::FFT(*resultReal, *resultImag, true);

            // Normalize result
            cimg_forXYZ(*resultReal, x, y, z){
                (*resultReal)(x, y, z) /= weight;
            }

            // save result; 'correlations' forms a 4D parameter space whose
            // local maxima denote the sought circles
            correlations[r] = resultReal;

            // release resources
            // do not release resultReal as we want to keep it in correlations
            delete kernelReal;
            delete kernelImag;
            delete resultImag;

            // publish progress
            progress += 100 / (deltaRadius + 1);
            this->progress.send(progress);
        }

        // release resources
        delete imReal;
        delete imImag;

        if (progress != 100) this->progress.send(100);
    }


    template <typename T>
    Sphere CorrelationSphere<T>::findSingleSphere() const
    {
        int progress = 0;

        int x0 = 0;
        int y0 = 0;
        int z0 = 0;
        int r0 = 0;

        // iterate slice by slice through the parameter space and find the maximum
        // value; this value corresponds to the center point of the circle
        double globalMaximum = 0;

        for (int r = 0; r < int(correlations.size()); ++r)
        {
            cimg_forXYZ(*correlations[r], x, y, z){
                if ((*correlations[r])(x, y, z) > globalMaximum){
                    x0 = x;
                    y0 = y;
                    z0 = z;
                    r0 = r;
                    globalMaximum = (*correlations[r])(x, y, z);
                }
            }

            progress += 100 / correlations.size();
            this->progress.send(progress);
        }

        r0 += minRadius;

        if (progress != 100) this->progress.send(100);

        return Sphere(x0, y0, z0, r0);
    }


    template<typename T>
    void CorrelationSphere<T>::setImage(const CImg<T> * image)
    {
        this->image = ConvertToGrayScaleDouble(image);
    }


    template<typename T>
    void CorrelationSphere<T>::setRadii(int minRadius, int maxRadius)
    {
        assert(minRadius >= 0);
        assert(maxRadius >= 0);
        assert(maxRadius >= minRadius);

        this->minRadius = minRadius;
        this->maxRadius = maxRadius;
    }


    template<typename T>
    void CorrelationSphere<T>::setThickness(int thickness)
    {
        this->thickness = thickness;
    }


    template<typename T>
    std::vector<CImg<double> *> & CorrelationSphere<T>::getCorrelations()
    {
        return this->correlations;
    }

} // namespace TRTK


#endif // CorrelationSphere_HPP_5629756123
