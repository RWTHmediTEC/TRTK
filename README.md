TRTK
====

Transformation and Registration Toolkit


Table of Contents
-----------------

* Introduction
* Dependencies
* Build Instructions
* Documentation


Introduction
------------

This package provides C++ classes to hold ordinary coordinates and compute
coordinate transformations. It also comprises classes to estimate the
underlying transformation between two point sets such as a rigid, affine or
projective transformation. All transformations are available in 2D and 3D.
In addition there are some classes to solve fitting problems like sphere
fitting etc.


Dependencies
------------

This library builds on top of Eigen 3, a C++ template library for linear
algebra (http://eigen.tuxfamily.org). To work properly, it must be
installed on your system.

You also need the Fast Library for Approximate Nearest Neighbors (FLANN)
(http://www.cs.ubc.ca/research/flann).

The installation of OpenCV (http://opencv.org) is optional.

### Linux ###

On Linux systems the above mentioned software packages are often availabe
through the system's package management software. On Ubuntu or
Debian-based systems just run:

    apt-get install libeigen3-dev
    apt-get install libflann-dev
    apt-get install libopencv-dev (optional)


Build Instructions
------------------

### Linux ###

    mkdir build && cd build
    cmake -G "Unix Makefiles" ..
    make && make install

If you also want to build the unit tests, change the cmake command as
follows:

    cmake -D BUILD_TEST=1 -G "Unix Makefiles" ..

The default installation location on Unix-like systems is /usr/local.
Thus the default installation directories are

    /usr/local/lib
    /usr/local/include/TRTK

You can change this by setting CMAKE_INSTALL_PREFIX:

    cmake -DCMAKE_INSTALL_PREFIX:PATH=/home/christoph/usr/local .

### Windows (Visual Studio 2010 or above) ###

From the Visual Studio command line run the following commands inside
the trunk folder:

    mkdir build
    cd build
    cmake -G "Visual Studio 10" ..

Alternatively, you can run cmake-gui. You can adapt the properties as
necessary.

Start the solution and buid the project. By building the subproject
"INSTALL" the library is installed.

The default installation location on Windows is C:\Program Files. 


Documentation
-------------

You can find the documentation at http://haenisch.github.io/TRTK.

### Building the Documentation ###

The documentation can be generated using 'Doxygen' Version 1.7.2 or higher
(www.doxygen.org). You also need the 'dot' tool provided by 'Graphviz'
(www.graphviz.org).

First install both software packages as denoted on their web sites.

NOTE: 'Graphviz' might need to be in the system variable 'path'.

Then you can either generate the documentation (a) using the graphical tool
'doxywizard' or (b) by running 'doxygen' on the console:

(a) Run 'doxywizard' and open the file './doc/Doxyfile'. Then open the 'Run'
    tab (below 'Step 2') and click on 'Run doxygen'. Now the documentation
    is generated.

(b) On command line change to the './doc' directory and enter 'doxygen'.

Now you should have an 'html' folder in './doc'. Open './doc/html/index.html'
to browse through the documentation.

NOTE: The HTML output is fine-tuned for Doxygen 1.7.2. If the output looks
      kind of ugly in later versions, just leave the 'HTML_FOOTER' tag as
      well as the 'HTML_STYLESHEET' tag located in './doc/Doxyfile' empty.
