Last changed on 2011-12-15.


This ReadMe.txt describes, how to build the TRTK unit tests on various systems
using the CMake build system.

We assume that Eigen as well as OpenCV are installed via the getLibraries.py
script, that is, that the libraries are residing in "../..". We currently also
assume that Boost is installed.

Maybe, you still need to include the Boost root directory and its library path
in the "path" environment variable.



Linux
=====

Just enter:

    mkdir build
    cd build
    cmake -G "Unix Makefiles" ..

on the console and the run

    make

That's all!



Windows
=======

Before compiling from the command prompt, you should run the vcvarsall.bat from 
the VC directory of your Visual Studio installation, which sets some neccessary 
environment variables. This could look like:

> "C:\Program Files\Microsoft Visual Studio 9.0\VC\vcvarsall.bat"

Building OpenCV:

    Download the sources from http://sourceforge.net/projects/opencvlibrary/files/opencv-win/
    Create a new build directory, such as "OpenCV-Build" somewhere. 
    Open a command prompt inside that build directory and invoke the 
    vcvarsall.bat as described above. Then compile with:

    > cmake -G "NMake Makefiles" -DBUILD_NEW_PYTHON_SUPPORT=OFF <Path_To_OpenCV_Sources> 
    > nmake

    Of course <Path_To_OpenCV_Sources> needs to be replaced with the actual path 
    of the downloaded sources. CMake remembers the location it has built OpenCV 
    and sets the OpenCV_DIR variable accordingly when building the unit tests in 
    the next step.
    
    Notice: Do not build this as a Visual Studio Project, because then library 
            files are placed in lib/debug instead of plain lib/, causing linker 
            errors later.


Install Eigen3:

    Download the sources from http://eigen.tuxfamily.org/index.php?title=Main_Page#Download
    and place them in the directory "C:\Eigen".

    Notice: Actually this is unnessaccary, because Eigen should already be placed 
            at "..\.." which is included by default. However the FindEigen3.cmake 
            module will still fail, because it does not find anything.
        

Building the TRTK unit tests:

    Open a command prompt inside the sources directory of the unit tests (where 
    this ReadMe file resists). Again do not forget to invoke the vcvarsall.bat.
    Then compile with:
    
    > mkdir build
    > cd build
    > set PATH=%PATH%;<PATH_TO_EIGEN_SOURCES>;<PATH_TO_OPENCV_SOURCES>
    > cmake -G "NMake Makefiles" ..
    > nmake

    For a Visual Studio project invoke CMake with (not tested):
        cmake -G "Visual Studio 9 2008" ..


Running the unit tests:

    You can either
        copy all *.dll files from the "<Path-To_OpenCV-Build>\bin" directory to 
        your current unit tests build directory
    or
        set the environment variable PATH as follows:
        > set PATH=%PATH%;<Path_To_OpenCV-Build>\bin
        
    From inside the sources directory of the unit tests run on the command prompt:
   
    > build\unit_tests.exe
    
    
Problems:
    
    Linker errors concerning OpenCV while building the unit tests can occur if the 
    OpenCV_DIR variable in the cache of CMake was not set up correctly. Verify that 
    the OpenCV_DIR variable inside the CMakeCache.txt file points to the build directory 
    of OpenCV (not the sources directory!).
    Also look for the line in the file OpenCVConfig.cmake in the OpenCV build directory 
    beginning with "LINK_DIRECTORIES" and make sure the proposed link directories really 
    contain the library files.




Windows + MSys + MinGW
======================

Just enter:

    mkdir build
    cd build
    cmake -G "MSYS Makefiles" ..

on the console and the run

    make

That's all!
