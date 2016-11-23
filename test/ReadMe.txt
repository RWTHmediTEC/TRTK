Last changed on 2011-12-15.


This ReadMe.txt describes, how to build the TRTK unit tests on various systems
using the CMake build system.

We assume that Eigen is installed via the getLibraries.py
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
    > set PATH=%PATH%;<PATH_TO_EIGEN_SOURCES>
    > cmake -G "NMake Makefiles" ..
    > nmake

    For a Visual Studio project invoke CMake with (not tested):
        cmake -G "Visual Studio 9 2008" ..


Running the unit tests:
        
    From inside the sources directory of the unit tests run on the command prompt:
   
    > build\unit_tests.exe


	
Windows Alternative
===================

Instead of building the unit tests separately, they can be built along with TRTK itself.
To do this switch to the TRTK main directory (the one that contains the 'tests' 
folder with this readme in it) and run command prompt:

	> cmake-gui .
	
The directories of the source code and the build directory should already be set (for example
to "C:/Users/your_username/TRTK" and "C:/Users/your_username/TRTK/build/VisualStudio2013/").
BUILD_TEST must be set to true. After configuring & generating, the VisualStudio TRTK project contains
a unit_tests project that can be built & run.


Windows + MSys + MinGW
======================

Just enter:

    mkdir build
    cd build
    cmake -G "MSYS Makefiles" ..

on the console and the run

    make

That's all!
