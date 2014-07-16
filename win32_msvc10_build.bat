@echo off

echo ############################################################
echo ### Build script for installation inside MEDITEC_LIBS    ###
echo ###                                                      ###
echo ### Specify command line parameters for build options:   ###
echo ###    "-DBUILD_TEST=ON"     Build unit tests            ###
echo ###    "-DBUILD_DOC=ON"      Build API documentation     ###
echo ############################################################
timeout /t 5

call "%VS100COMNTOOLS%\vsvars32.bat"

mkdir build
mkdir build\nmake_debug
mkdir build\nmake_release
mkdir build\VisualStudio2010

cd build\nmake_debug
cmake -G"NMake Makefiles" %1 %2 ..\..
nmake
nmake install

cd ..\nmake_release
cmake -G"NMake Makefiles" %1 %2 -DBUILD_DOC=OFF -DCMAKE_BUILD_TYPE=Release ..\..
nmake
nmake install

cd ..\VisualStudio2010
cmake -G"Visual Studio 10" %1 %2 ..\..

cd ..\..