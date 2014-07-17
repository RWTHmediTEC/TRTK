@echo off

call "%VS100COMNTOOLS%\vsvars32.bat"

mkdir build64
mkdir build64\VisualStudio2010

cd build64\VisualStudio2010
cmake -G"Visual Studio 10 Win64" ..\..

cd ..\..