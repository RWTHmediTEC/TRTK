@echo off

call "%VS100COMNTOOLS%\vsvars32.bat"

mkdir build
mkdir build\VisualStudio2010

cd build\VisualStudio2010
cmake -G "Visual Studio 10" ..\..

cd ..\..