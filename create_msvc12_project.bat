@echo off

call "%VS120COMNTOOLS%\vsvars32.bat"

mkdir build
mkdir build\VisualStudio2013

cd build\VisualStudio2013
cmake -G"Visual Studio 12" ..\..

cd ..\..

@echo Done.
timeout /t 15
