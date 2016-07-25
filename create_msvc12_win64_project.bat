@echo off

call "%VS120COMNTOOLS%\vsvars32.bat"

mkdir build
mkdir build\VisualStudio2013_Win64

cd build\VisualStudio2013_Win64
cmake -G"Visual Studio 12 Win64" ..\..

cd ..\..

@echo Done.
timeout /t 15
