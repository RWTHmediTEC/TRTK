#!/bin/bash

# cd /D/Users/haenisch/Programmierung/TRTK/trunk/src/doc && \
    doxygen && \
    kdialog.exe --passivepopup "Doxygen: Documentation generation done." 1 || \
    kdialog.exe --passivepopup "ERROR   Doxygen: Documentation generation failed." 3