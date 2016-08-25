#!/bin/bash

JAVA_WRAPPER_PATH=src/io/cens/android/onboard/crash
CRASH_REPO=/Users/antoine/eng/mobile-crash-sdk

rm -r $JAVA_WRAPPER_PATH
mkdir -p $JAVA_WRAPPER_PATH
find jni -name "*.hpp" -type f -delete
find jni -name "*.h" -type f -delete
find jni -name "*.cpp" -type f -delete
find jni -name "*.cxx" -type f -delete
find jni -name "*.cc" -type f -delete
find jni -name "*.c" -type f -delete

cp -R $CRASH_REPO/src/ jni
swig -c++ -java -package io.cens.android.onboard.crash -I../src -outdir $JAVA_WRAPPER_PATH jni/CrashDetector.i