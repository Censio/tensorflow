#!/bin/bash

JAVA_WRAPPER_PATH=src/io/cens/android/onboard/crash
CRASH_REPO=/Users/antoine/eng/mobile-crash-sdk
CPP_GENERATED_PATH=jni/generated

rm -r $CPP_GENERATED_PATH
rm -r $JAVA_WRAPPER_PATH
mkdir -p $JAVA_WRAPPER_PATH

cp -R $CRASH_REPO/src/ $CPP_GENERATED_PATH
swig -c++ -java -package io.cens.android.onboard.crash -I$CPP_GENERATED_PATH -outdir $JAVA_WRAPPER_PATH -o $CPP_GENERATED_PATH/CrashDetector_wrap.cpp jni/CrashDetector.i