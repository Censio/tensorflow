#!/bin/bash

JAVA_WRAPPER_PATH=src/io/cens/android/onboard/crash

rm -r $JAVA_WRAPPER_PATH
mkdir -p $JAVA_WRAPPER_PATH
swig -c++ -java -package io.cens.android.onboard.crash -I../src -outdir $JAVA_WRAPPER_PATH jni/CrashDetector.i

# TODO: build with Bazel
