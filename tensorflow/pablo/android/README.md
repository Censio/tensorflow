# Pablo Android

This folder contains the Pablo project for Android: it build and links the TrueMotion real-time processing C++ code against the TensorFlow library and generates the necessary Java bindings. The output of the build is an arm-v7a shared library and a java library to be consumed by the client Android application.

To build, run the following command from the root of your TensorFlow / Bazel workspace.

```
$ bazel build //tensorflow/pablo/android:censio_crash --crosstool_top=//external:android/crosstool --cpu=armeabi-v7a --host_crosstool_top=@bazel_tools//tools/cpp:toolchain
```