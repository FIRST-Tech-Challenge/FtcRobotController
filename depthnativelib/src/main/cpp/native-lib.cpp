#include <jni.h>
#include <string>

#include <libusb.h>
#include "depthai/depthai.hpp"

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_depthaiandroid_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    auto r = libusb_set_option(nullptr, LIBUSB_OPTION_ANDROID_JNIENV, env);

    // Create pipeline
    dai::Pipeline pipeline;

    // Connect to device and start pipeline
    dai::Device device(pipeline, dai::UsbSpeed::SUPER);


    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}

