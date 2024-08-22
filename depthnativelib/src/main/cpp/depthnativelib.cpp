#include <jni.h>
#include <string>

extern "C" JNIEXPORT jstring JNICALL
Java_com_example_depthnativelib_NativeLib_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
extern "C"
JNIEXPORT void JNICALL
Java_com_example_depthnativelib_depthAiCamera_prepareCameraJNI(JNIEnv *env, jobject thiz) {
    // TODO: implement prepareCameraJNI()
}