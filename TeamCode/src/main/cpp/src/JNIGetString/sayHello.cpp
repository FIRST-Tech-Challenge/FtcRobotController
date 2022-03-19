#include <jni.h>

//
// Created by willm on 3/8/2022.
//


extern "C"
JNIEXPORT jstring JNICALL
Java_org_firstinspires_ftc_teamcode_src_utills_JNIGetString_stringFromJNI(JNIEnv *env,
                                                                          jclass thiz) {
    return (env)->NewStringUTF("Hello from JNI! 🚀 Compiled By: Jacob.");
}
extern "C"
JNIEXPORT jint JNICALL
Java_org_firstinspires_ftc_teamcode_src_utills_JNIGetString_intFromJNI(JNIEnv *env, jclass clazz) {
    return 4;
}


extern "C"
JNIEXPORT jdouble JNICALL
Java_org_firstinspires_ftc_teamcode_src_utills_JNIGetString_extraCallFromJNI(JNIEnv *env,
                                                                             jclass clazz,
                                                                             jobject o) {
    jclass cls = (env)->GetObjectClass(o);
    jfieldID fidDouble = env->GetFieldID(cls, "val", "D");
    return env->GetDoubleField(o, fidDouble);
}