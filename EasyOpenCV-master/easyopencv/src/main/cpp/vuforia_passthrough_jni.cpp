/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <jni.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

extern "C"
JNIEXPORT jlong JNICALL
Java_org_openftc_easyopencv_OpenCvVuforiaPassthroughImpl_allocExtImgDatMat(JNIEnv *env, jclass clazz,
                                                               jint width, jint height, jint type)
{
    Mat* mat = new Mat(height, width, type, 4634); // placeholder data ptr. Is properly set in setMatDataPtr
    return (jlong) mat;
}

extern "C"
JNIEXPORT void JNICALL
Java_org_openftc_easyopencv_OpenCvVuforiaPassthroughImpl_freeExtImgDatMat(JNIEnv *env, jclass clazz, jlong ptr)
{
    delete (Mat*) ptr;
}

extern "C"
JNIEXPORT void JNICALL
Java_org_openftc_easyopencv_OpenCvVuforiaPassthroughImpl_setMatDataPtr(JNIEnv *env, jclass thiz,
    jobject pixelsBuffer, jlong raw_sensor_mat_ptr)
{
    // Get some pointers
    void* ptrPixels = env->GetDirectBufferAddress(pixelsBuffer);
    Mat* yuvMat = (Mat*) raw_sensor_mat_ptr;

    // Point the mat to the new image data
    yuvMat->data = (uchar*) ptrPixels;
}

extern "C"
JNIEXPORT void JNICALL
Java_org_openftc_easyopencv_OpenCvVuforiaPassthroughImpl_colorConversionInternalCamera(JNIEnv *env, jclass thiz,
                                                                         jlong raw_sensor_mat_ptr, jlong rgb_ptr)
{
    Mat* rawSensorMat = (Mat*) raw_sensor_mat_ptr;
    Mat* rgb = (Mat*) rgb_ptr;

    cvtColor(*rawSensorMat, *rgb, COLOR_YUV2RGBA_NV21, 4);
}

extern "C"
JNIEXPORT void JNICALL
Java_org_openftc_easyopencv_OpenCvVuforiaPassthroughImpl_colorConversionWebcam(JNIEnv *env, jclass thiz,
                                                                                       jlong raw_sensor_mat_ptr, jlong rgb_ptr)
{
    Mat* rawSensorMat = (Mat*) raw_sensor_mat_ptr;
    Mat* rgb = (Mat*) rgb_ptr;

    cvtColor(*rawSensorMat, *rgb, COLOR_BGR5652RGBA, 4);
}