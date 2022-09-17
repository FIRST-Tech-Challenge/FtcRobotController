/*
 * Copyright (c) 2020 OpenFTC Team
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
#include <camera2_jni.h>

Camera2jni_context::Camera2jni_context(int width, int height)
{
    y_mat = Mat(height, width, CV_8UC1, 4634); // placeholder data ptr. Is properly set in colorConversion
    uv_mat1 = Mat(height / 2, width / 2, CV_8UC2, 4634); // placeholder data ptr. Is properly set in colorConversion
    uv_mat2 = Mat(height / 2, width / 2, CV_8UC2, 4634); // placeholder data ptr. Is properly set in colorConversion
}

extern "C"
JNIEXPORT jlong JNICALL
Java_org_openftc_easyopencv_OpenCvInternalCamera2Impl_createNativeContext(JNIEnv *env, jobject thiz,
    jint width, jint height)
{
    Camera2jni_context* context = new Camera2jni_context(width, height);
    return (jlong)context;
}

extern "C"
JNIEXPORT void JNICALL
Java_org_openftc_easyopencv_OpenCvInternalCamera2Impl_releaseNativeContext(JNIEnv *env, jobject thiz, jlong ptrContext)
{
    Camera2jni_context* context = (Camera2jni_context*) ptrContext;
    delete context;
}

extern "C"
JNIEXPORT void JNICALL
Java_org_openftc_easyopencv_OpenCvInternalCamera2Impl_colorConversion(JNIEnv *env, jobject thiz,
    jobject plane0, jobject plane1, jobject plane2, jlong ptr_context, jlong ptr_rgb_frame)
{
    // Get some pointers
    void* ptrPlane0_data = env->GetDirectBufferAddress(plane0);
    void* ptrPlane1_data = env->GetDirectBufferAddress(plane1);
    void* ptrPlane2_data = env->GetDirectBufferAddress(plane2);
    Camera2jni_context* context = (Camera2jni_context*) ptr_context;
    Mat* rgbFrame = (Mat*) ptr_rgb_frame;

    // Point the mats to the new image data
    context->y_mat.data = (uchar*) ptrPlane0_data;
    context->uv_mat1.data = (uchar*) ptrPlane1_data;
    context->uv_mat2.data = (uchar*) ptrPlane2_data;

    /*
     * Actually do the color conversion. Found on the interwebs. I think what's going on here is that
     * the data from the camera is sent as the Y plane and then the Cr and Cb planes interleaved.
     */
    long addr_diff = context->uv_mat2.data - context->uv_mat1.data;
    if (addr_diff > 0)
    {
        //assert(addr_diff == 1);
        cvtColorTwoPlane(context->y_mat, context->uv_mat1, *rgbFrame, COLOR_YUV2RGBA_NV12);
    }
    else
    {
        //assert(addr_diff == -1);
        cvtColorTwoPlane(context->y_mat, context->uv_mat2, *rgbFrame, COLOR_YUV2RGBA_NV21);
    }
}