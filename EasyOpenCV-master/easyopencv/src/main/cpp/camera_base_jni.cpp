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
#include <android/native_window.h>
#include <android/native_window_jni.h>
#include <android/log.h>

using namespace cv;

extern "C" JNIEXPORT void JNICALL
Java_org_openftc_easyopencv_OpenCvCameraBase_nativeCopyMatToSurface(JNIEnv *env, jobject thiz, jlong handle, jlong buf)
{
    ANativeWindow* window = (ANativeWindow*)handle;
    Mat* originalMat = (Mat*) buf;
    Mat* mat = originalMat;
    Mat converted = Mat();

    /*
     *  We must put RGBA data into the encoder buffer
     */
    if(originalMat->channels() == 1)
    {
        cvtColor(*originalMat, converted, COLOR_GRAY2RGBA);
        mat = &converted;
    }
    else if(originalMat->channels() == 3)
    {
        cvtColor(*originalMat, converted, COLOR_RGB2RGBA);
        mat = &converted;
    }
    else if(originalMat->channels() != 4)
    {
        __android_log_print(ANDROID_LOG_DEBUG, "camera_base_jni", "Aborting frame delivery due to unsupported Mat format");
        return;
    }

    ANativeWindow_Buffer nativeWindowBuffer;
    ANativeWindow_lock(window, &nativeWindowBuffer, NULL);

    //__android_log_print(ANDROID_LOG_DEBUG, "Native", "Native buffer format %d [%dx%d] stride %d\n",
    //        nativeWindowBuffer.format, nativeWindowBuffer.width, nativeWindowBuffer.height, nativeWindowBuffer.stride);

    if(nativeWindowBuffer.format != WINDOW_FORMAT_RGBA_8888)
    {
        __android_log_print(ANDROID_LOG_DEBUG, "camera_base_jni", "Aborting frame delivery due to surface format not being RGBA_8888");
        goto exit;
    }

    /*
     * If the stride of the encoder buffer is the same as the
     * width of our Mat buffer, our life is easy, we can just
     * do the copy in one go.
     */
    if(nativeWindowBuffer.stride == mat->cols)
    {
        memcpy(nativeWindowBuffer.bits, mat->data, mat->cols*mat->rows*4);
    }
    /*
     * Ugh, the stride doesn't match the width. So, we have to do
     * a memcpy for each line of the source image, padding the dst
     * pointer with the difference between the stride and width.
     */
    else
    {
        int width = mat->cols;
        int height = mat->rows;
        int bitsPerPixel = 4;
        int scanline_length = width*bitsPerPixel;
        int scanline_end_padding = (nativeWindowBuffer.stride - width) * bitsPerPixel;

        for(int i = 0; i < height; i++)
        {
            memcpy((uint8_t*)nativeWindowBuffer.bits + i*(scanline_length+scanline_end_padding), mat->data+(i*scanline_length), scanline_length);
        }
    }

    exit:
    ANativeWindow_unlockAndPost(window);
}

void setAlphaChannelTo255(Mat* mat)
{
    uint8_t* endOfBuff = mat->data + (mat->rows * mat->cols * 4);
    uint8_t* startOfBuff = mat->data+3;
    for(uint8_t* alphaPtr = startOfBuff; alphaPtr <= endOfBuff; alphaPtr+=4)
    {
        *alphaPtr = 255;
    }
}

extern "C" JNIEXPORT jlong JNICALL
Java_org_openftc_easyopencv_OpenCvCameraBase_nativeGetSurfaceHandle(JNIEnv *env, jobject thiz, jobject surface)
{
    ANativeWindow* nativeWindow = ANativeWindow_fromSurface(env, surface);
    return (jlong) nativeWindow;
}

extern "C" JNIEXPORT void JNICALL
Java_org_openftc_easyopencv_OpenCvCameraBase_nativeReleaseSurfaceHandle(JNIEnv *env, jobject thiz, jlong handle)
{
    ANativeWindow_release((ANativeWindow*)handle);
}