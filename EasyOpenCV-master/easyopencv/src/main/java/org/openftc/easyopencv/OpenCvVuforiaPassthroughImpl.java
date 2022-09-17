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

package org.openftc.easyopencv;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.nio.ByteBuffer;
import java.util.concurrent.BlockingQueue;

class OpenCvVuforiaPassthroughImpl extends OpenCvCameraBase
{
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
    private FrameCruncherThread frameCruncherThread;
    private long ptrNativeExtSourceRawSensorMat;
    private Mat rgbMat;
    private OpenCvCameraRotation rotation;
    private boolean isWebcam;

    public OpenCvVuforiaPassthroughImpl(VuforiaLocalizer vuforiaLocalizer, VuforiaLocalizer.Parameters parameters)
    {
        this.vuforiaLocalizer = vuforiaLocalizer;
        isWebcam = parameters.cameraName instanceof WebcamName;
        this.parameters = parameters;
    }

    public OpenCvVuforiaPassthroughImpl(VuforiaLocalizer vuforiaLocalizer, VuforiaLocalizer.Parameters parameters, int viewport)
    {
        super(viewport);
        this.vuforiaLocalizer = vuforiaLocalizer;
        isWebcam = parameters.cameraName instanceof WebcamName;
        this.parameters = parameters;
    }

    @Override
    protected OpenCvCameraRotation getDefaultRotation()
    {
        return OpenCvCameraRotation.UPRIGHT;
    }

    @Override
    protected int mapRotationEnumToOpenCvRotateCode(OpenCvCameraRotation rotation)
    {
        if(isWebcam)
        {
            /*
             * The camera sensor in a webcam is mounted in the logical manner, such
             * that the raw image is upright when the webcam is used in its "normal"
             * orientation. However, if the user is using it in any other orientation,
             * we need to manually rotate the image.
             */

            if(rotation == OpenCvCameraRotation.SIDEWAYS_LEFT)
            {
                return Core.ROTATE_90_COUNTERCLOCKWISE;
            }
            if(rotation == OpenCvCameraRotation.SIDEWAYS_RIGHT)
            {
                return Core.ROTATE_90_CLOCKWISE;
            }
            else if(rotation == OpenCvCameraRotation.UPSIDE_DOWN)
            {
                return Core.ROTATE_180;
            }
            else
            {
                return -1;
            }
        }
        /*
         * The camera sensor in a phone is mounted sideways, such that the raw image
         * is only upright when the phone is rotated to the left. Therefore, we need
         * to manually rotate the image if the phone is in any other orientation
         */
        else if(parameters.cameraDirection == VuforiaLocalizer.CameraDirection.BACK)
        {
            if(rotation == OpenCvCameraRotation.UPRIGHT)
            {
                return Core.ROTATE_90_CLOCKWISE;
            }
            else if(rotation == OpenCvCameraRotation.UPSIDE_DOWN)
            {
                return Core.ROTATE_90_COUNTERCLOCKWISE;
            }
            else if(rotation == OpenCvCameraRotation.SIDEWAYS_RIGHT)
            {
                return Core.ROTATE_180;
            }
            else
            {
                return -1;
            }
        }
        else if(parameters.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT)
        {
            if(rotation == OpenCvCameraRotation.UPRIGHT)
            {
                return Core.ROTATE_90_COUNTERCLOCKWISE;
            }
            else if(rotation == OpenCvCameraRotation.UPSIDE_DOWN)
            {
                return Core.ROTATE_90_CLOCKWISE;
            }
            else if(rotation == OpenCvCameraRotation.SIDEWAYS_RIGHT)
            {
                return Core.ROTATE_180;
            }
            else
            {
                return -1;
            }
        }

        return -1;
    }

    @Override
    protected boolean cameraOrientationIsTiedToDeviceOrientation()
    {
        return !isWebcam;
    }

    @Override
    protected synchronized boolean isStreaming()
    {
        return frameQueue != null;
    }

    @Override
    public synchronized int openCameraDevice()
    {
        // nothing to do really
        return 0;
    }

    @Override
    public void openCameraDeviceAsync(final AsyncCameraOpenListener asyncCameraOpenListener)
    {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                synchronized (OpenCvVuforiaPassthroughImpl.this)
                {
                    try
                    {
                        int retCode = openCameraDevice();

                        if(retCode < 0)
                        {
                            asyncCameraOpenListener.onError(retCode);
                        }
                        else
                        {
                            asyncCameraOpenListener.onOpened();
                        }
                    }
                    catch (Exception e)
                    {
                        if(!hasBeenCleanedUp())
                        {
                            emulateEStop(e);
                        }
                        else
                        {
                            e.printStackTrace();
                        }

                    }
                }
            }
        }).start();
    }

    @Override
    public synchronized void closeCameraDevice()
    {
        cleanupForClosingCamera();
    }

    @Override
    public void closeCameraDeviceAsync(final AsyncCameraCloseListener asyncCameraCloseListener)
    {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                synchronized (OpenCvVuforiaPassthroughImpl.this)
                {
                    try
                    {
                        closeCameraDevice();
                        asyncCameraCloseListener.onClose();
                    }
                    catch (Exception e)
                    {
                        if(!hasBeenCleanedUp())
                        {
                            emulateEStop(e);
                        }
                        else
                        {
                            e.printStackTrace();
                        }
                    }
                }
            }
        }).start();
    }

    @Override
    public synchronized void startStreaming(int width, int height)
    {
        startStreaming(width, height, getDefaultRotation());
    }

    @Override
    public synchronized void startStreaming(int width, int height, OpenCvCameraRotation rotation)
    {
        if(frameQueue != null)
        {
            return;
        }

        this.rotation = rotation;

        if(isWebcam)
        {
            boolean worked = Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

            if(!worked)
            {
                throw new OpenCvCameraException("Failed to convince Vuforia to generate RGB565 frames");
            }
        }
        else
        {
            boolean worked = Vuforia.setFrameFormat(PIXEL_FORMAT.YUV, true);

            if(!worked)
            {
                throw new OpenCvCameraException("Failed to convince Vuforia to generate YUV frames");
            }
        }

        vuforiaLocalizer.setFrameQueueCapacity(1);
        frameQueue = vuforiaLocalizer.getFrameQueue();
        frameCruncherThread = new FrameCruncherThread();
        frameCruncherThread.start();
    }

    @Override
    public synchronized void stopStreaming()
    {
        if(frameQueue != null)
        {
            cleanupForEndStreaming();

            frameCruncherThread.interrupt();
            joinUninterruptibly(frameCruncherThread);

            if(ptrNativeExtSourceRawSensorMat != 0)
            {
                freeExtImgDatMat(ptrNativeExtSourceRawSensorMat);
                ptrNativeExtSourceRawSensorMat = 0;
            }

            rgbMat = null;
            frameQueue = null;
        }
    }

    public synchronized void handleNewFrame(VuforiaLocalizer.CloseableFrame frame)
    {
        long captureTime = System.nanoTime();
        notifyStartOfFrameProcessing();

        Image vuforiaImage = null;

        for(int i = 0; i < frame.getNumImages(); i++)
        {
            Image image = frame.getImage(i);

            if(!isWebcam && image.getFormat() == PIXEL_FORMAT.YUV)
            {
                vuforiaImage = image;
                break;
            }
            else if(isWebcam && image.getFormat() == PIXEL_FORMAT.RGB565) // YUV, RGB888, and RGB8888 all fail :(
            {
                vuforiaImage = image;
                break;
            }
        }

        if(vuforiaImage != null)
        {
            if(rgbMat == null)
            {
                prepareForStartStreaming(vuforiaImage.getWidth(), vuforiaImage.getHeight(), rotation);

                rgbMat = new Mat(vuforiaImage.getHeight(), vuforiaImage.getWidth(), CvType.CV_8UC1);
            }
            if(ptrNativeExtSourceRawSensorMat == 0)
            {
                if(isWebcam)
                {
                    ptrNativeExtSourceRawSensorMat = allocExtImgDatMat(vuforiaImage.getWidth(), vuforiaImage.getHeight(),  CvType.CV_8UC2);
                }
                else
                {
                    ptrNativeExtSourceRawSensorMat = allocExtImgDatMat(vuforiaImage.getWidth(), vuforiaImage.getHeight() + vuforiaImage.getHeight()/2,  CvType.CV_8UC1);
                }
            }

            setMatDataPtr(vuforiaImage.getPixels(), ptrNativeExtSourceRawSensorMat);

            if(isWebcam)
            {
                colorConversionWebcam(ptrNativeExtSourceRawSensorMat, rgbMat.nativeObj);
            }
            else
            {
                colorConversionInternalCamera(ptrNativeExtSourceRawSensorMat, rgbMat.nativeObj);
            }


            // We copied the data, so we can close the original frame
            frame.close();

            handleFrame(rgbMat, captureTime);
        }
    }

    class FrameCruncherThread extends Thread
    {
        @Override
        public void run()
        {

            while (!Thread.currentThread().isInterrupted())
            {
                try
                {
                    VuforiaLocalizer.CloseableFrame frame = frameQueue.take();
                    handleNewFrame(frame);
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    private void joinUninterruptibly(Thread thread)
    {
        boolean interrupted = false;

        while (true)
        {
            try
            {
                thread.join();
                break;
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
                interrupted = true;
            }
        }

        if(interrupted)
        {
            Thread.currentThread().interrupt();
        }
    }

    native static void setMatDataPtr(ByteBuffer pixles, long rawSensorMatPtr);
    public static native long allocExtImgDatMat(int width, int height, int type);
    public static native void freeExtImgDatMat(long ptr);
    public static native void colorConversionInternalCamera(long rawDataPtr, long rgbPtr);
    public static native void colorConversionWebcam(long rawDataPtr, long rgbPtr);
}
