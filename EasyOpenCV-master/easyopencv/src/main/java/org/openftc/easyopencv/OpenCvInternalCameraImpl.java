/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Note: credit where credit is due - some parts of OpenCv's
 *       JavaCameraView were used as a reference
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

import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

class OpenCvInternalCameraImpl extends OpenCvCameraBase implements Camera.PreviewCallback, OpenCvInternalCamera
{
    private Camera camera;
    private OpenCvInternalCamera.CameraDirection direction;
    private Mat rawSensorMat;
    private Mat rgbMat;
    private SurfaceTexture bogusSurfaceTexture;
    private int maxZoom = -1;

    private volatile boolean isStreaming = false;

    public OpenCvInternalCameraImpl(OpenCvInternalCamera.CameraDirection direction)
    {
        this.direction = direction;
    }

    public OpenCvInternalCameraImpl(OpenCvInternalCamera.CameraDirection direction, int containerLayoutId)
    {
        super(containerLayoutId);
        this.direction = direction;
    }

    @Override
    public OpenCvCameraRotation getDefaultRotation()
    {
        return OpenCvCameraRotation.UPRIGHT;
    }

    @Override
    protected int mapRotationEnumToOpenCvRotateCode(OpenCvCameraRotation rotation)
    {
        /*
         * The camera sensor in a phone is mounted sideways, such that the raw image
         * is only upright when the phone is rotated to the left. Therefore, we need
         * to manually rotate the image if the phone is in any other orientation
         */

        if(direction == OpenCvInternalCamera.CameraDirection.BACK)
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
        else if(direction == OpenCvInternalCamera.CameraDirection.FRONT)
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
        return true;
    }

    @Override
    protected boolean isStreaming()
    {
        return isStreaming;
    }

    @Override
    public synchronized int openCameraDevice()
    {
        if(hasBeenCleanedUp())
        {
            return CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE;// We're running on a zombie thread post-mortem of the OpMode GET OUT OF DODGE NOW
        }

        try
        {
            if(camera == null)
            {
                camera = Camera.open(direction.id);
            }

            return 0;
        }
        catch (Exception e)
        {
            e.printStackTrace();
            return CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE;
        }
    }

    @Override
    public void openCameraDeviceAsync(final AsyncCameraOpenListener asyncCameraOpenListener)
    {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                synchronized (OpenCvInternalCameraImpl.this)
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

        if(camera != null)
        {
            stopStreaming();
            camera.stopPreview();
            camera.release();
            camera = null;
        }
    }

    @Override
    public void closeCameraDeviceAsync(final AsyncCameraCloseListener asyncCameraCloseListener)
    {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                synchronized (OpenCvInternalCameraImpl.this)
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
        startStreaming(width, height, rotation, BufferMethod.DOUBLE);
    }

    @Override
    public synchronized void startStreaming(int width, int height, OpenCvCameraRotation rotation, BufferMethod bufferMethod)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("startStreaming() called, but camera is not opened!");
        }

        /*
         * If we're already streaming, then that's OK, but we need to stop
         * streaming in the old mode before we can restart in the new one.
         */
        if(isStreaming)
        {
            stopStreaming();
        }

        /*
         * Prep the viewport
         */
        prepareForStartStreaming(width, height, rotation);

        rawSensorMat = new Mat(height + (height/2), width, CvType.CV_8UC1);
        rgbMat = new Mat(height + (height/2), width, CvType.CV_8UC1);

        if(camera != null)
        {
            Camera.Parameters parameters = camera.getParameters();
            parameters.setPreviewFormat(ImageFormat.NV21);
            parameters.setPreviewSize(width, height);

            /*
             * Not all cameras support all focus modes...
             */
            if(parameters.getSupportedFocusModes().contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO))
            {
                parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_VIDEO);
            }
            else if(parameters.getSupportedFocusModes().contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE))
            {
                parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);
            }
            else if(parameters.getSupportedFocusModes().contains(Camera.Parameters.FOCUS_MODE_FIXED))
            {
                parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_FIXED);
            }

            boolean isRequestedSizeSupported = false;

            List<Camera.Size> cameraSupportedPreviewSizes = parameters.getSupportedPreviewSizes();

            for(Camera.Size size : cameraSupportedPreviewSizes)
            {
                if(size.width == width && size.height == height)
                {
                    isRequestedSizeSupported = true;
                    break;
                }
            }

            if(!isRequestedSizeSupported)
            {
                StringBuilder supportedSizesBuilder = new StringBuilder();

                for(Camera.Size s : cameraSupportedPreviewSizes)
                {
                    supportedSizesBuilder.append(String.format("[%dx%d], ", s.width, s.height));
                }

                throw new OpenCvCameraException("Camera does not support requested resolution! Supported resolutions are " + supportedSizesBuilder.toString());
            }

            maxZoom = parameters.getMaxZoom();
            camera.setParameters(parameters);

            int pixels = width * height;
            int bufSize  = pixels * ImageFormat.getBitsPerPixel(parameters.getPreviewFormat()) / 8;

            bogusSurfaceTexture = new SurfaceTexture(10);

            camera.setPreviewCallbackWithBuffer(this);

            if(bufferMethod == BufferMethod.SINGLE)
            {
                //One buffer
                camera.addCallbackBuffer(new byte[bufSize]);
            }
            else if(bufferMethod == BufferMethod.DOUBLE)
            {
                //Two buffers
                camera.addCallbackBuffer(new byte[bufSize]);
                camera.addCallbackBuffer(new byte[bufSize]);
            }
            else
            {
                throw new IllegalArgumentException("Illegal buffer method!");
            }

            try
            {
                camera.setPreviewTexture(bogusSurfaceTexture);
            }
            catch (IOException e)
            {
                e.printStackTrace();
                //closeCameraDevice();
                return;
            }

            camera.startPreview();
            isStreaming = true;
        }
    }

    @Override
    public synchronized void stopStreaming()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("stopStreaming() called, but camera is not opened!");
        }

        cleanupForEndStreaming();

        maxZoom = -1;

        if(camera != null)
        {
            camera.setPreviewCallback(null);
            camera.stopPreview();
        }

        if(rawSensorMat != null)
        {
            rawSensorMat.release();
            rawSensorMat = null;
        }

        if(rgbMat != null)
        {
            rgbMat.release();
            rgbMat = null;
        }

        isStreaming = false;
    }

    /*
     * This needs to be synchronized with stopStreamingImplSpecific()
     * because we touch objects that are destroyed in that method.
     */
    @Override
    public synchronized void onPreviewFrame(byte[] data, Camera camera)
    {
        long callbackTimestamp = System.nanoTime();

        notifyStartOfFrameProcessing();

        /*
         * Unfortunately, we can't easily create a Java byte[] that
         * references the native memory in a Mat, so we have to do
         * a memcpy from our Java byte[] to the native one in the Mat.
         * (If we could, then we could have the camera dump the preview
         * image directly into the Mat).
         *
         * TODO: investigate using a bit of native code to remove the need to do a memcpy
         */
        if(rawSensorMat != null)
        {
            rawSensorMat.put(0,0,data);

            Imgproc.cvtColor(rawSensorMat, rgbMat, Imgproc.COLOR_YUV2RGBA_NV21, 4);
            handleFrame(rgbMat, callbackTimestamp);

            if(camera != null)
            {
                camera.addCallbackBuffer(data);
            }
        }
    }

    @Override
    public synchronized void setFocusMode(FocusMode focusMode)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot set focus mode until camera is opened!");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();

            ArrayList<FocusMode> supportedModes = getEocvCameraApiFocusModes(parameters.getSupportedFocusModes());

            if(!supportedModes.contains(focusMode))
            {
                StringBuilder supportedSizesBuilder = new StringBuilder();

                for(FocusMode f : supportedModes)
                {
                    supportedSizesBuilder.append(String.format("%s, ", f.toString()));
                }

                throw new OpenCvCameraException(String.format("Focus mode %s is not supported on this camera. Supported focus modes are %s", focusMode.toString(), supportedSizesBuilder.toString()));
            }
            else
            {
                parameters.setFocusMode(focusMode.android_string);
            }
        }
    }

    private ArrayList<FocusMode> getEocvCameraApiFocusModes(List<String> focusModes)
    {
        ArrayList<FocusMode> ret = new ArrayList<>(focusModes.size());

        for(String s : focusModes)
        {
            FocusMode m = FocusMode.fromAndroidString(s);

            if(m != null)
            {
                ret.add(m);
            }
        }

        return ret;
    }

    @Override
    public synchronized void setFlashlightEnabled(boolean enabled)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot control flash until camera is opened!");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();

            List<String> supportedFlashModes = parameters.getSupportedFlashModes();

            if(supportedFlashModes == null)
            {
                throw new OpenCvCameraException("Camera does not have a flash!");
            }
            else if(!supportedFlashModes.contains(Camera.Parameters.FLASH_MODE_TORCH))
            {
                throw new OpenCvCameraException("Camera flash does not support torch mode!");
            }

            if(enabled)
            {
                parameters.setFlashMode(Camera.Parameters.FLASH_MODE_TORCH);
            }
            else
            {
                parameters.setFlashMode(Camera.Parameters.FLASH_MODE_OFF);
            }

            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized void setExposureLocked(boolean lock)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot lock exposure until camera is opened");
        }

        Camera.Parameters parameters = camera.getParameters();

        if(!parameters.isAutoExposureLockSupported())
        {
            throw new OpenCvCameraException("Locking exposure is not supported on this camera");
        }

        parameters.setAutoExposureLock(lock);
        camera.setParameters(parameters);
    }

    @Override
    public synchronized void setExposureCompensation(int exposureCompensation)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot set exposure compensation until camera is opened!");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();

            int minExposureCompensation = parameters.getMinExposureCompensation();
            int maxExposureCompensation = parameters.getMaxExposureCompensation();

            if(exposureCompensation > maxExposureCompensation)
            {
                throw new OpenCvCameraException(String.format("Exposure compensation value of %d requested, but max supported compensation is %d", exposureCompensation, maxExposureCompensation));
            }
            else if(exposureCompensation < minExposureCompensation)
            {
                throw new OpenCvCameraException(String.format("Exposure compensation value of %d requested, but min supported compensation is %d", exposureCompensation, minExposureCompensation));
            }

            parameters.setExposureCompensation(exposureCompensation);
            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized int getMaxSupportedExposureCompensation()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot get max supported exposure compensation until camera is opened");
        }
        else
        {
            return camera.getParameters().getMaxExposureCompensation();
        }
    }

    @Override
    public synchronized int getMinSupportedExposureCompensation()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot get min supported exposure compensation until camera is opened");
        }
        else
        {
            return camera.getParameters().getMinExposureCompensation();
        }
    }

    @Override
    public synchronized int getMaxSupportedZoom()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot get supported zooms until camera is opened and streaming is started");
        }
        else
        {
            if(maxZoom == -1)
            {
                throw new OpenCvCameraException("Cannot get supported zooms until streaming has been started");
            }

            return maxZoom;
        }
    }

    @Override
    public synchronized void setZoom(int zoom)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot set zoom until camera is opened and streaming is started");
        }
        else
        {
            if(maxZoom == -1)
            {
                throw new OpenCvCameraException("Cannot set zoom until streaming has been started");
            }
            else if(zoom > maxZoom)
            {
                throw new OpenCvCameraException(String.format("Zoom value of %d requested, but maximum zoom supported in current configuration is %d", zoom, maxZoom));
            }
            else if(zoom < 0)
            {
                throw new OpenCvCameraException("Zoom value cannot be less than 0");
            }
            Camera.Parameters parameters = camera.getParameters();
            parameters.setZoom(zoom);
            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized void setRecordingHint(boolean hint)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot set recording hint until camera is opened");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();
            parameters.setRecordingHint(hint);
            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized void setHardwareFrameTimingRange(FrameTimingRange frameTiming)
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot set hardware frame timing range until camera is opened");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();
            parameters.setPreviewFpsRange(frameTiming.min*1000, frameTiming.max*1000);
            camera.setParameters(parameters);
        }
    }

    @Override
    public synchronized FrameTimingRange[] getFrameTimingRangesSupportedByHardware()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("Cannot get frame timing ranges until camera is opened");
        }
        else
        {
            Camera.Parameters parameters = camera.getParameters();
            List<int[]> rawRanges = parameters.getSupportedPreviewFpsRange();
            FrameTimingRange[] ranges = new FrameTimingRange[rawRanges.size()];

            for(int i = 0; i < ranges.length; i++)
            {
                int[] raw = rawRanges.get(i);
                ranges[i] = new FrameTimingRange(raw[0]/1000, raw[1]/1000);
            }

            return ranges;
        }
    }
}
