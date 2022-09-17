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

package org.openftc.easyopencv;

import android.annotation.SuppressLint;
import android.content.Context;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.Image;
import android.media.ImageReader;
import android.os.Handler;
import android.util.Log;
import android.util.Size;
import android.view.Surface;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.locks.ReentrantLock;

@SuppressLint({"NewApi", "MissingPermission"})
public class OpenCvInternalCamera2Impl extends OpenCvCameraBase implements OpenCvInternalCamera2, ImageReader.OnImageAvailableListener
{
    CameraDevice mCameraDevice;

    FixedHandlerThread cameraHardwareHandlerThread;
    private Handler cameraHardwareHandler;

    FixedHandlerThread frameWorkerHandlerThread;
    Handler frameWorkerHandler;

    volatile CountDownLatch cameraOpenedLatch;
    volatile CountDownLatch streamingStartedLatch;
    ImageReader imageReader;
    CaptureRequest.Builder mPreviewRequestBuilder;
    CameraCaptureSession cameraCaptureSession;
    long ptrNativeContext;
    Mat rgbMat;
    OpenCvInternalCamera2.CameraDirection direction;
    public float exposureTime = 1/50f;
    private volatile boolean isStreaming = false;
    Surface surface;
    CameraManager cameraManager;
    CameraCharacteristics cameraCharacteristics;

    ReentrantLock sync = new ReentrantLock();
    final static String TAG = "OpenCvInternalCamera2";

    private boolean sensorTimestampsAreRealtime = false;

    @SuppressLint("WrongConstant")
    public OpenCvInternalCamera2Impl(OpenCvInternalCamera2.CameraDirection direction)
    {
        this.direction = direction;
        cameraManager = (CameraManager) AppUtil.getInstance().getActivity().getSystemService(Context.CAMERA_SERVICE);
    }

    @SuppressLint("WrongConstant")
    public OpenCvInternalCamera2Impl(OpenCvInternalCamera2.CameraDirection direction, int containerLayoutId)
    {
        super(containerLayoutId);
        this.direction = direction;
        cameraManager = (CameraManager) AppUtil.getInstance().getActivity().getSystemService(Context.CAMERA_SERVICE);
    }

    @Override
    protected OpenCvCameraRotation getDefaultRotation()
    {
        return OpenCvCameraRotation.UPRIGHT;
    }

    @Override
    protected int mapRotationEnumToOpenCvRotateCode(OpenCvCameraRotation rotation)
    {
        if(direction == CameraDirection.BACK)
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
        else
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
    public int openCameraDevice()
    {
        sync.lock();

        if(hasBeenCleanedUp())
        {
            return CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE; // We're running on a zombie thread post-mortem of the OpMode GET OUT OF DODGE NOW
        }

        if(mCameraDevice == null)
        {
            try
            {
                startCameraHardwareHandlerThread();

                String camList[] = cameraManager.getCameraIdList();

                String camId = null;

                for(String s : camList)
                {
                    if(cameraManager.getCameraCharacteristics(s).get(CameraCharacteristics.LENS_FACING) == direction.id)
                    {
                        camId = s;
                        break;
                    }
                }

                cameraOpenedLatch = new CountDownLatch(1);
                cameraCharacteristics = cameraManager.getCameraCharacteristics(camId);
                cameraManager.openCamera(camId, mStateCallback, cameraHardwareHandler);
                sensorTimestampsAreRealtime = cameraCharacteristics.get(CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE) == CameraCharacteristics.SENSOR_INFO_TIMESTAMP_SOURCE_REALTIME;
                RobotLog.vv("OpenCvInternalCamera2Impl", "Camera sensor timestamps are realtime: %b", sensorTimestampsAreRealtime);

                cameraOpenedLatch.await();
            }
            catch (CameraAccessException e)
            {
                e.printStackTrace();
            }
            catch (InterruptedException e)
            {
                e.printStackTrace();
                Thread.currentThread().interrupt();
            }
        }

        int ret = mCameraDevice != null ? 0 : CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE;
        sync.unlock();

        return ret;
    }

    @Override
    public void openCameraDeviceAsync(final AsyncCameraOpenListener asyncCameraOpenListener)
    {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                sync.lock();

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
                finally
                {
                    sync.unlock();
                }
            }
        }).start();
    }

    @Override
    public void closeCameraDevice()
    {
        sync.lock();

        cleanupForClosingCamera();

        if(mCameraDevice != null)
        {
            stopStreaming();

            mCameraDevice.close();
            stopCameraHardwareHandlerThread();
            mCameraDevice = null;
        }

        sync.unlock();
    }

    @Override
    public void closeCameraDeviceAsync(final AsyncCameraCloseListener asyncCameraCloseListener)
    {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                sync.lock();

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
                finally
                {
                    sync.unlock();
                }
            }
        }).start();
    }

    @Override
    public void startStreaming(int width, int height)
    {
        startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
    }

    @Override
    public void startStreaming(int width, int height, OpenCvCameraRotation rotation)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
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

            prepareForStartStreaming(width, height, rotation);

            StreamConfigurationMap streamConfigurationMap = cameraCharacteristics.get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            Size[] sizes = streamConfigurationMap.getOutputSizes(ImageFormat.YUV_420_888);

            boolean isRequestedSizeSupported = false;

            for(Size size : sizes)
            {
                if(size.getWidth() == width && size.getHeight() == height)
                {
                    isRequestedSizeSupported = true;
                    break;
                }
            }

            if(!isRequestedSizeSupported)
            {
                StringBuilder supportedSizesBuilder = new StringBuilder();

                for(Size s : sizes)
                {
                    supportedSizesBuilder.append(String.format("[%dx%d], ", s.getWidth(), s.getHeight()));
                }

                throw new OpenCvCameraException("Camera does not support requested resolution! Supported resolutions are " + supportedSizesBuilder.toString());
            }

            rgbMat = new Mat(height, width, CvType.CV_8UC3);

            startFrameWorkerHandlerThread();

            mPreviewRequestBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);

            imageReader = ImageReader.newInstance(width, height, ImageFormat.YUV_420_888, 2);
            imageReader.setOnImageAvailableListener(this, frameWorkerHandler);
            surface = imageReader.getSurface();
            mPreviewRequestBuilder.addTarget(surface);

            streamingStartedLatch = new CountDownLatch(1);


            CameraCaptureSession.StateCallback callback = new CameraCaptureSession.StateCallback()
            {
                @Override
                public void onConfigured( CameraCaptureSession session)
                {
                    try
                    {
                        if (null == mCameraDevice)
                        {
                            return; // camera is already closed
                        }
                        cameraCaptureSession = session;


                        cameraCaptureSession.setRepeatingRequest(mPreviewRequestBuilder.build(), null, cameraHardwareHandler);
                    }
                    catch (CameraAccessException e)
                    {
                        e.printStackTrace();
                    }
                    finally
                    {
                        streamingStartedLatch.countDown();
                    }
                }

                @Override
                public void onConfigureFailed( CameraCaptureSession session)
                {
                }
            };

            mCameraDevice.createCaptureSession(Arrays.asList(surface), callback, cameraHardwareHandler);

            streamingStartedLatch.await();

            isStreaming = true;
        }
        catch (CameraAccessException e)
        {
            e.printStackTrace();
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
            Thread.currentThread().interrupt();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void stopStreaming()
    {
        Log.d(TAG, "stopStreaming()");

        sync.lock();

        cleanupForEndStreaming();

        try
        {
            if (null != cameraCaptureSession)
            {
                cameraCaptureSession.close();
                cameraCaptureSession = null;
                isStreaming = false;
            }
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }

        try
        {
            //stopCameraHardwareHandlerThread();
            //stopFrameWorkerHandlerThread();
            if (null != imageReader)
            {
                imageReader.close();
                imageReader = null;

                stopFrameWorkerHandlerThread();

                // Only release the native context after the frame worker has stopped.
                // That guarantees that onPreviewFrame() won't be called again. Although
                // since onPreviewFrame() is called with 'sync' held, I guess we could do
                // it earlier up. But here seems like a logical place. *shrug*
                if(ptrNativeContext != 0)
                {
                    Log.d(TAG, "Releasing native context");
                    releaseNativeContext(ptrNativeContext);
                    ptrNativeContext = 0;
                }
            }
        }
        catch (Exception e)
        {
            e.printStackTrace();
        }
        finally
        {
            sync.unlock();
            Log.d(TAG, "...stopStreaming()");
        }
    }

    private final CameraDevice.StateCallback mStateCallback = new CameraDevice.StateCallback()
    {
        @Override
        public void onOpened(CameraDevice cameraDevice)
        {
            mCameraDevice = cameraDevice;
            cameraOpenedLatch.countDown();
        }

        @Override
        public void onDisconnected(CameraDevice cameraDevice)
        {
            cameraDevice.close();
            mCameraDevice = null;
        }

        @Override
        public void onError(CameraDevice cameraDevice, int error)
        {
            cameraDevice.close();
            mCameraDevice = null;
            cameraOpenedLatch.countDown();
        }

    };

    /* CALLED WITH 'sync' held!! */
    private void onPreviewFrame(Image image, long callbackTimestamp)
    {
        notifyStartOfFrameProcessing();

        if(ptrNativeContext == 0)
        {
            Log.d(TAG, "Creating native context");
            ptrNativeContext = createNativeContext(image.getWidth(), image.getHeight());
        }

        Image.Plane[] planes = image.getPlanes();
        colorConversion(planes[0].getBuffer(), planes[1].getBuffer(), planes[2].getBuffer(), ptrNativeContext, rgbMat.nativeObj);
        long imgTimestamp = image.getTimestamp();
        image.close();

        handleFrame(rgbMat, sensorTimestampsAreRealtime ? imgTimestamp : callbackTimestamp);
    }

    private void startFrameWorkerHandlerThread()
    {

        sync.lock();

        frameWorkerHandlerThread = new FixedHandlerThread("FrameWorkerHandlerThread");
        frameWorkerHandlerThread.start();

        frameWorkerHandler = new Handler(frameWorkerHandlerThread.getLooper());

        sync.unlock();
    }

    private void stopFrameWorkerHandlerThread()
    {
        sync.lock();

        if (frameWorkerHandlerThread != null)
        {
            frameWorkerHandlerThread.quit();
            frameWorkerHandlerThread.interrupt();
            joinUninterruptibly(frameWorkerHandlerThread);

            frameWorkerHandlerThread = null;
            frameWorkerHandler = null;
        }

        sync.unlock();
    }

    private void startCameraHardwareHandlerThread()
    {
        sync.lock();

        cameraHardwareHandlerThread = new FixedHandlerThread("CameraHardwareHandlerThread");
        cameraHardwareHandlerThread.start();
        cameraHardwareHandler = new Handler(cameraHardwareHandlerThread.getLooper());

        sync.unlock();
    }

    private void stopCameraHardwareHandlerThread()
    {
        sync.lock();

        if (cameraHardwareHandlerThread == null)
            return;

        cameraHardwareHandlerThread.quitSafely();
        cameraHardwareHandlerThread.interrupt();

        joinUninterruptibly(cameraHardwareHandlerThread);
        cameraHardwareHandlerThread = null;
        cameraHardwareHandler = null;

        sync.unlock();
    }

    @Override
    public void onImageAvailable(ImageReader reader)
    {
        long callbackTimestamp = System.nanoTime();

        try
        {
            /*
             * Note: this it is VERY important that we lock
             * interruptibly, because otherwise we can get
             * into a deadlock with the OpMode thread where
             * it's waiting for us to exit, but we can't exit
             * because we're waiting on this lock which the OpMode
             * thread is holding!!!
             */
            sync.lockInterruptibly();

            try
            {
                Image image = reader.acquireLatestImage();

                if(image != null)
                {
                    /*
                     * For some reason, when we restart the streaming while live,
                     * the image returned from the image reader is somehow
                     * already closed and so onPreviewFrame() dies. Therefore,
                     * until we can figure out what's going on here, we simply
                     * catch and ignore the exception.
                     */
                    try
                    {
                        onPreviewFrame(image, callbackTimestamp);
                    }
                    catch (IllegalStateException e)
                    {
                        e.printStackTrace();
                    }

                    image.close();
                }
            }
            finally
            {
                sync.unlock();
            }
        }
        catch (InterruptedException e)
        {
            e.printStackTrace();
            Thread.currentThread().interrupt();
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

    @Override
    public int getMinSensorGain()
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("getMinSensorGain() called, but camera is not opened!");
            }

            return cameraCharacteristics.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE).getLower();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public int getMaxSensorGain()
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("getMinSensorGain() called, but camera is not opened!");
            }

            return cameraCharacteristics.get(CameraCharacteristics.SENSOR_INFO_SENSITIVITY_RANGE).getUpper();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setSensorGain(int iso)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setSensorGain() called, but camera is not opened!");
            }

            mPreviewRequestBuilder.set(CaptureRequest.SENSOR_SENSITIVITY, iso);
            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setWhiteBalanceMode(WhiteBalanceMode whiteBalanceMode)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setWhiteBalanceMode() called, but camera is not opened!");
            }

            switch (whiteBalanceMode)
            {
                case AUTO:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_AUTO);
                    break;

                case MANUAL:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_OFF);
                    break;

                case SHADE:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_SHADE);
                    break;

                case DAYLIGHT:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_DAYLIGHT);
                    break;

                case TWILIGHT:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_TWILIGHT);
                    break;

                case FLUORESCENT:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_FLUORESCENT);
                    break;

                case INCANDESCENT:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_INCANDESCENT);
                    break;

                case CLOUDY_DAYLIGHT:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_CLOUDY_DAYLIGHT);
                    break;

                case WARM_FLUORESCENT:
                    mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_MODE, CaptureRequest.CONTROL_AWB_MODE_WARM_FLUORESCENT);
                    break;
            }

            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setAutoWhiteBalanceLocked(boolean locked)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setAutoWhiteBalanceLocked() called, but camera is not opened!");
            }

            mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AWB_LOCK, locked);

            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setExposureMode(ExposureMode exposureMode)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setExposureMode() called, but camera is not opened!");
            }

            if(exposureMode == ExposureMode.MANUAL)
            {
                mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_OFF);
            }
            else if(exposureMode == ExposureMode.AUTO)
            {
                mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
            }

            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setAutoExposureLocked(boolean locked)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setAutoExposureLocked() called, but camera is not opened!");
            }

            mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AE_LOCK, locked);

            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setSensorFps(int sensorFps)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setSensorFps() called, but camera is not opened!");
            }

            long nanos = (long) ((1.0/sensorFps)*1e9);

            mPreviewRequestBuilder.set(CaptureRequest.SENSOR_FRAME_DURATION, nanos);

            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setFlashlightEnabled(boolean enabled)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setSensorFps() called, but camera is not opened!");
            }

            if(enabled)
            {
                mPreviewRequestBuilder.set(CaptureRequest.FLASH_MODE, CaptureRequest.FLASH_MODE_TORCH);
            }
            else
            {
                mPreviewRequestBuilder.set(CaptureRequest.FLASH_MODE, CaptureRequest.FLASH_MODE_OFF);
            }

            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public int getMinAutoExposureCompensation()
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("getMinAutoExposureCompensation() called, but camera is not opened!");
            }

            return cameraCharacteristics.get(CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE).getLower();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public int getMaxAutoExposureCompensation()
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("getMaxAutoExposureCompensation() called, but camera is not opened!");
            }

            return cameraCharacteristics.get(CameraCharacteristics.CONTROL_AE_COMPENSATION_RANGE).getUpper();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setAutoExposureCompensation(int aeCompensation)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setAutoExposureCompensation() called, but camera is not opened!");
            }

            if(aeCompensation < getMinAutoExposureCompensation())
            {
                throw new OpenCvCameraException("Auto exposure compensation must be >= the value returned by getMinAutoExposureCompensation()");
            }
            else if(aeCompensation > getMaxAutoExposureCompensation())
            {
                throw new OpenCvCameraException("Auto exposure compensation must be <= the value returned by getMaxAutoExposureCompensation()");
            }

            mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AE_EXPOSURE_COMPENSATION, aeCompensation);

            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setExposureFractional(int denominator)
    {
        double exposureTimeSeconds = 1.0/denominator;
        long exposureTimeNanos = (long) (exposureTimeSeconds * (int) 1e9);

        setExposureNanos(exposureTimeNanos);
    }

    @Override
    public void setExposureNanos(long nanos)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setExposureNanos() called, but camera is not opened!");
            }

            mPreviewRequestBuilder.set(CaptureRequest.SENSOR_EXPOSURE_TIME, nanos);
            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setFocusMode(FocusMode focusMode)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("setFocusMode() called, but camera is not opened!");
            }

            if(focusMode == FocusMode.MANUAL)
            {
                mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
            }
            else if(focusMode == FocusMode.CONTINUOUS_AUTO_PICTURE)
            {
                mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_PICTURE);
            }
            else if(focusMode == FocusMode.CONTINUOUS_AUTO_VIDEO)
            {
                mPreviewRequestBuilder.set(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_CONTINUOUS_VIDEO);
            }

            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public float getMinFocusDistance()
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new OpenCvCameraException("getMinFocusDistance() called, but camera is not opened!");
            }

            return cameraCharacteristics.get(CameraCharacteristics.LENS_INFO_MINIMUM_FOCUS_DISTANCE);
        }
        finally
        {
            sync.unlock();
        }
    }

    @Override
    public void setFocusDistance(float diopters)
    {
        sync.lock();

        try
        {
            if(mCameraDevice == null)
            {
                throw new RuntimeException("setFocusDistance() called, but camera is not opened!");
            }
            else if(diopters < 0.0)
            {
                throw new RuntimeException("Focus distance must be >= 0.0!");
            }
            else if(diopters > getMinFocusDistance())
            {
                throw new RuntimeException("Focus distance must be <= the value returned by getMinFocusDistance()");
            }

            mPreviewRequestBuilder.set(CaptureRequest.LENS_FOCUS_DISTANCE, diopters);
            apply();
        }
        finally
        {
            sync.unlock();
        }
    }

    private void apply()
    {
        try
        {
            cameraCaptureSession.setRepeatingRequest(mPreviewRequestBuilder.build(), null, cameraHardwareHandler);
        }
        catch (CameraAccessException e)
        {
            e.printStackTrace();
        }
    }

    private native long createNativeContext(int width, int height);
    private native void releaseNativeContext(long ptr);
    private native void colorConversion(ByteBuffer plane0, ByteBuffer plane1, ByteBuffer plane2, long ptrContext, long ptrRgbFrame);

    static
    {
        System.loadLibrary("EasyOpenCV");
    }
}
