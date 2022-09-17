/*
 * Original work (WebcamExample.java) copyright (c) 2018 Robert Atkinson
 * Derived work copyright (c) 2019 OpenFTC Team
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of Robert Atkinson nor the names of his contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.openftc.easyopencv;

import android.graphics.ImageFormat;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.internal.camera.CameraManagerInternal;
import org.firstinspires.ftc.robotcore.internal.camera.ImageFormatMapper;
import org.firstinspires.ftc.robotcore.internal.camera.RenumberedCameraFrame;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.api.UvcApiCameraFrame;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.nativeobject.UvcFrame;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.CameraMode;
import org.firstinspires.ftc.robotcore.internal.vuforia.externalprovider.FrameFormat;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.lang.reflect.Field;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executor;
import java.util.concurrent.TimeUnit;

@SuppressWarnings("WeakerAccess")
class OpenCvWebcamImpl extends OpenCvCameraBase implements OpenCvWebcam, CameraCaptureSession.CaptureCallback
{
    private final CameraManagerInternal cameraManager;
    private final Executor serialThreadPool;
    private volatile int millisecondsPermissionTimeout = 2000;
    private final CameraName cameraName;
    private CameraCharacteristics cameraCharacteristics = null;
    protected Camera camera = null;
    private CameraCaptureSession cameraCaptureSession = null;
    private long ptrNativeExtSourceRawSensorMat;
    private Mat rgbMat;
    private volatile boolean isStreaming = false;
    private final Object sync = new Object();
    private final Object newFrameSync = new Object();
    private boolean abortNewFrameCallback = false;
    private volatile boolean hasSeenFrame = false;
    private ExposureControl exposureControl;
    private FocusControl focusControl;
    private PtzControl ptzControl;
    private GainControl gainControl;

    //----------------------------------------------------------------------------------------------
    // Constructors
    //----------------------------------------------------------------------------------------------

    public OpenCvWebcamImpl(CameraName cameraName)
    {
        this.cameraManager = (CameraManagerInternal) ClassFactory.getInstance().getCameraManager();
        this.serialThreadPool = cameraManager.getSerialThreadPool();
        this.cameraName = cameraName;
    }

    public OpenCvWebcamImpl(CameraName cameraName, int containerLayoutId)
    {
        super(containerLayoutId);
        this.cameraManager = (CameraManagerInternal) ClassFactory.getInstance().getCameraManager();
        this.serialThreadPool = cameraManager.getSerialThreadPool();
        this.cameraName = cameraName;
    }

    //----------------------------------------------------------------------------------------------
    // Opening and closing
    //----------------------------------------------------------------------------------------------

    public CameraCharacteristics getCameraCharacteristics()
    {
        return cameraCharacteristics;
    }

    @Override
    public int openCameraDevice() /*throws CameraException*/
    {
        synchronized (sync)
        {
            if(hasBeenCleanedUp())
            {
                return CAMERA_OPEN_ERROR_POSTMORTEM_OPMODE;// We're running on a zombie thread post-mortem of the OpMode GET OUT OF DODGE NOW
            }

            if(camera == null)
            {
                try
                {
                    camera = cameraManager.requestPermissionAndOpenCamera(new Deadline(millisecondsPermissionTimeout, TimeUnit.MILLISECONDS), cameraName, null);

                    if (camera != null) //Opening succeeded!
                    {
                        cameraCharacteristics = camera.getCameraName().getCameraCharacteristics();

                        exposureControl = camera.getControl(ExposureControl.class);
                        focusControl = camera.getControl(FocusControl.class);
                        ptzControl = camera.getControl(PtzControl.class);
                        gainControl = camera.getControl(GainControl.class);
                    }
                    else //Opening failed! :(
                    {
                        return CAMERA_OPEN_ERROR_FAILURE_TO_OPEN_CAMERA_DEVICE;
                    }
                }
                catch (Exception e)
                {
                    camera = null;
                    throw e;
                }
            }

            return 0;
        }
    }

    @Override
    public void openCameraDeviceAsync(final AsyncCameraOpenListener callback)
    {
        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                synchronized (sync)
                {
                    try
                    {
                        int retCode = openCameraDevice();

                        if(retCode < 0)
                        {
                            callback.onError(retCode);
                        }
                        else
                        {
                            callback.onOpened();
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
    public void closeCameraDevice()
    {
        synchronized (sync)
        {
            cleanupForClosingCamera();

            if(camera != null)
            {
                stopStreaming();
                camera.close();
                camera = null;
            }
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
                synchronized (sync)
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
    public void startStreaming(int width, int height)
    {
        startStreaming(width, height, getDefaultRotation());
    }

    @Override
    public void startStreaming(final int width, final int height, OpenCvCameraRotation rotation)
    {
        synchronized (sync)
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

            final CountDownLatch captureStartResult = new CountDownLatch(1);

            boolean sizeSupported = false;
            for(Size s : cameraCharacteristics.getSizes(ImageFormat.YUY2))
            {
                if(s.getHeight() == height && s.getWidth() == width)
                {
                    sizeSupported = true;
                    break;
                }
            }

            if(!sizeSupported)
            {
                StringBuilder supportedSizesBuilder = new StringBuilder();

                for(Size s : cameraCharacteristics.getSizes(ImageFormat.YUY2))
                {
                    supportedSizesBuilder.append(String.format("[%dx%d], ", s.getWidth(), s.getHeight()));
                }

                throw new OpenCvCameraException("Camera does not support requested resolution! Supported resolutions are " + supportedSizesBuilder.toString());
            }

            try
            {
                camera.createCaptureSession(Continuation.create(serialThreadPool, new CameraCaptureSession.StateCallback()
                {
                    @Override
                    public void onConfigured( CameraCaptureSession session)
                    {
                        try
                        {
                            CameraMode streamingMode = new CameraMode(
                                    width,
                                    height,
                                    cameraCharacteristics.getMaxFramesPerSecond(
                                            ImageFormatMapper.androidFromVuforiaWebcam(FrameFormat.YUYV),
                                            new Size(width, height)),
                                    FrameFormat.YUYV);

                            //Indicate how we want to stream
                            final CameraCaptureRequest cameraCaptureRequest = camera.createCaptureRequest(
                                    streamingMode.getAndroidFormat(),
                                    streamingMode.getSize(),
                                    streamingMode.getFramesPerSecond());

                            // Start streaming!
                            session.startCapture(cameraCaptureRequest,
                                    OpenCvWebcamImpl.this,
                                    Continuation.create(serialThreadPool, new CameraCaptureSession.StatusCallback()
                                    {
                                        @Override
                                        public void onCaptureSequenceCompleted(
                                                CameraCaptureSession session,
                                                CameraCaptureSequenceId cameraCaptureSequenceId,
                                                long lastFrameNumber)
                                        {
                                            RobotLog.d("capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                        }
                                    }));
                        }
                        catch (CameraException | RuntimeException e)
                        {
                            e.printStackTrace();
                            RobotLog.e("exception setting repeat capture request: closing session: %s", session);
                            session.close();
                            session = null;
                        }

                        System.out.println("OpenCvWebcam: onConfigured");
                        cameraCaptureSession = session;
                        captureStartResult.countDown();
                    }

                    @Override
                    public void onClosed( CameraCaptureSession session)
                    {

                    }
                }));
            }
            catch (CameraException | RuntimeException e)
            {
                System.out.println("OpenCvWebcam: exception starting capture");
                captureStartResult.countDown();
            }

            // Wait for the above to complete
            try
            {
                captureStartResult.await(1, TimeUnit.SECONDS);
                System.out.println("OpenCvWebcam: streaming started");
            }
            catch (InterruptedException e)
            {
                Thread.currentThread().interrupt();
            }

            isStreaming = true;
        }
    }

    @Override
    protected OpenCvCameraRotation getDefaultRotation()
    {
        return OpenCvCameraRotation.SIDEWAYS_LEFT;
    }

    @Override
    protected int mapRotationEnumToOpenCvRotateCode(OpenCvCameraRotation rotation)
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

    @Override
    protected boolean cameraOrientationIsTiedToDeviceOrientation()
    {
        return false;
    }

    @Override
    protected boolean isStreaming()
    {
        return isStreaming;
    }

    /***
     * Stop streaming frames from the webcam, if we were
     * streaming in the first place. If not, we don't do
     * anything at all here.
     */
    @Override
    public void stopStreaming()
    {
        synchronized (sync)
        {
            if(camera == null)
            {
                throw new OpenCvCameraException("stopStreaming() called, but camera is not opened!");
            }

            synchronized (newFrameSync)
            {
                abortNewFrameCallback = true;

                cleanupForEndStreaming();

                if(ptrNativeExtSourceRawSensorMat != 0)
                {
                    freeExtImgDatMat(ptrNativeExtSourceRawSensorMat);
                    ptrNativeExtSourceRawSensorMat = 0;
                }

                rgbMat = null;
            }

            if (cameraCaptureSession != null)
            {
                /*
                 * Testing has shown that if we try to close too quickly,
                 * (i.e. before we've even gotten a frame) then while the
                 * close DOES go OK, we cannot subsequently re-open.
                 */

                boolean wasInterrupted = false;

                while (!hasSeenFrame)
                {
                    try
                    {
                        Thread.sleep(10);
                    }
                    catch (InterruptedException e)
                    {
                        e.printStackTrace();
                        wasInterrupted = true;
                    }
                }

                cameraCaptureSession.stopCapture();
                cameraCaptureSession.close();
                cameraCaptureSession = null;

                if(wasInterrupted)
                {
                    Thread.currentThread().interrupt();
                }
            }

            isStreaming = false;
        }
    }

    @Override
    public void onNewFrame( CameraCaptureSession session,  CameraCaptureRequest request,  CameraFrame cameraFrame)
    {
        hasSeenFrame = true;

        synchronized (newFrameSync)
        {
            if(abortNewFrameCallback)
            {
                /*
                 * Get out of dodge NOW. nativeStopStreaming() can deadlock with nativeCopyImageData(),
                 * so we use this flag to avoid that happening. But also, it can make stopping slightly
                 * more responsive if the user pipeline is particularly expensive.
                 */
                return;
            }

            notifyStartOfFrameProcessing();

            if(rgbMat == null)
            {
                rgbMat = new Mat(cameraFrame.getSize().getHeight(), cameraFrame.getSize().getWidth(), CvType.CV_8UC1);
            }
            if(ptrNativeExtSourceRawSensorMat == 0)
            {
                ptrNativeExtSourceRawSensorMat = allocExtImgDatMat(cameraFrame.getSize().getWidth(), cameraFrame.getSize().getHeight(),  CvType.CV_8UC2);
            }

            setMatDataPtr(ptrNativeExtSourceRawSensorMat, cameraFrame.getImageBuffer());
            colorConversion(ptrNativeExtSourceRawSensorMat, rgbMat.nativeObj);

            handleFrame(rgbMat, cameraFrame.getCaptureTime());
        }
    }

    @Override
    public void setMillisecondsPermissionTimeout(int ms)
    {
        millisecondsPermissionTimeout = ms;
    }

    @Override
    public ExposureControl getExposureControl()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("getExposureControl() called, but camera is not opened!");
        }

        return exposureControl;
    }

    @Override
    public FocusControl getFocusControl()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("getFocusControl() called, but camera is not opened!");
        }

        return focusControl;
    }

    @Override
    public PtzControl getPtzControl()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("getPtzControl() called, but camera is not opened!");
        }

        return ptzControl;
    }

    @Override
    public GainControl getGainControl()
    {
        if(camera == null)
        {
            throw new OpenCvCameraException("getGainControl() called, but camera is not opened!");
        }

        return gainControl;
    }

    public static native void setMatDataPtr(long matPtr, long dataPtr);
    public static native void colorConversion(long rawDataPtr, long rgbPtr);
    public static native long allocExtImgDatMat(int width, int height, int type);
    public static native void freeExtImgDatMat(long ptr);

    static
    {
        System.loadLibrary("EasyOpenCV");
    }
}