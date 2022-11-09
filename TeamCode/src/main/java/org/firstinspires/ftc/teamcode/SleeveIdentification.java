/* Copyright (c) 2020 FIRST. All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Locale;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * This OpMode illustrates how to open a webcam and retrieve images from it. It requires a configuration
 * containing a webcam with the default name ("Webcam 1"). When the opmode runs, pressing the 'A' button
 * will cause a frame from the camera to be written to a file on the device, which can then be retrieved
 * by various means (e.g.: Device File Explorer in Android Studio; plugging the device into a PC and
 * using Media Transfer; ADB; etc)
 */

public class SleeveIdentification {

    public enum sleeveSignal
    {
        UNKNOWN,
        RED,
        GREEN,
        BLUE
    }

    private sleeveSignal colorName = sleeveSignal.UNKNOWN;

    HardwareMap hardwareMap =  null;
    private static final String TAG = "Webcam Sample";

    /** How long we are to wait to be granted permission to use the camera before giving up. Here,
     * we wait indefinitely */
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;

    /** State regarding our interaction with the camera */
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;

    /** The queue into which all frames from the camera are placed as they become available.
     * Frames which are not processed by the OpMode are automatically discarded. */
    private EvictingBlockingQueue<Bitmap> frameQueue;

    /** State regarding where and how to save frames when the 'A' button is pressed. */
    private final File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    /** A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}
     * if you're curious): no knowledge of multi-threading is needed here. */
    private Handler callbackHandler;

    //----------------------------------------------------------------------------------------------
    // Main OpMode entry
    //----------------------------------------------------------------------------------------------

    /* Constructor */
    public SleeveIdentification(HardwareMap hardwareMap){
        initCamera(hardwareMap);
    }

    public boolean initCamera(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;
        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = this.hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(2);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);

        openCamera();
        if (camera == null)
            return false;

        startCamera();
        return cameraCaptureSession != null;
    }

    public sleeveSignal captureSleeveSignal() {
        Bitmap bmp = frameQueue.poll();

        while (null != camera) {
            if (bmp != null) {
                int pixel = this.getDominantColor(bmp);
                int redValue = Color.red(pixel);
                int blueValue = Color.blue(pixel);
                int greenValue = Color.green(pixel);

                Logging.log("picture found %d", pixel);
                Logging.log("green color %d", greenValue);
                Logging.log("red color %d", redValue);
                Logging.log("blue color %d", blueValue);

                if(redValue > greenValue && redValue > blueValue)
                {
                    colorName = sleeveSignal.RED;
                }
                if(greenValue > blueValue && greenValue > redValue)
                {
                    colorName = sleeveSignal.GREEN;
                }
                if(blueValue > redValue && blueValue > greenValue)
                {
                    colorName = sleeveSignal.BLUE;
                }
                break;
            }
        }
        closeCamera();
        return colorName;
    }

    public sleeveSignal getSleeveSignal() {
        return colorName;
    }

    private int getDominantColor(Bitmap bitmap) {

        // TODO : update below 4 variable values due to sleeve location in the view of image.
        int cropStartX = bitmap.getWidth() / 4;
        int cropStartY = bitmap.getHeight() / 4;
        int cropWidth = bitmap.getWidth() / 2;
        int cropHeight = bitmap.getHeight() / 2;
        Bitmap croppedBitmap = Bitmap.createBitmap(bitmap, cropStartX, cropStartY, cropWidth, cropHeight);

        Bitmap newBitmap = Bitmap.createScaledBitmap(croppedBitmap, 1, 1, true);

        return newBitmap.getPixel(0, 0);
    }

    //----------------------------------------------------------------------------------------------
    // Camera operations
    //----------------------------------------------------------------------------------------------

    private void initializeFrameQueue(int capacity) {
        /* The frame queue will automatically throw away bitmap frames if they are not processed
          quickly by the OpMode. This avoids a buildup of frames in memory */
        frameQueue = new EvictingBlockingQueue<Bitmap>(new ArrayBlockingQueue<Bitmap>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override public void accept(Bitmap frame) {
                // RobotLog.ii(TAG, "frame recycled w/o processing");
                frame.recycle(); // not strictly necessary, but helpful
            }
        });
    }

    private void openCamera() {
        if (camera != null) return; // be idempotent

        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            Logging.log("camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        /* YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
          for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
          image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /* Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /* Some of the logic below runs asynchronously on other threads. Use of the synchronizer
          here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /* Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /* The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                            new CameraCaptureSession.CaptureCallback() {
                                @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                    /* A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                     * for the duration of the callback. So we copy here manually. */
                                    Bitmap bmp = captureRequest.createEmptyBitmap();
                                    cameraFrame.copyToBitmap(bmp);
                                    frameQueue.offer(bmp);
                                }
                            },
                            Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                    RobotLog.ii(TAG, "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                }
                            })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException|RuntimeException e) {
                        RobotLog.ee(TAG, e, "exception starting capture");
                        error("exception starting capture");
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
            RobotLog.ee(TAG, e, "exception starting camera");
            error("exception starting camera");
            synchronizer.finish(null);
        }

        /* Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /* Retrieve the created session. This will be null on error. */
        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Utilities
    //----------------------------------------------------------------------------------------------

    private void error(String msg) {
        Logging.log(msg);
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }
}
