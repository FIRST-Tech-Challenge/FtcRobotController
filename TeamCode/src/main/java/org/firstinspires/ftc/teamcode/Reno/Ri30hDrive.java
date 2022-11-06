/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Reno;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// Add webcam
import android.annotation.SuppressLint;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.graphics.ImageFormat;
import android.graphics.Point;
import android.os.Build;
import android.os.Handler;

import androidx.annotation.NonNull;
import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
 * This file is used for Ri30h event at Homestead high school on Sept. 10th, 2022.
 * It include codes for Autonomous running.
 * 1. Get the color of center pixel from webcam which is connected to control hub usb2.0
 * 2. Move robot to the parking spots #1, #2, #3 based on color.
 *
 */

@Autonomous(name="Ri30hDrive", group="Neptune")
@Disabled
public class Ri30hDrive extends LinearOpMode {

    static final int RED = 1;
    static final int GREEN = 2;
    static final int BLUE = 3;
    static final int UNKNOWNCOLOR = 0;

    HardwareRobot robot   = new HardwareRobot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    double power = 0.2;

    int colorCode = 0;

// add for webcam
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
    private int captureCounter = 0;
    private File captureDirectory = AppUtil.ROBOT_DATA_DIR;

    /** A utility object that indicates where the asynchronous callbacks from the camera
     * infrastructure are to run. In this OpMode, that's all hidden from you (but see {@link #startCamera}
     * if you're curious): no knowledge of multi-threading is needed here. */
    private Handler callbackHandler;

    @RequiresApi(api = Build.VERSION_CODES.Q)

    @Override
    public void runOpMode() {

        // add for webcam
        callbackHandler = CallbackLooper.getDefault().getHandler();
        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        initializeFrameQueue(10);
        AppUtil.getInstance().ensureDirectoryExists(captureDirectory);
        waitForStart();

        // end webcam
        robot.init(hardwareMap);

        telemetry.log().add("Current color code is %d", colorCode);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        robot.leftDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.leftDriveBack.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        runtime.reset();
        telemetry.clear();
        //while (opModeIsActive() && (runtime.seconds() < 30.0))
        if (opModeIsActive() && (runtime.seconds() < 30.0))
        {
            // Moving forward
            telemetry.addData("Status", "Move forward to read color.");
            power = 0.2;
            robot.leftDriveFront.setPower(power);
            robot.leftDriveBack.setPower(power);
            robot.rightDriveBack.setPower(power);
            robot.rightDriveFront.setPower(power);
            sleep(600);
            this.robotStop();

            try {
                openCamera();
                if (camera == null){
                    telemetry.addData(">", "camera is not found!");
                    return;
                }

                startCamera();
                if (cameraCaptureSession == null) {
                    telemetry.addData(">", " Cannot start camera!");
                    return;
                }

                telemetry.addData(">", "Press Play to start");
                telemetry.update();


                if (opModeIsActive() ) {
                    Bitmap bmp = frameQueue.poll();

                    if (bmp != null) {
                        colorCode = onNewFrame(bmp);
                        telemetry.log().add("Current color code %d", colorCode);
                    }
                    else{
                        telemetry.log().add("Current bitmap is NULL");
                    }
                }
                telemetry.update();

            } finally {
                closeCamera();
            }
            sleep(2000); // temp added sleep for log reading from screen.


            if ((RED == colorCode) | (UNKNOWNCOLOR == colorCode)) {

                // Moving forward then
                telemetry.addData("Status", "Move forward to zone 1 when red.");
                power = 0.2;
                robot.leftDriveFront.setPower(power);
                robot.leftDriveBack.setPower(power);
                robot.rightDriveBack.setPower(power);
                robot.rightDriveFront.setPower(power);
                sleep(2500);
                this.robotStop();

                // Moving left first
                telemetry.addData("Status", "Move Left to zone 1 when red.");
                power = 0.2;
                robot.leftDriveFront.setPower(-power);
                robot.leftDriveBack.setPower(power);
                robot.rightDriveBack.setPower(-power);
                robot.rightDriveFront.setPower(power);
                sleep(4000);
                this.robotStop();

            }
            else if(BLUE == colorCode) {

                // Moving forward then
                telemetry.addData("Status", "Move forward to zone 3 when blue.");
                power = 0.2;
                robot.leftDriveFront.setPower(power);
                robot.leftDriveBack.setPower(power);
                robot.rightDriveBack.setPower(power);
                robot.rightDriveFront.setPower(power);
                sleep(2500);
                this.robotStop();

                // Moving right first
                telemetry.addData("Status", "Move right to zone 3 when blue.");
                power = 0.2;
                robot.leftDriveFront.setPower(power);
                robot.leftDriveBack.setPower(-power);
                robot.rightDriveBack.setPower(power);
                robot.rightDriveFront.setPower(-power);
                sleep(4000);
                this.robotStop();

            }
            else {
                // Moving forward
                telemetry.addData("Status", "Move forward to zone 2 when green.");
                power = 0.2;
                robot.leftDriveFront.setPower(power);
                robot.leftDriveBack.setPower(power);
                robot.rightDriveBack.setPower(power);
                robot.rightDriveFront.setPower(power);
                sleep(2500);
                this.robotStop();
            }

/*
            // add slide motor actions.
            telemetry.addData("Status", "ready to run slider motor");
            power = 0.2;
            robot.sliderMotor.setPower(power);
            sleep(4000); // run slider motor 4 seconds
            this.robotStop();

            // add servo motor actions.
            telemetry.addData("Status", "ready to open servo motor");
            robot.gripperServo.setPosition(0.3);
            sleep(2000);

            telemetry.addData("Status", "ready to move backward for 2 seconds");
            //Moving backward
            power = -0.2;
            robot.leftDriveFront.setPower(power);
            robot.leftDriveBack.setPower(power);
            robot.rightDriveBack.setPower(power);
            robot.rightDriveFront.setPower(power);
            sleep(2000);
            this.robotStop();

            telemetry.addData("Status", "ready to turn left for 1 second");
            //turning left
            power = 0.10;
            robot.leftDriveFront.setPower(-power);
            robot.leftDriveBack.setPower(-power);
            robot.rightDriveBack.setPower(power);
            robot.rightDriveFront.setPower(power);
            sleep(2000);
            this.robotStop();

            telemetry.addData("Status", "ready to move forward for 2 second");
            //Move Forward
            power = 0.2;
            robot.leftDriveFront.setPower(power);
            robot.leftDriveBack.setPower(power);
            robot.rightDriveBack.setPower(power);
            robot.rightDriveFront.setPower(power);
            sleep(2000);
            this.robotStop();

            // add servo motor actions.
            telemetry.addData("Status", "ready to open servo motor");
            robot.gripperServo.setPosition(0.0);
            sleep(1000);

            telemetry.addData("Status", "ready to turn right for 1 seconds");
            //turning right
            power = 0.10;
            robot.leftDriveFront.setPower(power);
            robot.leftDriveBack.setPower(power);
            robot.rightDriveBack.setPower(-power);
            robot.rightDriveFront.setPower(-power);
            sleep(2000);
            this.robotStop();

            telemetry.update();
 */
        }

        this.robotStop();
        telemetry.addData("Status" , "Program Finished");
        telemetry.update();
    }

    public void robotStop(){
      double  power = 0.0;

        robot.leftDriveFront.setPower(power);
        robot.leftDriveBack.setPower(power);
        robot.rightDriveBack.setPower(power);
        robot.rightDriveFront.setPower(power);
        robot.sliderMotor.setPower(power);
        sleep(1000);
    }

    // code copied from webcam

    /** Do something with the frame */
    @SuppressLint("NewApi")
    @RequiresApi(api = Build.VERSION_CODES.Q)

    private int onNewFrame(Bitmap frame) {
        saveBitmap(frame);

        int x0 = frame.getWidth()/2;
        int y0 = frame.getHeight()/2;

        telemetry.addData("Start get pixel", "...start");
        int color0 = frame.getPixel(x0, y0);
        int r0 = Color.red(color0);
        int g0 = Color.green(color0);
        int b0 = Color.blue(color0);

        int color1 = frame.getPixel(x0+1, y0);
        int r1 = Color.red(color1);
        int g1 = Color.green(color1);
        int b1 = Color.blue(color1);
        telemetry.log().add("Finishing first pixel color", "color0 = %d, r1=%d, g1=%d, b1=%d", color0, r1, g1, b1);
        int color2 = frame.getPixel(x0, y0+1);
        int r2 = Color.red(color2);
        int g2 = Color.green(color2);
        int b2 = Color.blue(color2);

        int color3 = frame.getPixel(x0+1, y0+1);
        int r3 = Color.red(color3);
        int g3 = Color.green(color3);
        int b3 = Color.blue(color3);

        int r = r0+r1+r2+r3;
        int g = g0+g1+g2+g3;
        int b = b0+b1+b2+b3;

        telemetry.log().add("color code %s %s %s", r/4,g/4,b/4);
        if(r>g & r>b) {
            return RED;
        }

        if(g>r & g>b) {
            return GREEN;
        }
        if(b>g & b>r) {
            return BLUE;
        }

        frame.recycle(); // not strictly necessary, but helpful

        return UNKNOWNCOLOR;
    }



    //----------------------------------------------------------------------------------------------
    // Camera operations
    //----------------------------------------------------------------------------------------------

    private void initializeFrameQueue(int capacity) {
        /** The frame queue will automatically throw away bitmap frames if they are not processed
         * quickly by the OpMode. This avoids a buildup of frames in memory */
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
            error("camera not found or permission to use not granted: %s", cameraName);
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) return; // be idempotent

        /** YUY2 is supported by all Webcams, per the USB Webcam standard: See "USB Device Class Definition
         * for Video Devices: Uncompressed Payload, Table 2-1". Further, often this is the *only*
         * image format supported by a camera */
        final int imageFormat = ImageFormat.YUY2;

        /** Verify that the image is supported, and fetch size and desired frame rate if so */
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if (!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)) {
            error("image format not supported");
            return;
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);

        /** Some of the logic below runs asynchronously on other threads. Use of the synchronizer
         * here allows us to wait in this method until all that asynchrony completes before returning. */
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            /** Create a session in which requests to capture frames can be made */
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        /** The session is ready to go. Start requesting frames */
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                        /** A new frame is available. The frame data has <em>not</em> been copied for us, and we can only access it
                                         * for the duration of the callback. So we copy here manually. */
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                        RobotLog.ii(TAG, "frameQueue is just updated.");
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

        /** Wait for all the asynchrony to complete */
        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        /** Retrieve the created session. This will be null on error. */
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
        telemetry.log().add(msg);
        telemetry.update();
    }
    private void error(String format, Object...args) {
        telemetry.log().add(format, args);
        telemetry.update();
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) return true;
        }
        return false;
    }

    private void saveBitmap(Bitmap bitmap) {
        File file = new File(captureDirectory, String.format(Locale.getDefault(), "webcam-frame-%d.jpg", captureCounter++));
        try {
            try (FileOutputStream outputStream = new FileOutputStream(file)) {
                bitmap.compress(Bitmap.CompressFormat.JPEG, 100, outputStream);
                telemetry.log().add("captured %s", file.getName());
            }
        } catch (IOException e) {
            RobotLog.ee(TAG, e, "exception in saveBitmap()");
            error("exception saving %s", file.getName());
        }
    }
    //end for webcam
}
