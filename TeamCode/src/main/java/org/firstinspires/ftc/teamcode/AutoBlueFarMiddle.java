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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
 * This sample demonstrates how to run analysis during INIT
 * and then snapshot that value for later use when the START
 * command is issued. The pipeline is re-used from SkystoneDeterminationExample
 */
@Autonomous(name = "AutoBlueFarMiddle", group = "linear autoMode")
public class AutoBlueFarMiddle extends RobotLinearOpMode
{
    OpenCvWebcam webcam;
    BluePropDetector.SkystoneDeterminationPipeline pipeline;
    BluePropDetector.SkystoneDeterminationPipeline.SkystonePosition snapshotAnalysis = BluePropDetector.SkystoneDeterminationPipeline.SkystonePosition.LEFT; // default

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BluePropDetector.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        declareHardwareProperties();

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }

        /*
         * The START command just came in: snapshot the current analysis now
         * for later use. We must do this because the analysis will continue
         * to change as the camera view changes once the robot starts moving!
         */
        sleep(5000);
        snapshotAnalysis = pipeline.getAnalysis();

        /*
         * Show that snapshot on the telemetry
         */
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                /* Your autonomous code */
                encoderDrive(0.5, 26, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(0.5, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(0.5, 4, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(0.5, 12, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(0.5, 23, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(0.5, 4, MOVEMENT_DIRECTION.FORWARD);
                sleep(5000);
                encoderDrive(0.5, 45, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(0.5, 50, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(0.5, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
                sleep(25000);
            }

            case RIGHT:
            {
                /* Your autonomous code */
                encoderDrive(0.5, 25, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(0.5, 9, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(0.5, 10, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(0.5, 8, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(0.5, 16, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(0.5, 3, MOVEMENT_DIRECTION.FORWARD);
                sleep(5000);
                encoderDrive(0.5, 45, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(0.5, 50, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(0.5, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
                sleep(25000);
            }

            case CENTER:
            {
                /* Your autonomous code*/
                encoderDrive(0.5, 33.5, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(0.5, 10, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(0.5, 2, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(0.5, 25, MOVEMENT_DIRECTION.REVERSE);
                encoderDrive(0.5, 3, MOVEMENT_DIRECTION.FORWARD);
                sleep(5000);
                encoderDrive(0.5, 45, MOVEMENT_DIRECTION.STRAFE_LEFT);
                encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
                encoderDrive(0.5, 50, MOVEMENT_DIRECTION.FORWARD);
                encoderDrive(0.5, 10, MOVEMENT_DIRECTION.STRAFE_LEFT);
                sleep(25000);
            }
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        while (opModeIsActive())
        {
            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}