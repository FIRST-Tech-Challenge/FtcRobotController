/*
 * Copyright (c) 2019 OpenFTC Team
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

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.RobotUtilities.MyPosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

/**
 * Created by 12090 STEM Punk
 */
public abstract class UltimateGoalAutoFull extends UltimateGoalAutoBase
{
    // Coordinates for the vision pipeline to be overriden in the alliance classes.
    protected int stonePosition = 1;
    public static int position = 0;
    protected enum TARGET_ZONE {
        A,
        B,
        C
    }
    protected TARGET_ZONE targetZone = TARGET_ZONE.A;

    OpenCvCamera phoneCam;
    public abstract void setAutoWayPoints();
    public abstract void setSkystoneValues(int position);
    public abstract void setVisionPoints();

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
//            tfod.setZoom(2.5, 16.0/9.0);
        }

        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        setupRobotParameters();

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
        robot.encodersReset = true;

        setAutoWayPoints();
        /*
         * Wait for the user to press start on the Driver Station
         */
        while(!isStarted()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_FIRST_ELEMENT)) {
                            targetZone = TARGET_ZONE.C;
                        } else {
                            targetZone = TARGET_ZONE.B;
                        }
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                    }
                    telemetry.update();
                }
            }
        }
//        waitForStart();

        // In case Stop was pressed.
        if(opModeIsActive()) {
            autoTimer.reset();
            telemetry.addData("Target Zone: ", targetZone);
            telemetry.update();

            // This sets up everything for the auto to run.
            setSkystoneValues(stonePosition);

            //give MyPosition our current positions so that it saves the last positions of the wheels
            //this means we won't teleport when we start the match. Just in case, run this twice
            for (int i = 0; i < 2; i++) {
                robot.resetReads();
                MyPosition.initialize(robot.getLeftEncoderWheelPosition(),
                        robot.getRightEncoderWheelPosition(),
                        robot.getStrafeEncoderWheelPosition());
            }

            // Set our robot starting coordinates on the field.
            robot.resetReads();
            MyPosition.setPosition(startLocation.x, startLocation.y, startLocation.angle);

            // Start moving intake out, should be done by the time driving is done.
//        robot.startExtendingIntake();
//        robot.moveLift(HardwareOmnibot.LiftPosition.STOWED);

            driveToWayPoint(distanceFromWall, true, false);
            driveToWayPoint(positionToGrabSkystone1, false, false);

            // Start the intake spinning
//        robot.startIntake(false);

            // Make sure we are at the right angle
            rotateToWayPointAngle(positionToGrabSkystone1, false);
//        while (!robot.intakeExtended() && opModeIsActive()) {
//                updatePosition();
//        }
            driveToWayPoint(grabSkystone1, false, false);
            driveToWayPoint(pullBackSkystone1, true, false);

            // Stop the intake
//        robot.stopIntake();

            // Help line up to go under bridge
            driveToWayPoint(quarryUnderBridge, true, false);

            // Drive under the bridge with our skystone.
            driveToWayPoint(buildSiteUnderBridge, true, false);

            // If we didn't collect a stone, no sense placing it.
//        if (robot.stonePresent()) {
//            if(!skipThis) {
//                robot.liftTargetHeight = HardwareOmnibot.LiftPosition.STONE_AUTO;
//                robot.startStoneStacking();
//            }
//        }

            // Drive into foundation to grab it
            driveToWayPoint(snuggleFoundation, false, false);
            rotateToWayPointAngle(snuggleFoundation, false);

            driveToWayPoint(grabFoundation, true, false);
            autoTaskTimer.reset();
            while (autoTaskTimer.milliseconds() < 500 && opModeIsActive()) {
                updatePosition();
            }
            autoTaskTimer.reset();
//        robot.fingersDown();
//        while (autoTaskTimer.milliseconds() < robot.FINGER_ROTATE_TIME && opModeIsActive()) {
//            updatePosition();
//        }

            // Pull and rotate the foundation.
            driveToWayPoint(pullFoundation, false, true);
            rotateToWayPointAngle(pushFoundation, true);

            // Release the foundation.
//        robot.fingersUp();
            autoTaskTimer.reset();
//        while (autoTaskTimer.milliseconds() < robot.FINGER_ROTATE_TIME && opModeIsActive()) {
//            updatePosition();
//        }

            driveToWayPoint(buildSiteDodgingPartner, true, false);

            // Drive back to collect second skystone.
            collectStoneFoundation(positionToGrabSkystone2, grabSkystone2, pullBackSkystone2, true);

            // Drive back to collect first mundanestone.
            collectStoneFoundation(positionToGrabMundanestone1, grabMundanestone1, pullBackMundanestone1, false);

            // Finish auto by parking.
            driveToWayPoint(buildSiteReadyToRun, false, false);
            // Make sure the lift is down before going under bridge
//        while (robot.stackStone != HardwareOmnibot.StackActivities.IDLE && opModeIsActive()) {
//            updatePosition();
//        }
            driveToWayPoint(park, false, false);
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }
}