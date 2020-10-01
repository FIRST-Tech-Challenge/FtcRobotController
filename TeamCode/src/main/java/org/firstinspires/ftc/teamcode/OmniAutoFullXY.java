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

/**
 * Created by 12090 STEM Punk
 */
public abstract class OmniAutoFullXY extends OmniAutoXYBase
{
    // Coordinates for the vision pipeline to be overriden in the alliance classes.
    protected Point sub1PointA;
    protected Point sub1PointB;
    protected Point sub2PointA;
    protected Point sub2PointB;
    protected Point sub3PointA;
    protected Point sub3PointB;
    protected int stonePosition = 1;
    public static int position = 0;

    OpenCvCamera phoneCam;
    public abstract void setAutoWayPoints();
    public abstract void setSkystoneValues(int position);
    public abstract void setVisionPoints();

    @Override
    public void runOpMode() {
        // Setup the data needed for the vision pipeline
        setVisionPoints();

        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using the phone's internal camera. We pass it a
         * CameraDirection enum indicating whether to use the front or back facing
         * camera, as well as the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        /*
         * Open the connection to the camera device
         */
        phoneCam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        phoneCam.setPipeline(new OmniAutoFullXY.SamplePipeline());

        /*
         * Tell the camera to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Also, we specify the rotation that the camera is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        setupRobotParameters(4.0, 19.2);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
        robot.encodersReset = true;

        setAutoWayPoints();
        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        autoTimer.reset();
        stonePosition = position;
        telemetry.addData("Stone Position: ", stonePosition);
        telemetry.update();

        // Stop the image pipeline.
        phoneCam.stopStreaming();

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
        robot.startExtendingIntake();
        robot.moveLift(HardwareOmnibot.LiftPosition.STOWED);

        driveToWayPoint(distanceFromWall, true, false);
        driveToWayPoint(positionToGrabSkystone1, false, false);

        // Start the intake spinning
        robot.startIntake(false);

        // Make sure we are at the right angle
        rotateToWayPointAngle(positionToGrabSkystone1, false);
        while (!robot.intakeExtended() && opModeIsActive()) {
                updatePosition();
        }
        driveToWayPoint(grabSkystone1, false, false);
        driveToWayPoint(pullBackSkystone1, true, false);

        // Stop the intake
        robot.stopIntake();

        // Help line up to go under bridge
        driveToWayPoint(quarryUnderBridge, true, false);

        // Drive under the bridge with our skystone.
        driveToWayPoint(buildSiteUnderBridge, true, false);

        // If we didn't collect a stone, no sense placing it.
//        if (robot.stonePresent()) {
            if(!skipThis) {
                robot.liftTargetHeight = HardwareOmnibot.LiftPosition.STONE_AUTO;
                robot.startStoneStacking();
            }
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
        robot.fingersDown();
        while (autoTaskTimer.milliseconds() < robot.FINGER_ROTATE_TIME && opModeIsActive()) {
            updatePosition();
        }

        // Pull and rotate the foundation.
        driveToWayPoint(pullFoundation, false, true);
        rotateToWayPointAngle(pushFoundation, true);

        // Release the foundation.
        robot.fingersUp();
        autoTaskTimer.reset();
        while (autoTaskTimer.milliseconds() < robot.FINGER_ROTATE_TIME && opModeIsActive()) {
            updatePosition();
        }

        driveToWayPoint(buildSiteDodgingPartner, true, false);

        // Drive back to collect second skystone.
        collectStoneFoundation(positionToGrabSkystone2, grabSkystone2, pullBackSkystone2, true);

        // Drive back to collect first mundanestone.
        collectStoneFoundation(positionToGrabMundanestone1, grabMundanestone1, pullBackMundanestone1, false);

        // Finish auto by parking.
        driveToWayPoint(buildSiteReadyToRun, false, false);
        // Make sure the lift is down before going under bridge
        while (robot.stackStone != HardwareOmnibot.StackActivities.IDLE && opModeIsActive()) {
            updatePosition();
        }
        driveToWayPoint(park, false, false);
    }

    /*
     * An example image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SamplePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */
        private Mat YCrCb = new Mat();
        private Mat Cb = new Mat();
        private Mat subMat1;
        private Mat subMat2;
        private Mat subMat3;
        private int max;
        private int avg1;
        private int avg2;
        private int avg3;
        private Point skystone = new Point();

        @Override
        public Mat processFrame(Mat input)
        {
            // Convert the image from RGB to YCrCb
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);

            // Extract the Cb channel from the image
            Core.extractChannel(YCrCb, Cb, 2);

            // The the sample areas fromt the Cb channel
            subMat1 = Cb.submat(new Rect(sub1PointA, sub1PointB));
            subMat2 = Cb.submat(new Rect(sub2PointA, sub2PointB));
            subMat3 = Cb.submat(new Rect(sub3PointA, sub3PointB));

            // Average the sample areas
            avg1 = (int)Core.mean(subMat1).val[0]; // Stone4
            avg2 = (int)Core.mean(subMat2).val[0]; // Stone5
            avg3 = (int)Core.mean(subMat3).val[0]; // Stone6

            // Draw rectangles around the sample areas
            Imgproc.rectangle(input, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, sub2PointA, sub2PointB, new Scalar(0, 0, 255), 1);
            Imgproc.rectangle(input, sub3PointA, sub3PointB, new Scalar(0, 0, 255), 1);

            // Figure out which sample zone had the lowest contrast from blue (lightest color)
            max = Math.max(avg1, Math.max(avg2, avg3));

            // Draw a circle on the detected skystone
            if(max == avg1) {
                skystone.x = (sub1PointA.x + sub1PointB.x) / 2;
                skystone.y = (sub1PointA.y + sub1PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 1;
            } else if(max == avg2) {
                skystone.x = (sub2PointA.x + sub2PointB.x) / 2;
                skystone.y = (sub2PointA.y + sub2PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 2;
            } else if(max == avg3) {
                skystone.x = (sub3PointA.x + sub3PointB.x) / 2;
                skystone.y = (sub3PointA.y + sub3PointB.y) / 2;
                Imgproc.circle(input, skystone, 5, new Scalar(225, 52, 235), -1);
                position = 3;
            } else {
                position = 1;
            }

            // Free the allocated submat memory
            subMat1.release();
            subMat1 = null;
            subMat2.release();
            subMat2 = null;
            subMat3.release();
            subMat3 = null;

            return input;
        }
    }
}