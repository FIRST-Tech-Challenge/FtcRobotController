/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "SecondBlueClose", group = "Concept")
//@Disabled
public class Centerstage_AutoBlueClose extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    private static final int DESIRED_TAG_ID = 3;

    private AprilTagProcessor aprilTag;

    private AprilTagDetection desiredTag = null;

    boolean targetFound = false;

    // Do we have the second camera needed to correct the robot positioning relative to the
    // backboard to score a second pixel in autonomy mode
    private static final boolean hasSecondCamera = false;

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    Gobbler gobbler = null;

    ElapsedTime runtimeTimer = null;

    @Override
    public void runOpMode() {

        runtimeTimer = new ElapsedTime();
        runtimeTimer.startTime();

        initAprilTag();

        gobbler = new Gobbler(hardwareMap);

        //initTfod();
        gobbler.intake.intakeDown(false); // TODO: 1/30/24 We can remove this since we are not using a servo to push down the lift anymore.
        //robot.outtake.launchDrone(0.0); TODO This was intended to reset the drone launcher servo, we can move the servo manually.
        // Wait for the DS start button to be touched.
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Here we need to store an appropriate value in DESIRED_TAG_ID

        waitForStart();

        if (opModeIsActive()) {
            PlaceSecondPixel();
            parkRobot();
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    private void parkRobot() {



    }

    private void PlaceSecondPixel() {
        // Want to avoid a scenario wherein the camera doesn't recognize the april tag on the first
        // few frames, but is capable of identifying the april tag positions in subsequent frames.
        runtimeTimer.reset();
        while (runtimeTimer.time() < 0.5) {
            LocateTargetAprilTag();
        }
        // How do we want to handle potentially needing to wait for our alliance partner to
        // place their pixel on the backboard before we can?

        // What do we want        to do if we don't identify the target?
        if (targetFound) {

            driveToTarget();
           //placePixelOnBackboard();

        }
    }

    private void LocateTargetAprilTag() {
        targetFound = false;
        desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                // **DESIRED TAG IS DETERMINED BY FIRST PIXEL CODE**

                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;

                    //gobbler.driveTrain.moveForward(1,0.5);

                    break;  // don't look any further.
                } else {
                    // This tag is in the library, but we do not want to track it right now.
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
    }

    private void initAprilTag() { // TODO: 1/30/24 Once we implement tensorflow, we'll need to edit this for double vision.
        // TODO: 1/30/24 If we DO end up using two cameras, there is a way to see both cameras on the driver station.
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder()
                // TODO: 1/30/24 add the bounding boxes and axis
                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Cam1"))
                    .addProcessor(aprilTag)
                    .build();
        } else { // TODO: 1/30/24 Maybe we can remove this? We are not planning on using a phone as a camera (i hope).
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void driveToTarget() {
        // First, we want to move the robot to a known location with the april tag still visible
        // to the camera.  Once there, we need to move the robot assuming the camera will no longer
        // see the april tag.  As a consequence, we want the second leg of the trip to be as simple
        // as possible.  To that extent, we'll position the robot in such a way that it will only
        // need to drive forward.

        //driveToIntermediatePosition();
        driveToFinalPosition();

        if (hasSecondCamera) {
            fineTunePositioning();
        }
    }

    // Once we implement RoadRunner, this function will likely not be needed as RoadRunner should
    // be able to accurately and smoothly take us from whatever initial position we have to the
    // desired final position.
    private void driveToIntermediatePosition() {
        // Need to travel from current location to a predetermined intermediate location so that
        // we can accurately drive the robot up to the backboard without keeping the april tag
        // in sight the entire time.

        // What is our desired intermediate position? This can and should be independent of which
        // april tag we're moving toward.  Probably needs to be determined through testing, but a
        // reasonable approximation could be calculated from the geometry of the field/robot.
        double desiredDistance = 21.6;
       // double desiredHeading = 20.0;
        // double desiredYaw = 20.0;



        while (opModeIsActive()) {
            LocateTargetAprilTag();
            // if the camera didn't detect the desired tag in the previous cycle, give it a chance
            // to try again.
            // Might want to build in some mechanism to prevent getting stuck here, such as only
            // spending a certain amount of time here.
            if (desiredTag == null) {
                continue;
            }

            double rangeError = desiredTag.ftcPose.range - desiredDistance;
            double headingError = desiredTag.ftcPose.bearing;    // - desiredHeading;
            double yawError = desiredTag.ftcPose.yaw;   // - desiredYaw;

            double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            if (!errorIsAcceptable(rangeError, headingError, yawError)) {

                gobbler.driveTrain.drive(-drive, -strafe, turn, true, runtimeTimer);

                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.update();
            }
            else {
                telemetry.addData("Made it to the intermediate position", "");
                telemetry.update();
                break;
            }
        }
    }

    private boolean errorIsAcceptable(double rangeError, double headingError, double yawError) {
        double epsilon = 0.1;
        return ((Math.abs(rangeError) <= epsilon) && (Math.abs(headingError) < epsilon) && (Math.abs(yawError) < epsilon));
    }

    private void driveToFinalPosition() {
        // If using an intermediate position, we'll need to put in the (simple) controls to drive
        // the robot the last leg to in front of the backboard, in position to score a pixel
//        gobbler.driveTrain.strafe(6, 0.25);
//        gobbler.driveTrain.moveForward(-20, 0.5);
        if (DESIRED_TAG_ID == 1) {
            gobbler.driveTrain.moveBackward(15, 0.5);
            gobbler.driveTrain.Wait(0.5);
            gobbler.driveTrain.strafeRight(4, 0.5);
            gobbler.driveTrain.Wait(0.5);
        }

        else if (DESIRED_TAG_ID == 2) {
            gobbler.driveTrain.moveBackward(15, 0.5);
            gobbler.driveTrain.Wait(0.5);
            gobbler.driveTrain.strafeLeft(2, 0.5);
            gobbler.driveTrain.Wait(0.5);
        }

        else if (DESIRED_TAG_ID == 3) {
            gobbler.driveTrain.moveBackward(15, 0.5);
            gobbler.driveTrain.Wait(0.5);
            gobbler.driveTrain.strafeLeft(8, 0.5);
            gobbler.driveTrain.Wait(0.5);
        }

        gobbler.driveTrain.moveBackward(7, 0.5);

        // If using RoadRunner, can just directly put in a RR path to go from current position to
        // the desired final position in front of the backboard
    }

    private void fineTunePositioning() {
        // If we have the second camera ready for viewing the april tag when we're up against the
        // backboard, we can put a similar loop in here to make finer adjustments to the robot
        // position before attempting to score a pixel on the backboard.
    }

    // Once we're in position, execute code to put the pixel on the backboard
    // Probably just a combination of moving the lift and mailbox.
    private void placePixelOnBackboard() {
        gobbler.outtake.driveLift(-1.0);
        gobbler.driveTrain.Wait(2);
        gobbler.outtake.driveLift(0.0);
        gobbler.outtake.openTrapdoor();
        gobbler.driveTrain.Wait(1);
        gobbler.outtake.closeTrapdoor();
        gobbler.outtake.driveLift(1.0);
        gobbler.driveTrain.Wait(1.5);
        gobbler.outtake.driveLift(0.0);
    }
}   // end class