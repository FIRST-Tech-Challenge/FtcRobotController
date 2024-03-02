/* Copyright (c) 2023 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Testing.Auton_test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="Tensor April F4", group = "Concept")
@Disabled
public class Auton_F4_test extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 2.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.0138  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.006 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.00625  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.75;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.5;   //  Clip the turn speed to this max value (adjust for your robot)

    private WebcamName webcamAT;
    private WebcamName webcamTF;
    private DcMotor lf_drive   = null;  //  Used to control the left front drive wheel
    private DcMotor rf_drive  = null;  //  Used to control the right front drive wheel
    private DcMotor lb_drive    = null;  //  Used to control the left back drive wheel
    private DcMotor rb_drive   = null;  //  Used to control the right back drive wheel
    private DcMotor arm = null; // Used to control arm
    Servo grip; // Used to control gripper

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source for AprilTags.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    private TfodProcessor tfod;                      // Used for managing TensorFlow detection
    private static final String TFOD_MODEL_FILE = "model_20231110_182709.tflite";
    private static final String[] LABELS = {
            "red"
    };
    private boolean TFdone = false;
    private boolean ATdone = false;
    @Override
    public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initDoubleVision();
//        ATdone = true;
//        doCameraSwitching();
//        ATdone = false;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        lb_drive = hardwareMap.get(DcMotor.class, "lb_drive");
        rb_drive = hardwareMap.get(DcMotor.class, "rb_drive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        grip = hardwareMap.get(Servo.class, "grip");

        grip.setPosition(0.7);

//        if (USE_WEBCAM) {
//            setManualExposure(10, 100);  // Use low exposure time to reduce motion blur
//        } // End if statement

        // Wait for driver to press start
//        while (!isStarted()) {
//            if (opModeInInit()) {
//                telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//                telemetry.addLine();
//                telemetry.addLine("----------------------------------------");
//            }
//            telemetryCameraSwitching();
//
//            // Push telemetry to the Driver Station.
//            telemetry.update();
//
//
//        } // End while loop

        waitForStart();

        while (opModeIsActive())
        {
//            ATdone = true;
//            doCameraSwitching();
//            ATdone = false;

            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            lf_drive.setDirection(DcMotor.Direction.REVERSE);
            rf_drive.setDirection(DcMotor.Direction.FORWARD);
            lb_drive.setDirection(DcMotor.Direction.FORWARD);
            rb_drive.setDirection(DcMotor.Direction.FORWARD);
            telemetryTfod();
            telemetry.addData("Desired Tag", DESIRED_TAG_ID);
            telemetry.update();
            lf_drive.setPower(0);
            rf_drive.setPower(0);
            lb_drive.setPower(0);
            rb_drive.setPower(0);

//            TFdone = true;
//            doCameraSwitching();
//            TFdone = false;


//            strafe_left(500);
//            move_forward(790);
//            rotate_left90();
//            move_forward(115);
//            pause(1);

            lf_drive.setDirection(DcMotor.Direction.REVERSE);
            rf_drive.setDirection(DcMotor.Direction.REVERSE);
            lb_drive.setDirection(DcMotor.Direction.FORWARD);
            rb_drive.setDirection(DcMotor.Direction.REVERSE);

            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
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

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(turn, strafe, drive);
            sleep(10);

            requestOpModeStop();
        }
    }

    //---------------------------------------------
    // Move robot according to desired axes motions
    // <p>
    // Positive X is forward
    // <p>
    // Positive Y is strafe left
    // <p>
    // Positive Yaw is counter-clockwise
    //---------------------------------------------
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double lf_power   =  x + y + yaw;
        double rf_power   =  x - y - yaw;
        double lb_power   =  x - y + yaw;
        double rb_power   =  x + y - yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(lf_power), Math.abs(rf_power));
        max = Math.max(max, Math.abs(lb_power));
        max = Math.max(max, Math.abs(rb_power));

        if (max > 1.0) {
            lf_power /= max;
            rf_power /= max;
            lb_power /= max;
            rb_power /= max;
        }

        // Send powers to the wheels.
        lf_drive.setPower(lf_power);
        rf_drive.setPower(rf_power);
        lb_drive.setPower(lb_power);
        rb_drive.setPower(rb_power);
    }

    //-----------------------------------
    // Initialize the AprilTag processor.
    //-----------------------------------
    private void initDoubleVision() {

        // ----------------------
        // AprilTag Configuration
        // ----------------------
        aprilTag = new AprilTagProcessor.Builder()

                .build();

        // ------------------
        // TFOD Configuration
        // ------------------
        tfod = new TfodProcessor.Builder()

                .setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)

                .build();

        // --------------------
        // Camera Configuration
        // --------------------
//        webcamAT = hardwareMap.get(WebcamName.class, "23258 AprilTag Cam");
        webcamTF = hardwareMap.get(WebcamName.class, "23258 TensorFlow Cam");
//        SwitchableCameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(webcamAT, webcamTF);


        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(webcamTF)
                    .addProcessors(tfod, aprilTag)
                    .build();

        }
    }   // end initDoubleVision()

    //----------------------------------------------------------------------------------
    // Manually set the camera gain and exposure.
    // This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    //----------------------------------------------------------------------------------
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("camera", "Ready");
            telemetry.update();
        }


        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

            sleep(20);
        }
    } // End setManualExposure()

    //--------------------------------------
    // Add telemetry about camera switching.
    //--------------------------------------
    private void telemetryCameraSwitching() {

        if (visionPortal.getActiveCamera().equals(webcamAT)) {
            telemetry.addData("activeCamera", "WebcamAT");
            telemetry.addData("Press RightBumper", "to switch to WebcamAT");
        } else {
            telemetry.addData("activeCamera", "WebcamTF");
            telemetry.addData("Press LeftBumper", "to switch to WebcamTF");
        }

    }   // end method telemetryCameraSwitching()

    //----------------------------------------
    // Add telemetry about AprilTag detection.
    //----------------------------------------
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

    }   // end method telemetryAprilTag()

    //---------------------------------------------------------------------
    // Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
    //---------------------------------------------------------------------
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        telemetry.addData("# Objects Detected", currentRecognitions.size());


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            String pixelPosition = "";
            double cameraWidth = 640;
            if (x < cameraWidth / 2) {
                pixelPosition = "middle";
                lf_drive.setPower(0.4);
                rf_drive.setPower(0.4);
                lb_drive.setPower(0.4);
                rb_drive.setPower(0.4);
                sleep(1350);
                lf_drive.setPower(-0.4);
                rf_drive.setPower(-0.4);
                lb_drive.setPower(-0.4);
                rb_drive.setPower(-0.4);
                sleep(1300);
                lf_drive.setPower(0);
                rf_drive.setPower(0);
                lb_drive.setPower(0);
                rb_drive.setPower(0);
                DESIRED_TAG_ID = 2;

                strafe_right(900);
                move_forward(790);
                rotate_right90();
                move_forward(450);
                pause(1);

                arm.setPower(0.5);
                sleep(100);
                arm.setPower(0);
                grip.setPosition(0.55);

            } else if (x > cameraWidth /2) {
                pixelPosition = "right";

                lf_drive.setPower(0.3);
                rf_drive.setPower(0.3);
                lb_drive.setPower(0.3);
                rb_drive.setPower(0.3);
                sleep(1700);
//                arm.setPower(0.1);
//                sleep(100);
                lf_drive.setPower(-0.4);
                rf_drive.setPower(0.4);
                lb_drive.setPower(-0.4);
                rb_drive.setPower(0.4);
                sleep(1000);
                lf_drive.setPower(0.4);
                rf_drive.setPower(0.4);
                lb_drive.setPower(0.4);
                rb_drive.setPower(0.4);
                sleep(300);

                lf_drive.setPower(-0.4);
                rf_drive.setPower(-0.4);
                lb_drive.setPower(-0.4);
                rb_drive.setPower(-0.4);
                sleep(500);
                lf_drive.setPower(0.4);
                rf_drive.setPower(-0.4);
                lb_drive.setPower(0.4);
                rb_drive.setPower(-0.4);
                sleep(1000);
                lf_drive.setPower(-0.3);
                rf_drive.setPower(-0.3);
                lb_drive.setPower(-0.3);
                rb_drive.setPower(-0.3);
                sleep(1600);
                DESIRED_TAG_ID = 3;

                strafe_right(900);
                move_forward(790);
                rotate_right90();
                strafe_right(225);
                move_forward(450);
                pause(1);

                arm.setPower(0.5);
                sleep(100);
                arm.setPower(0);
                grip.setPosition(0.55);
            }



            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f",x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.addLine(pixelPosition);

        }   // end for() loop
        if(currentRecognitions.size()==0){

            lf_drive.setPower(0.3);
            rf_drive.setPower(0.3);
            lb_drive.setPower(0.3);
            rb_drive.setPower(0.3);
            sleep(1700);
//            arm.setPower(0.1);
//            sleep(100);
            lf_drive.setPower(0.4);
            rf_drive.setPower(-0.4);
            lb_drive.setPower(0.4);
            rb_drive.setPower(-0.4);
            sleep(1000);
            lf_drive.setPower(0.4);
            rf_drive.setPower(0.4);
            lb_drive.setPower(0.4);
            rb_drive.setPower(0.4);
            sleep(300);

            lf_drive.setPower(-0.4);
            rf_drive.setPower(-0.4);
            lb_drive.setPower(-0.4);
            rb_drive.setPower(-0.4);
            sleep(375);
            lf_drive.setPower(-0.4);
            rf_drive.setPower(0.4);
            lb_drive.setPower(-0.4);
            rb_drive.setPower(0.4);
            sleep(1000);
            lf_drive.setPower(-0.3);
            rf_drive.setPower(-0.3);
            lb_drive.setPower(-0.3);
            rb_drive.setPower(-0.3);
            sleep(1500);
            DESIRED_TAG_ID = 1;

            strafe_right(900);
            move_forward(790);
            rotate_right90();
            strafe_left(300);
            move_forward(450);
            pause(1);

            arm.setPower(0.5);
            sleep(100);
            arm.setPower(0);
            grip.setPosition(0.55);
        }

    }   // end method telemetryTfod()

    //-----------------------------------------------------------
    // Set the active camera according to input from the gamepad.
    //-----------------------------------------------------------
    private void doCameraSwitching() {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            if (TFdone) {
                visionPortal.setActiveCamera(webcamAT);
            }
            if (ATdone) {
                visionPortal.setActiveCamera(webcamTF);
            }
        }
    }   // end method doCameraSwitching()

    public void pause(long Sleep) {
        lf_drive.setPower(0);
        rf_drive.setPower(0);
        lb_drive.setPower(0);
        rb_drive.setPower(0);
        sleep(Sleep);
    }

    public void move_forward(long Sleep) {
        lf_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        sleep(Sleep);
    }

    public void move_backward(long Sleep) {
        lf_drive.setPower(-0.5);
        rf_drive.setPower(-0.5);
        lb_drive.setPower(-0.5);
        rb_drive.setPower(-0.5);
        sleep(Sleep);
    }

    public void strafe_left(long Sleep) {
        lf_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        lb_drive.setPower(-0.5);
        rb_drive.setPower(-0.5);
        sleep(Sleep);
    }

    public void strafe_right(long Sleep) {
        lf_drive.setPower(-0.5);
        rf_drive.setPower(-0.5);
        lb_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        sleep(Sleep);
    }

    public void rotate_left(long Sleep) {
        lf_drive.setPower(-0.5);
        rf_drive.setPower(0.5);
        lb_drive.setPower(-0.5);
        rb_drive.setPower(0.5);
        sleep(Sleep);
    }

    public void rotate_right(long Sleep) {
        lf_drive.setPower(0.5);
        rf_drive.setPower(-0.5);
        lb_drive.setPower(0.5);
        rb_drive.setPower(-0.5);
        sleep(Sleep);
    }
    public void rotate_right90() {
        lf_drive.setPower(0.5);
        rf_drive.setPower(-0.5);
        lb_drive.setPower(0.5);
        rb_drive.setPower(-0.5);
        sleep(725);
    }
    public void rotate_left90() {
        lf_drive.setPower(-0.5);
        rf_drive.setPower(0.5);
        lb_drive.setPower(-0.5);
        rb_drive.setPower(0.5);
        sleep(725);
    }
}
