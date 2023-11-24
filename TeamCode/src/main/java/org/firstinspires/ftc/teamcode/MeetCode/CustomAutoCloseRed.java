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

package org.firstinspires.ftc.teamcode.MeetCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
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
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@Autonomous(name = "Auto Red Close", group = "Concept")

public class CustomAutoCloseRed extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "RedProp.tflite";


    String position = "";

    final double DESIRED_DISTANCE = 7.5; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.02;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.02;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.5;

    private  int DESIRED_TAG_ID = -1;

    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  drive           = 0;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)

    String park = "";

    boolean indicator = false;
    private static final String[] LABELS = {
            "RedProp"

    };
    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    Hardware robot = new Hardware();

    @Override
    public void runOpMode() {





        initTfod();
        robot.init(hardwareMap);
        robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm.setTargetPosition(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(.3);
        robot.wrist.setPosition(.55);
        robot.claw.setPosition(0);





        // Wait for the DS start button to be touched.
        //tfod.setZoom(.5);
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        while (park.equals("")) {
            if (gamepad1.left_bumper) {
                park = "Left";
            } else if (gamepad1.right_bumper) {
                park = "Right";
            }

            telemetry.addData("Park Location", park);
            telemetry.update();
        }

        waitForStart();
        telemetry.clear();

        robot.timer.reset();
        while (robot.timer.seconds() < .75) {
            telemetryTfod();
        }
        robot.encoderStrafeLeft(5);
        if(position.equals("Center")) {
            DESIRED_TAG_ID = 5;
            robot.encoderDrive(30.5);
            sleep(100);
            robot.encoderStrafeRight(3);
            robot.dropper.setPosition(1);
            sleep(250);
            robot.encoderDrive(-6);
            sleep(100);
            robot.encoderTurnRight(23);
            //robot.squareUp();
            sleep(50);
            robot.turnOffEncoders();
            setManualExposure(6, 250);
            robot.timer.reset();
            while (robot.timer.seconds() < 3.5){
                aprilTagDetection();
            }
            visionPortal.close();
            robot.timer.reset();
            while (robot.timer.seconds() < 1) {
                robot.cascadeDrive(1350);
            }
            robot.arm.setTargetPosition(438);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            while (robot.arm.getCurrentPosition() - robot.arm.getTargetPosition() != 0 && robot.timer.seconds() < 1) {
                robot.arm.setPower(.4);
                robot.cascadeDrive(1350);
            }
            robot.claw.setPosition(.4);
            sleep(500);
            robot.encoderDrive(-2);
            robot.claw.setPosition(0);
            robot.wrist.setPosition(1);
            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            while (robot.arm.getCurrentPosition() - robot.arm.getTargetPosition() != 0 && robot.timer.seconds() < 1) {
                robot.arm.setPower(.4);
                robot.cascadeDrive(1350);
            }
            sleep(500);
            robot.cascadeDrive(0);
            if (park.equals("Left")) {
                robot.encoderStrafeLeft(30);
                robot.encoderDrive(18);
            }
            else if (park.equals("Right")){
                robot.encoderStrafeRight(30);
                robot.encoderDrive(18);
            }

        }
        else if(position.equals("Right")){
            DESIRED_TAG_ID = 6;
            robot.encoderDrive(26);
            sleep(100);
            robot.encoderStrafeRight(13);
            robot.dropper.setPosition(1);
            sleep(250);
            robot.encoderDrive(-7.5);
            robot.encoderTurnRight(23);
            //robot.squareUp();
            sleep(50);
            robot.turnOffEncoders();
            setManualExposure(6, 250);
            robot.timer.reset();
            while (robot.timer.seconds() < 3.5){
                aprilTagDetection();
            }
            visionPortal.close();
            robot.timer.reset();
            while (robot.timer.seconds() < 1) {
                robot.cascadeDrive(1350);
            }
            robot.arm.setTargetPosition(438);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            while (robot.arm.getCurrentPosition() - robot.arm.getTargetPosition() != 0 && robot.timer.seconds() < 1) {
                robot.arm.setPower(.4);
                robot.cascadeDrive(1350);
            }
            robot.claw.setPosition(.4);
            sleep(500);
            robot.encoderDrive(-2);
            robot.claw.setPosition(0);
            robot.wrist.setPosition(1);
            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            while (robot.arm.getCurrentPosition() - robot.arm.getTargetPosition() != 0 && robot.timer.seconds() < 1) {
                robot.arm.setPower(.4);
                robot.cascadeDrive(1350);
            }
            sleep(500);
            robot.cascadeDrive(0);
            if (park.equals("Left")) {
                robot.encoderStrafeLeft(36);
                robot.encoderDrive(18);
            }
            else if (park.equals("Right")){
                robot.encoderStrafeRight(24);
                robot.encoderDrive(18);
            }

            //23, 9, 1, -.5, 65

        }
        else{
            DESIRED_TAG_ID = 4;
            robot.encoderDrive(28.5);
            sleep(100);
            robot.encoderStrafeLeft(14.5);
            robot.dropper.setPosition(1);
            sleep(100);
            robot.encoderDrive(-2);
            sleep(250);
            robot.encoderStrafeRight(15);
            robot.encoderTurnRight(21);
            //robot.squareUp();
            sleep(50);
            robot.turnOffEncoders();
            setManualExposure(6, 250);
            robot.timer.reset();
            while (robot.timer.seconds() < 3.5){
                aprilTagDetection();
            }
            visionPortal.close();
            robot.timer.reset();
            while (robot.timer.seconds() < 1) {
                robot.cascadeDrive(1350);
            }
            robot.arm.setTargetPosition(438);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            while (robot.arm.getCurrentPosition() - robot.arm.getTargetPosition() != 0 && robot.timer.seconds() < 1) {
                robot.arm.setPower(.4);
                robot.cascadeDrive(1350);
            }
            robot.claw.setPosition(.4);
            sleep(500);
            robot.encoderDrive(-2);
            robot.claw.setPosition(0);
            robot.wrist.setPosition(1);
            robot.arm.setTargetPosition(0);
            robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.timer.reset();
            while (robot.arm.getCurrentPosition() - robot.arm.getTargetPosition() != 0 && robot.timer.seconds() < 1) {
                robot.arm.setPower(.4);
                robot.cascadeDrive(1350);
            }
            sleep(500);
            robot.cascadeDrive(0);
            if (park.equals("Left")) {
                robot.encoderStrafeLeft(24);
                robot.encoderDrive(18);
            }
            else if (park.equals("Right")){
                robot.encoderStrafeRight(36);
                robot.encoderDrive(18);
            }

            //21.5, 15, 47
        }


        // Push telemetry to the Driver Station.


        // Save CPU resources; can resume streaming when needed.

        // Share the CPU.
        sleep(20);

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end runOpMode()

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
//            .setIsModelTensorFlow2(true)
//            .setIsModelQuantized(true)
//            .setModelInputSize(300)
//            .setModelAspectRatio(16.0 / 9.0)

                .build();

        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();


        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            telemetry.update();
            if(x > 400){
                position = "Right";
                break;
            }
            else if(x > 50 && x < 300){
                position = "Center";
                break;
            }
            else{
                position = "Left";
                break;
            }

        }   // end for() loop

    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robot.frontLeft.setPower(rightBackPower);
        robot.frontRight.setPower(leftBackPower);
        robot.backLeft.setPower(rightFrontPower);
        robot.backRight.setPower(leftFrontPower);

        /*if (desiredTag.ftcPose.range > DESIRED_DISTANCE) {
            while (desiredTag.ftcPose.range > DESIRED_DISTANCE) {
                robot.setPowerOfAllMotorsTo(-.3);
            }
            robot.setPowerOfAllMotorsTo(0);
        } else if (desiredTag.ftcPose.range < DESIRED_DISTANCE) {
            while (desiredTag.ftcPose.range < DESIRED_DISTANCE) {
                robot.setPowerOfAllMotorsTo(.3);
            }
            robot.setPowerOfAllMotorsTo(0);
        }*/
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();

        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
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
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
    public void  aprilTagDetection(){
        targetFound = false;
        desiredTag = null;

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID)) {
                targetFound = true;
                desiredTag = detection;

                break;  // don't look any further.
            }
        }
        // Tell the driver what we see, and what to do.
        // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
        if (targetFound) {

            // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else{
            // Use the speed and turn "gains" to calculate how we want the robot to move.
            drive = 0;
            turn = 0;
            strafe = 0;
            telemetry.addLine("Target not found");
        }
        telemetry.update();
        moveRobot(drive, strafe, turn);
        sleep(10);

        // Apply desired axes motions to the drivetrain
    }

}   // end class
