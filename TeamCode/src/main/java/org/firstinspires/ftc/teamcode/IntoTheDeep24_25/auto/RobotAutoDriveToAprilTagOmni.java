package org.firstinspires.ftc.teamcode.IntoTheDeep24_25.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/* This is an autonomous OpMode for FTC robots using Mecanum drive, designed to autonomously drive
 * towards an AprilTag based on its distance, yaw, and bearing. The robot will automatically adjust
 * its motion until it reaches a desired distance from the detected AprilTag.
 */


  @Autonomous(name = "Autonomous Drive to AprilTag", group = "Autonomous")
  public class RobotAutoDriveToAprilTagOmni extends LinearOpMode {
    // Constants for driving towards the AprilTag
    final double DESIRED_DISTANCE = 12.0; // Target distance in inches from AprilTag
    final double SPEED_GAIN  =  0.02;     // Forward speed gain
    final double STRAFE_GAIN =  0.015;    // Strafe speed gain
    final double TURN_GAIN   =  0.01;     // Turn speed gain

    final double MAX_AUTO_SPEED = 0.5;    // Maximum forward speed
    final double MAX_AUTO_STRAFE = 0.5;   // Maximum strafe speed
    final double MAX_AUTO_TURN = 0.3;     // Maximum turn speed

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1; // Track any detected AprilTag

    @Override
    public void runOpMode() {

        // Initialize the AprilTag detection system and hardware map
        initAprilTag();

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set exposure for webcam to reduce motion blur
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        // Wait for the autonomous phase to begin
        telemetry.addData("Status", "Waiting for start...");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean targetFound = false;
            desiredTag = null;

            // Check for AprilTag detections
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                }
            }

            if (targetFound) {
                // Calculate distance, yaw, and bearing errors
                double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                double yawError = desiredTag.ftcPose.yaw;
                double headingError = desiredTag.ftcPose.bearing;

                // Calculate drive, strafe, and turn speeds based on the errors
                double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("Driving", "Drive %5.2f, Strafe %5.2f, Turn %5.2f", drive, strafe, turn);
                telemetry.update();

                // Move the robot according to calculated values
                moveRobot(drive, strafe, turn);
            } else {
                telemetry.addData("Status", "No AprilTag detected. Searching...");
                telemetry.update();
                // Stop the robot if no target is found
                moveRobot(0, 0, 0);
            }

            sleep(10); // Small delay to allow smooth updates
        }
    }

    // Method to move the robot based on calculated speeds
    public void moveRobot(double x, double y, double yaw) {
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        double maxPower = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(leftBackPower));
        maxPower = Math.max(maxPower, Math.abs(rightBackPower));

        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    // Initialize the AprilTag detection system
    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

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

    // Method to set manual exposure for reducing motion blur
    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}