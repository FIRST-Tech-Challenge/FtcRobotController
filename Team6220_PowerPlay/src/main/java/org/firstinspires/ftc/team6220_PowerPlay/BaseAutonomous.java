package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Locale;

public abstract class BaseAutonomous extends BaseOpMode {
    /**
     * this method will allow the robot to drive straight in a specified direction given a specified heading and distance
     * @param driveCourse 360-degree direction robot should drive (front is 0)
     * @param targetDistance distance robot should move in inches
     */
    public void driveAutonomous(double driveCourse, double targetDistance) {
        // encoder values of the drive motors
        int eFL, eFR, eBL, eBR;

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double currentAngle;
        double tPower;

        // looking at robot from the back, x is left/right and y is forwards/backwards
        double xPosition, yPosition;

        // power for any heading
        double xPower;
        double yPower;

        double traveledDistance;
        double remainingDistance = targetDistance;

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (remainingDistance > 0 && opModeIsActive()) {
            eFL = motorFL.getCurrentPosition();
            eFR = motorFR.getCurrentPosition();
            eBL = motorBL.getCurrentPosition();
            eBR = motorBR.getCurrentPosition();

            // robot turns slightly to correct for small changes in heading
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            tPower = (currentAngle - startAngle) * Constants.HEADING_CORRECTION_KP_AUTONOMOUS;

            // vector rotation to make robot capable of moving in any direction while maintaining heading
            xPower = Math.cos(Math.toRadians(driveCourse + Constants.UNIT_CIRCLE_OFFSET_DEGREES)) * Constants.MAXIMUM_DRIVE_POWER_AUTONOMOUS;
            yPower = Math.sin(Math.toRadians(driveCourse + Constants.UNIT_CIRCLE_OFFSET_DEGREES)) * Constants.MAXIMUM_DRIVE_POWER_AUTONOMOUS;

            // gives a power to each motor to make the robot move in the specified direction
            motorFL.setPower(yPower + xPower + tPower);
            motorFR.setPower(yPower - xPower - tPower);
            motorBL.setPower(yPower - xPower + tPower);
            motorBR.setPower(yPower + xPower - tPower);

            // calculates x-position and y-position based on drive encoder ticks
            xPosition = (eFL - eFR - eBL + eBR) * Constants.DRIVE_MOTOR_TICKS_TO_INCHES * 0.25;
            yPosition = (eFL + eFR + eBL + eBR) * Constants.DRIVE_MOTOR_TICKS_TO_INCHES * 0.25;

            // calculates traveled and remaining distance using the pythagorean theorem
            traveledDistance = Math.sqrt(Math.pow(xPosition, 2) + Math.pow(yPosition, 2));
            remainingDistance = targetDistance - traveledDistance;
        }

        stopDriveMotors();
    }

    /**
     * allows the robot to turn to a specified absolute angle using the IMU
     * @param targetHeading absolute angle robot should turn to
     */
    public void turnToAngle(double targetHeading) {
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleError = targetHeading - currentAngle + startAngle;
        double motorPower;

        // while robot hasn't reached target heading
        while (Math.abs(angleError) >= Constants.ROBOT_HEADING_TOLERANCE_DEGREES && opModeIsActive()) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            angleError = targetHeading - currentAngle + startAngle;

            // prevents angle from gong above 180 degrees and below -180 degrees
            // makes sure robot takes most optimal path to get to the target heading
            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            // proportional motor power based on angle error
            motorPower = angleError * -Constants.TURNING_KP;
            motorPower = Math.max(Math.min(Math.abs(motorPower), Constants.MAXIMUM_TURN_POWER_AUTONOMOUS), Constants.MINIMUM_TURN_POWER) * Math.signum(motorPower);

            // gives a power to each motor to make the robot pivot
            motorFL.setPower(motorPower);
            motorFR.setPower(-motorPower);
            motorBL.setPower(motorPower);
            motorBR.setPower(-motorPower);
        }

        stopDriveMotors();
    }

    /**
     * allows the slides to move to a specified target position - for autonomous only
     * @param targetPosition target position for slides motors in ticks
     */
    public void driveSlidesAutonomous(int targetPosition) {
        long startTime = System.currentTimeMillis();
        int error = targetPosition - motorLeftSlides.getCurrentPosition();
        double motorPower;

        // while slides aren't at target position
        while (Math.abs(error) > Constants.ROBOT_SLIDE_TOLERANCE_TICKS && opModeIsActive()) {
            error = targetPosition - motorLeftSlides.getCurrentPosition();
            motorPower = error * Constants.SLIDE_MOTOR_KP;

            // slides going down - full speed
            if (error < 0) {
                motorLeftSlides.setPower(-1.0);
                motorRightSlides.setPower(-1.0);
            // slides going up - proportional control
            } else {
                motorLeftSlides.setPower(motorPower);
                motorRightSlides.setPower(motorPower);
            }

            long currentTime = System.currentTimeMillis();

            // breaks out of the while loop if it has been running for 3 seconds
            if (currentTime - startTime > 3000) {
                break;
            }
        }

        // feedforward constant to counteract gravity
        motorLeftSlides.setPower(Constants.SLIDE_FEEDFORWARD);
        motorRightSlides.setPower(Constants.SLIDE_FEEDFORWARD);
    }

    // detects signal on signal sleeve using april tags
    public int detectSignal() {
        final int ID_TAG_OF_INTEREST_0 = 0; // tag 0 from the 36h11 family
        final int ID_TAG_OF_INTEREST_1 = 1; // tag 1 from the 36h11 family
        final int ID_TAG_OF_INTEREST_2 = 2; // tag 2 from the 36h11 family

        ArrayList<AprilTagDetection> currentDetections;
        AprilTagDetection tagOfInterest = null;
        boolean tagFound = false;

        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        robotCamera.setPipeline(aprilTagDetectionPipeline);

        // replaces waitForStart()
        // detects AprilTags during initialization
        while (!isStarted() && !isStopRequested()) {
            currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            // tag has been detected at one point
            if (currentDetections.size() != 0) {

                // finds out which tag is detected
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == ID_TAG_OF_INTEREST_0 || tag.id == ID_TAG_OF_INTEREST_1 || tag.id == ID_TAG_OF_INTEREST_2) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                // tag has been detected and is still being detected
                // displays tag details
                if (tagFound) {
                    telemetry.addLine("tag found!\n\nlocation data:\n");
                    telemetry.addLine(String.format(Locale.US, "Detected tag ID: %d", tagOfInterest.id));
                    telemetry.addLine(String.format(Locale.US, "X distance: %d inches", (int) (tagOfInterest.pose.x * Constants.INCHES_PER_METER)));
                    telemetry.addLine(String.format(Locale.US, "Y Distance: %d inches", (int) (tagOfInterest.pose.y * Constants.INCHES_PER_METER)));
                    telemetry.addLine(String.format(Locale.US, "Z Distance: %d inches", (int) (tagOfInterest.pose.z * Constants.INCHES_PER_METER)));
                    telemetry.addLine(String.format(Locale.US, "Yaw Rotation: %d degrees", (int) (Math.toDegrees(tagOfInterest.pose.yaw))));
                    telemetry.addLine(String.format(Locale.US, "Pitch Rotation: %d degrees", (int) (Math.toDegrees(tagOfInterest.pose.pitch))));
                    telemetry.addLine(String.format(Locale.US, "Roll Rotation: %d degrees", (int) (Math.toDegrees(tagOfInterest.pose.roll))));
                }

                // tag has never been detected
            } else {
                telemetry.addLine("can't see tag of interest :(\n");
                telemetry.addLine("the tag has never been seen");
            }

            telemetry.update();
        }

        robotCamera.stopStreaming();

        // return default
        if (tagOfInterest == null) {
            return 1;

        // return detected tag
        } else {
            return tagOfInterest.id;
        }
    }

    /**
     * Starts streaming a camera and sets a pipeline to it, this is a method now because it will have to be called multiple times in any auton class where pipeline switching is used
     * @param pipeline pipeline that the method inputs, any pipeline-specific methods will have to be called separately
     * @param camera camera that the method inputs
     * @param X desired camera width (in pixels)
     * @param Y desired camera height (in pixels)
     */
    public void startCameraWithPipeline(OpenCvPipeline pipeline, OpenCvCamera camera, int X, int Y){
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(X, Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        camera.setPipeline(pipeline);
    }
}