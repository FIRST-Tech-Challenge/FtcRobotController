package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Locale;

public abstract class BaseAutonomous extends BaseOpMode {
    /**
     * this method will allow the robot to drive straight in a specified direction given a specified heading and distance
     * @param heading 360-degree direction robot should move (front is 0)
     * @param targetDistance distance robot should move in inches
     */
    public void driveInches(double heading, double targetDistance) {
        // encoder values of the drive motors
        int eFL, eFR, eBL, eBR;

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double turningPower;

        // looking at robot from the back, x is left/right and y is forwards/backwards
        double xPosition, yPosition;

        // power for any heading
        double xPower = Math.cos(Math.toRadians(heading + 95)) * Constants.MAXIMUM_DRIVE_POWER;
        double yPower = Math.sin(Math.toRadians(heading + 95)) * Constants.MAXIMUM_DRIVE_POWER;

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

            turningPower = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle) * Constants.HEADING_CORRECTION_KP;

            motorFL.setPower(yPower + xPower + turningPower);
            motorFR.setPower(yPower - xPower - turningPower);
            motorBL.setPower(yPower - xPower + turningPower);
            motorBR.setPower(yPower + xPower - turningPower);

            xPosition = (eFL - eFR - eBL + eBR) * Constants.DRIVE_MOTOR_TICKS_TO_INCHES * 0.25;
            yPosition = (eFL + eFR + eBL + eBR) * Constants.DRIVE_MOTOR_TICKS_TO_INCHES * 0.25;

            traveledDistance = Math.sqrt(Math.pow(xPosition, 2) + Math.pow(yPosition, 2));
            remainingDistance = targetDistance - traveledDistance;
        }

        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    /**
     * this method will allow the robot to turn to a specified absolute angle using the IMU
     * @param targetAngle absolute angle robot should turn to
     */
    public void turnToAngle(double targetAngle) {
        double currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleError = targetAngle - currentAngle + startAngle;
        double motorPower;

        while (Math.abs(angleError) >= Constants.ROBOT_HEADING_TOLERANCE_DEGREES && opModeIsActive()) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            angleError = targetAngle - currentAngle + startAngle;

            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            // robot is turning counter-clockwise
            if (angleError > 0) {
                motorPower = Math.min(angleError / -250.0, -Constants.MINIMUM_TURNING_POWER);

            // robot is turning clockwise
            } else {
                motorPower = Math.max(angleError / -250.0, Constants.MINIMUM_TURNING_POWER);
            }

            motorFL.setPower(motorPower);
            motorFR.setPower(-motorPower);
            motorBL.setPower(motorPower);
            motorBR.setPower(-motorPower);

            telemetry.addData("current", currentAngle);
            telemetry.addData("error", angleError);
            telemetry.addData("power", motorPower);
            telemetry.update();
        }

        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    /**
     * this method will allow the slides to move to a specified target position
     * @param targetPosition target position for slides motors in ticks
     */
    public void driveSlidesAutonomous(int targetPosition) {
        int error = targetPosition - motorLeftSlides.getCurrentPosition();
        double motorPower;

        // while slides aren't at target position
        while (Math.abs(error) > Constants.ROBOT_SLIDE_TOLERANCE_TICKS && opModeIsActive()) {
            error = targetPosition - motorLeftSlides.getCurrentPosition();
            motorPower = error * Constants.SLIDE_MOTOR_KP;

            // slides going down - full speed
            if (error < 0) {
                motorLeftSlides.setPower(-0.75);
                motorRightSlides.setPower(-0.75);
            // slides going up - proportional control
            } else {
                motorLeftSlides.setPower(motorPower);
                motorRightSlides.setPower(motorPower);
            }
        }

        motorLeftSlides.setPower(Constants.SLIDE_FEEDFORWARD);
        motorRightSlides.setPower(Constants.SLIDE_FEEDFORWARD);
    }

    // detect signal on signal sleeve
    public int detectSignal() {
        OpenCvCamera robotCamera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        final int ID_TAG_OF_INTEREST_0 = 0; // tag 0 from the 36h11 family
        final int ID_TAG_OF_INTEREST_1 = 1; // tag 1 from the 36h11 family
        final int ID_TAG_OF_INTEREST_2 = 2; // tag 2 from the 36h11 family

        ArrayList<AprilTagDetection> currentDetections;
        AprilTagDetection tagOfInterest = null;
        boolean tagFound = false;

        // initializes camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline();

        robotCamera.setPipeline(aprilTagDetectionPipeline);
        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

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

    // moves towards a stack while centering on it, until the stack fills the entire camera view
    // puts the robot about 3 inches away from the junction based on manual testing
    public void centerRobotCamera(RobotCameraPipeline pipeline, int maxWidth) {
        double xOffset, detectionWidth;

        do {
            xOffset = pipeline.xPosition - Constants.CAMERA_CENTER_X;
            detectionWidth = pipeline.detectionWidth;

            // convert the width to motor power to drive forward with
            driveWithIMU(Constants.CONE_CENTERING_KP, 0.3, 0.0);

            telemetry.addData("xPosition", pipeline.xPosition);
            telemetry.addData("yPosition", pipeline.yPosition);
            telemetry.addData("width", detectionWidth);
            telemetry.update();

        // while either not centered in front of stack, or not close enough that the stack fills the view
        } while (detectionWidth < maxWidth);

        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }
}
