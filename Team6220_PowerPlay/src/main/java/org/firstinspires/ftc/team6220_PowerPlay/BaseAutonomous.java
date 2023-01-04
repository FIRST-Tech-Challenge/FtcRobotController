package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Locale;

public abstract class BaseAutonomous extends BaseOpMode {
    public TFObjectDetector tfod;
    public VuforiaLocalizer vuforia;

    public void initialize() {
        super.initialize();
        initializeVuforia();
        initializeTensorFlow();
    }

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
        double xPower = Math.cos(Math.toRadians(heading + 95)) * 0.6;
        double yPower = Math.sin(Math.toRadians(heading + 95)) * 0.6;

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

            motorBR.setVelocity(100, AngleUnit.RADIANS);

            turningPower = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - startAngle) * 0.02;

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

        while (Math.abs(angleError) >= 1 && opModeIsActive()) {
            currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            angleError = targetAngle - currentAngle + startAngle;

            if (angleError > 180.0) {
                angleError -= 360.0;
            } else if (angleError < -180.0) {
                angleError += 360.0;
            }

            // robot is turning counter-clockwise
            if (angleError > 0) {
                motorPower = Math.min(angleError / -250.0, -0.05);

            // robot is turning clockwise
            } else {
                motorPower = Math.max(angleError / -250.0, 0.05);
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
        while (Math.abs(error) > 50 && opModeIsActive()) {
            error = targetPosition - motorLeftSlides.getCurrentPosition();
            motorPower = error * 0.01;

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

        motorLeftSlides.setPower(0.05);
        motorRightSlides.setPower(0.05);
    }

    // detect signal on signal sleeve
    public int detectSignal() {
        OpenCvCamera robotCamera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        // units are pixels
        // calibration is for Logitech C920 webcam at 1920 x 1080
        final double fx = 1385.92; // focal length x
        final double fy = 1385.92; // focal length y
        final double cx = 951.982; // camera principal point x
        final double cy = 534.084; // camera principal point y

        // units are meters
        final double tagSize = 0.03429;

        final int ID_TAG_OF_INTEREST_0 = 0; // tag 0 from the 36h11 family
        final int ID_TAG_OF_INTEREST_1 = 1; // tag 1 from the 36h11 family
        final int ID_TAG_OF_INTEREST_2 = 2; // tag 2 from the 36h11 family

        ArrayList<AprilTagDetection> currentDetections;
        AprilTagDetection tagOfInterest = null;
        boolean tagFound = false;

        // initializes camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        robotCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "RobotCamera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        robotCamera.setPipeline(aprilTagDetectionPipeline);
        robotCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robotCamera.startStreaming(1920, 1080, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
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

    // initialize the Vuforia localization engine
    private void initializeVuforia() {
        // configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;

        WebcamName robotCamera = hardwareMap.get(WebcamName.class, "RobotCamera");
        WebcamName grabberCamera = hardwareMap.get(WebcamName.class, "GrabberCamera");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(robotCamera, grabberCamera);

        // instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // set the active camera to the robot camera
        SwitchableCamera camera = (SwitchableCamera) vuforia.getCamera();
        camera.setActiveCamera(robotCamera);
    }

    // initialize the TensorFlow object detection engine
    private void initializeTensorFlow() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(Constants.TENSORFLOW_MODEL_ASSET, Constants.TENSORFLOW_LABELS);
    }
}
